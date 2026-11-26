"""Persistent ROS process inside the isolated MoveIt container."""

import json
import math
from pathlib import Path
import subprocess
import sys

import rclpy
import yaml
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, Constraints, JointConstraint, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from scene import FINGERS, GRIPPER_JOINT, GRIPPER_OPEN, JOINTS


def config(*, robot, srdf, limits):
    params = dict(robot_description=robot, robot_description_semantic=srdf,
                  robot_description_planning={'joint_limits':limits}, allow_trajectory_execution=False,
                  planning_pipelines=['ompl'], default_planning_pipeline='ompl',
                  ompl=dict(planning_plugins=['ompl_interface/OMPLPlanner'],
                            request_adapters=['default_planning_request_adapters/ResolveConstraintFrames',
                                              'default_planning_request_adapters/ValidateWorkspaceBounds',
                                              'default_planning_request_adapters/CheckStartStateBounds',
                                              'default_planning_request_adapters/CheckStartStateCollision'],
                            response_adapters=['default_planning_response_adapters/AddTimeOptimalParameterization',
                                               'default_planning_response_adapters/ValidateSolution'],
                            planner_configs={'RRTConnect':{'type':'geometric::RRTConnect','range':.18}},
                            manipulator=dict(planner_configs=['RRTConnect'],
                                             longest_valid_segment_fraction=.003,
                                             maximum_waypoint_distance=.012)))
    params['robot_description_kinematics'] = {'manipulator': dict(
        kinematics_solver='kdl_kinematics_plugin/KDLKinematicsPlugin',
        kinematics_solver_search_resolution=.001, kinematics_solver_timeout=.1)}
    path = Path('/tmp/lm3_moveit.yaml')
    path.write_text(yaml.safe_dump({'move_group':{'ros__parameters':params}}, width=120))
    return path


def pose(position, rotation=(0., 0., 0., 1.)):
    result = Pose()
    result.position.x, result.position.y, result.position.z = position
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = rotation
    return result


def collision(shape, frame='base_link'):
    cylinder = 'cylinder' in shape
    part = SolidPrimitive(type=SolidPrimitive.CYLINDER if cylinder else SolidPrimitive.BOX,
                          dimensions=shape['cylinder'] if cylinder else shape['size'])
    obj = CollisionObject(id=shape['id'], operation=CollisionObject.ADD)
    obj.header.frame_id = frame
    obj.primitives = [part]
    obj.primitive_poses = [pose(shape['center'], shape.get('rotation', (0., 0., 0., 1.)))]
    return obj


class WorkbenchPlanner:
    def invoke(self, client, request, seconds=8):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=seconds)
        if not future.done():
            raise TimeoutError(client.srv_name)
        return future.result()

    @staticmethod
    def points(path):
        order = [path.joint_names.index(name) for name in JOINTS]
        return [dict(t=p.time_from_start.sec+p.time_from_start.nanosec/1e9,
                     q=[p.positions[i] for i in order],v=[p.velocities[i] for i in order],
                     a=[p.accelerations[i] for i in order]) for p in path.points]

    def __init__(self, node, start, limits):
        self.node = node
        self.q = start
        self.jaw = GRIPPER_OPEN
        self.limits = limits
        self.ids = set()
        self.held = False
        self.scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.ik = node.create_client(GetPositionIK, '/compute_ik')
        self.motion = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.cartesian = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.validity = node.create_client(GetStateValidity, '/check_state_validity')
        publisher = node.create_publisher(JointState, '/joint_states', 10)
        self.timer = node.create_timer(.05, lambda: publisher.publish(self.state(self.q).joint_state))
        for service in (self.scene, self.ik, self.motion, self.cartesian, self.validity):
            if not service.wait_for_service(timeout_sec=95):
                raise RuntimeError(f'Unavailable MoveIt service: {service.srv_name}')

    def state(self, q):
        state = RobotState()
        state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        state.joint_state.name = [*JOINTS, GRIPPER_JOINT]
        state.joint_state.position = [*q, self.jaw]
        state.is_diff = True
        return state

    def update(self, req):
        self.q = req['start']
        self.jaw = req['jaw']
        scene = PlanningScene(is_diff=True)
        scene.robot_state = self.state(self.q)
        shapes = req['shapes']
        ids = {shape['id'] for shape in shapes}
        for name in self.ids-ids:
            obj = CollisionObject(id=name, operation=CollisionObject.REMOVE)
            obj.header.frame_id = 'base_link'
            scene.world.collision_objects.append(obj)
        for shape in shapes:
            scene.world.collision_objects.append(collision(shape))
        attached = req.get('held')
        if attached:
            item = AttachedCollisionObject(link_name='tool0', object=collision(attached, 'tool0'))
            item.touch_links = list(FINGERS)
            scene.robot_state.attached_collision_objects = [item]
        elif self.held:
            item = AttachedCollisionObject(link_name='tool0',
                object=CollisionObject(id='payload', operation=CollisionObject.REMOVE))
            scene.robot_state.attached_collision_objects = [item]
        if not self.invoke(self.scene, ApplyPlanningScene.Request(scene=scene)).success:
            raise RuntimeError('MoveIt rejected the current planning scene')
        self.ids, self.held = ids, bool(attached)

    def solve(self, req):
        self.update(req)
        target = pose(*req['goal'])
        start = self.state(req['start'])
        if req['cartesian']:
            query = GetCartesianPath.Request(start_state=start, group_name='manipulator', link_name='tool0',
                waypoints=[target], max_step=.005, avoid_collisions=True,
                max_velocity_scaling_factor=.55, max_acceleration_scaling_factor=.65)
            query.header.frame_id = 'base_link'
            result = self.invoke(self.cartesian, query)
            if result.error_code.val == 1 and result.fraction >= .999:
                return self.points(result.solution.joint_trajectory)
            if not req.get('global_fallback'):
                check = GetStateValidity.Request(robot_state=start,group_name='manipulator')
                validity = self.invoke(self.validity,check)
                contacts = [(c.contact_body_1,c.contact_body_2) for c in validity.contacts]
                raise RuntimeError(f'MoveIt Cartesian path: {result.fraction:.0%} ({result.error_code.val}); start contacts: {contacts}')
        query = GetPositionIK.Request()
        ik = query.ik_request
        ik.group_name, ik.ik_link_name, ik.robot_state, ik.avoid_collisions = 'manipulator', 'tool0', start, True
        ik.pose_stamped.header.frame_id = 'base_link'
        ik.pose_stamped.pose = target
        ik.timeout.nanosec = 350_000_000
        result = self.invoke(self.ik, query)
        if result.error_code.val != 1:
            raise RuntimeError(f'MoveIt IK: {result.error_code.val}')
        endpoint = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
        for name, initial in zip(JOINTS,req['start'],strict=True):
            q = endpoint[name]
            bounds = self.limits[name]
            compatible = [q+i*math.tau for i in range(-4,5)
                          if bounds['min_position'] <= q+i*math.tau <= bounds['max_position']]
            endpoint[name] = min(compatible,key=lambda value:abs(value-initial))
        query = GetMotionPlan.Request()
        motion = query.motion_plan_request
        motion.group_name, motion.planner_id, motion.pipeline_id = 'manipulator', 'RRTConnect', 'ompl'
        motion.start_state = start
        motion.allowed_planning_time, motion.num_planning_attempts = 2., 2
        motion.max_velocity_scaling_factor = .85
        motion.max_acceleration_scaling_factor = .85
        motion.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=endpoint[name],
                            tolerance_above=.0003, tolerance_below=.0003, weight=1.)
            for name in JOINTS])]
        result = self.invoke(self.motion, query)
        if result.motion_plan_response.error_code.val != 1:
            raise RuntimeError(f'MoveIt planning: {result.motion_plan_response.error_code.val}')
        return self.points(result.motion_plan_response.trajectory.joint_trajectory)


def send(**payload):
    print('LM3_WORKBENCH '+json.dumps(payload, allow_nan=False), flush=True)


def main():
    init = json.loads(sys.stdin.readline())
    init.pop('start')
    params = config(**init)
    log_path = Path('/tmp/workbench-moveit.log')
    with log_path.open('w',encoding='utf-8') as log:
        process = subprocess.Popen(['ros2','run','moveit_ros_move_group','move_group',
                                    '--ros-args','--params-file',str(params)], stdout=log, stderr=subprocess.STDOUT)
        rclpy.init()
        node = rclpy.create_node('lm3_grasp_workbench')
        try:
            planner = WorkbenchPlanner(node, [0.]*6, init['limits'])
            send(ready=True)
            for line in sys.stdin:
                try:
                    send(path=planner.solve(json.loads(line)))
                except (RuntimeError, ValueError, TimeoutError, KeyError) as error:
                    print(f'Planning request failed: {error}', file=sys.stderr, flush=True)
                    send(error=str(error))
        except RuntimeError:
            print(log_path.read_text(encoding='utf-8', errors='replace'), file=sys.stderr)
            raise
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(3)
            except subprocess.TimeoutExpired:
                process.kill()


if __name__=='__main__':
    main()
