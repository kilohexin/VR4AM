"""Clutch-relative XR pose math; no robot or dispatcher access."""
from scipy.spatial.transform import Rotation
from app.schemas.messages import Pose
from app.control.coordinate_mapper import CoordinateMapper


class VRPoseReference:
    # XR right/up/back -> controller right/forward/up, a proper rotation.
    axes=Rotation.from_matrix(((1.,0.,0.),(0.,0.,-1.),(0.,1.,0.)))
    # WebXR held-rod forward is -Z; the gripper tool points along +Z.
    # A proper basis rotation preserves Y and reverses X/Z, without Euler
    # component sign edits or a discontinuity when pitch crosses zero.
    grip_to_tool=Rotation.from_euler('y',180.,degrees=True)

    def __init__(self,*,clock,translation_scale=2.,orientation_mode='world'):
        if type(translation_scale) not in (int,float) or not 0<translation_scale<=5:
            raise ValueError('vr_pose_scale')
        self.clock=clock
        if orientation_mode not in ('world','tool'):raise ValueError('vr_pose_orientation_mode')
        self.orientation_mode=orientation_mode
        self._hand_rotation=self._tool_rotation=None
        self.mapper=CoordinateMapper(translation_scale=translation_scale,rotation_dead_zone_deg=0.)
        self.generation=None;self.seq=-1;self.stamp=-1

    def _mapped(self,hand):
        p=self.axes.apply(hand.p)
        q=(self.axes*Rotation.from_quat(hand.q)).as_quat()
        return Pose(p=tuple(p),q=tuple(q))

    def capture(self,hand,tcp,*,generation,hand_ns,tcp_ns):
        self.release()
        now=self.clock()
        if (type(generation) is not int or generation<0 or type(hand_ns) is not int
                or type(tcp_ns) is not int or not 0<=now-hand_ns<100_000_000
                or not 0<=now-tcp_ns<100_000_000):raise ValueError('vr_pose_capture_expired')
        self.mapper.capture(self._mapped(hand),tcp)
        self._hand_rotation=Rotation.from_quat(hand.q)
        self._tool_rotation=Rotation.from_quat(tcp.q)
        self.generation=generation
        self.stamp=hand_ns

    def release(self):
        self.mapper.clear();self.generation=None;self.seq=-1;self.stamp=-1
        self._hand_rotation=self._tool_rotation=None

    def target(self,hand,*,generation,seq,captured_ns):
        if not self.mapper.has_anchor:raise ValueError('vr_pose_requires_capture')
        if generation!=self.generation or type(generation) is not int:
            self.release();raise ValueError('vr_pose_generation_changed')
        if type(captured_ns) is not int or not 0<=self.clock()-captured_ns<100_000_000:
            self.release();raise ValueError('vr_pose_input_expired')
        if type(seq) is not int or seq<=self.seq or captured_ns<self.stamp:raise ValueError('vr_pose_input_order')
        self.seq=seq;self.stamp=captured_ns
        goal=self.mapper.target(self._mapped(hand))
        if self.orientation_mode=='tool':
            delta=self._hand_rotation.inv()*Rotation.from_quat(hand.q)
            rotation=self._tool_rotation*self.grip_to_tool*delta*self.grip_to_tool.inv()
            goal=goal.model_copy(update={'q':tuple(float(v) for v in rotation.as_quat())})
        return goal
