"""Clutch-relative XR pose math; no robot or dispatcher access."""
from scipy.spatial.transform import Rotation
from app.schemas.messages import Pose
from app.control.coordinate_mapper import CoordinateMapper


class VRPoseReference:
    # XR right/up/back -> controller right/forward/up, a proper rotation.
    axes=Rotation.from_matrix(((1.,0.,0.),(0.,0.,-1.),(0.,1.,0.)))

    def __init__(self,*,clock,translation_scale=2.):
        if type(translation_scale) not in (int,float) or not 0<translation_scale<=5:
            raise ValueError('vr_pose_scale')
        self.clock=clock
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
        self.generation=generation
        self.stamp=hand_ns

    def release(self):
        self.mapper.clear();self.generation=None;self.seq=-1;self.stamp=-1

    def target(self,hand,*,generation,seq,captured_ns):
        if not self.mapper.has_anchor:raise ValueError('vr_pose_requires_capture')
        if generation!=self.generation or type(generation) is not int:
            self.release();raise ValueError('vr_pose_generation_changed')
        if type(captured_ns) is not int or not 0<=self.clock()-captured_ns<100_000_000:
            self.release();raise ValueError('vr_pose_input_expired')
        if type(seq) is not int or seq<=self.seq or captured_ns<self.stamp:raise ValueError('vr_pose_input_order')
        self.seq=seq;self.stamp=captured_ns
        return self.mapper.target(self._mapped(hand))
