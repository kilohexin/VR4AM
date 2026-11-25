import bisect
import hashlib
import json
import math


def path_hash(points):
    return hashlib.sha256(json.dumps(points, sort_keys=True).encode()).hexdigest()


def pvat_resource(name, points):
    return dict(name=name, dir='', data=dict(kind='PVAT',data=[
        dict(duration=stop['t']-start['t'], joints=[
            dict(pose=q,velocity=v,acc=a) for q,v,a in zip(stop['q'],stop['v'],stop['a'],strict=True)
        ]) for start,stop in zip(points,points[1:])
    ]))


class Trajectory:
    def __init__(self, points):
        self.points = points
        self.times = [point["t"] for point in points]
        if len(points) < 2 or self.times[0] != 0 or any(
            not math.isfinite(t) or t <= before for before, t in zip(self.times, self.times[1:])
        ):
            raise ValueError("trajectory timestamps must start at zero and increase")
        if any(len(point[field]) != 6 or not all(map(math.isfinite, point[field]))
               for point in points for field in ("q", "v", "a")):
            raise ValueError("trajectory requires six finite position, velocity and acceleration values")
        self.coefficients = [self.segment(start, stop) for start, stop in zip(points, points[1:])]

    @staticmethod
    def segment(start, stop):
        h = stop["t"] - start["t"]
        coefficients = []
        for q0, v0, a0, q1, v1, a1 in zip(
            start["q"], start["v"], start["a"], stop["q"], stop["v"], stop["a"], strict=True
        ):
            c0, c1, c2 = q0, v0 * h, .5 * a0 * h * h
            dp, dv, da = q1 - c0 - c1 - c2, v1 * h - c1 - 2*c2, a1*h*h - 2*c2
            coefficients.append((c0, c1, c2, 10*dp-4*dv+.5*da, -15*dp+7*dv-da, 6*dp-3*dv+.5*da))
        return coefficients

    def sample(self, time):
        index = max(0, min(bisect.bisect_right(self.times, time) - 1, len(self.times) - 2))
        h = self.times[index + 1] - self.times[index]
        u = min(1, max(0, (time - self.times[index]) / h))
        q, v, a = [], [], []
        for c0, c1, c2, c3, c4, c5 in self.coefficients[index]:
            q.append(c0+u*(c1+u*(c2+u*(c3+u*(c4+u*c5)))))
            v.append((c1+u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5))))/h)
            a.append((2*c2+u*(6*c3+u*(12*c4+u*20*c5)))/(h*h))
        return q, v, a

    def on_controller_grid(self, period=.04, min_duration=0):
        steps=math.ceil(max(self.times[-1],min_duration)/period)
        duration=steps*period
        scale=duration/self.times[-1]
        result=[]
        for step in range(steps+1):
            time=round(step*period,9)
            q,v,a=self.sample(time/scale)
            result.append(dict(t=time,q=q,v=[x/scale for x in v],a=[x/(scale*scale) for x in a]))
        return Trajectory(result)
