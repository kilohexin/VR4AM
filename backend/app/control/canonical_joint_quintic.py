"""Portable binary64 operation order for compact owned-tail P/V/A.

No numpy solve, pow, fused multiply/add, actuator, clocks or admission. Each
generated 80ms piece still needs the existing full Cartesian validation.
The native implementation uses the same scalar operation order; the owner
retains EXACT equality checks rather than trusting a numerical tolerance.
"""
import math


def coefficients(first,last,duration):
    if type(duration) not in (int,float) or not math.isfinite(duration) or duration<=0:
        raise ValueError('canonical_quintic_duration')
    rows=[]
    for name in ('q','v','a'):
        if any(len(getattr(s,name))!=6 or not all(math.isfinite(x) for x in getattr(s,name)) for s in (first,last)):
            raise ValueError('canonical_quintic_state')
    for axis in range(6):
        p=first.q[axis];v=first.v[axis]*duration;a=first.a[axis]*duration*duration/2
        b0=last.q[axis]-p-v-a;b1=last.v[axis]*duration-v-2*a;b2=last.a[axis]*duration*duration-2*a
        row=(p,v,a,10*b0-4*b1+b2/2,-15*b0+7*b1-b2,6*b0-3*b1+b2/2)
        if not all(math.isfinite(x) for x in row):raise ValueError('canonical_quintic_nonfinite')
        rows.append(row)
    return tuple(rows)


def evaluate(c,time,duration):
    if (type(time) not in (int,float) or not math.isfinite(time)
            or not math.isfinite(duration) or duration<=0 or not 0<=time<=duration):
        raise ValueError('canonical_quintic_time')
    u=time/duration;q=[];v=[];a=[]
    for row in c:
        p=row[5];speed=5*row[5];acceleration=20*row[5]
        for degree in range(4,-1,-1):p=p*u+row[degree]
        for degree in range(4,0,-1):speed=speed*u+degree*row[degree]
        for degree in range(4,1,-1):acceleration=acceleration*u+degree*(degree-1)*row[degree]
        q.append(p);v.append(speed/duration);a.append(acceleration/duration/duration)
    result=tuple(q),tuple(v),tuple(a)
    if not all(math.isfinite(x) for row in result for x in row):raise ValueError('canonical_quintic_nonfinite')
    return result
