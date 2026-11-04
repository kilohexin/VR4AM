from scripts.audit_pose_trace import analyze


def trace(zs):
    rows=[dict(kind='robot_kinematics',server_mono_ns=0,actual_tcp={'p':[0,0,.2]})]
    for i,z in enumerate(zs,1):
        t=i*100_000_000
        rows.extend([dict(kind='finite_position_input',event='held',server_mono_ns=t,hand_pose={'p':[0,0,0]}),
            dict(kind='robot_kinematics',server_mono_ns=t,actual_tcp={'p':[0,0,z]})])
    return rows


def test_stable_height_drop_is_a_failed_cartesian_regression():
    r=analyze(trace([.2,.19,.1,.01]))
    assert r['stable_height_drop_observed'] and r['max_downward_error_m']>.18
    assert r['sdk_connections']==r['motion_writes']==0 and not r['motion_safety_verified']


def test_no_observed_violation_is_not_a_safety_certificate():
    r=analyze(trace([.2,.201,.199,.2]))
    assert not r['stable_height_drop_observed'] and not r['motion_safety_verified']
