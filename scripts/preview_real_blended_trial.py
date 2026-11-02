"""Offline contract preview. Intentionally has no SDK/execute/address option."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def preview():
    sources=('backend/app/commissioning/blended_trial.py',
        'backend/app/control/blended_position_session.py','backend/app/control/blended_position_gate.py',
        'backend/app/robots/blended_position_docker.py','backend/app/robots/blended_position_path.py',
        'backend/app/robots/finite_position_web.py','backend/app/robots/finite_position_adapter.py',
        'backend/app/robots/lebai_adapter.py','scripts/preview_real_blended_trial.py',
        'backend/app/commissioning/blended_trial_window.py','scripts/real_robot_blended_trial.py')
    return dict(schema=1,scope='two_segment_real_blend_candidate',execute=False,
        deployment_ready=False,native_entry_available=True,sdk_connections=0,physical_verified=False,
        direction='base_+Z',step_mm=4,max_target_distance_mm=8,max_pending_ids=2,
        max_movel=2,max_fault_stop=1,normal_release='no_new_targets_drain_owned_ids',
        input_and_feedback_freshness_ms=100,movel_reply_ms=200,
        joint_speed_cap_radps=.05,joint_acceleration_cap_radps2=.2,
        stop_reply_ms=200,stop_verification_ms=500,native_fk_per_segment=5,
        source_hashes={s:hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources},
        disabled=['web','VR','gripper','Home','prepare_move','resume','reset','automatic_retry','reverse'],
        pending=['current_readonly_site_survey',
                 'fresh_onsite_authorization','physical_empty_load_acceptance'])


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.parse_args()
    print(json.dumps(preview(),indent=2))
