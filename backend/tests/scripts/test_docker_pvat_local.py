import importlib.util
import pytest
from pathlib import Path

spec=importlib.util.spec_from_file_location('docker_pvat_local',Path(__file__).resolve().parents[3]/'scripts/docker_pvat_local.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def test_missing_completion_and_zero_progress_do_not_pass():
    assert not m.analyze('PVAT_POINT\t0\t.01\t0\t0\t1\n')['passed']
    assert not m.analyze('PVAT_DONE\ttrue\tnil\n')['passed']


def test_trace_reports_cadence_progress_and_drain():
    result=m.analyze('PVAT_BEGIN\t0\t.08\nPVAT_POINT\t0\t.001\t0\t.01\t2\n'
        'PVAT_POINT\t21\t.002\t.001\t.01\t3\nPVAT_RELEASE\t40\t2\n'
        'PVAT_TAIL\t300\tIDLE\t.002\t0\nPVAT_DONE\ttrue\tnil\n')
    assert result['passed'] and result['max_send_gap_ms']==21
    assert result['release_to_quiet_ms']==260 and result['max_rpc_ms']==3


def test_failed_controller_run_preserves_raw_trace():
    trace='PVAT_ABORT\ttrue\ttrue\nPVAT_DONE\tfalse\tsend_gap\n'
    result=m.analyze(trace)
    assert result['completed'] and not result['passed'] and result['raw_output']==trace


def test_generated_program_has_only_explicit_bounded_profiles():
    text=m.code(True,True,False)
    assert '__INTERRUPT__' not in text and '__PHASE_CLOCK__' not in text and '__DIRECT_API__' not in text
    assert 'local interrupt = true' in text and 'local direct_api = false' in text
    assert 'disable_auto_sync()' in text and 'program_begin(0)' in text and 'program_end(0)' in text


def test_pvt_profile_requires_same_phase_clock_and_sdk():
    with pytest.raises(ValueError):m.code(pvt=True)
    with pytest.raises(ValueError):m.code(phase_clock=True,direct_api=True,pvt=True)
    text=m.code(phase_clock=True,pvt=True)
    assert 'local pvt = true' in text and '__PVT__' not in text


def test_metrics_count_near_zero_samples_and_observed_acceleration():
    result=m.analyze('PVAT_POINT\t500\t.002\t.001\t.003\t10\t530\t.02\n'
        'PVAT_POINT\t540\t.003\t.002\t0\t11\t571\t-.03\n')
    assert result['middle_samples']==2 and result['middle_near_zero_velocity_samples']==1
    assert result['max_sample_delay_ms']==31
    assert result['max_sampled_joint_acceleration_radps2']==.03


def test_complete_reference_must_reach_endpoint_not_merely_show_progress():
    trace=('PVAT_BEGIN\t0\t.04\nPVAT_POINT\t1000\t.01\t.003\t.01\t10\n'
        'PVAT_TAIL\t2000\tIDLE\t.005\t0\nPVAT_DONE\ttrue\tnil\n')
    assert not m.analyze(trace,interrupt=False)['passed']
    assert m.analyze(trace,interrupt=True)['passed']
