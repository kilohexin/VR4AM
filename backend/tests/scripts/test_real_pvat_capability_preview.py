import re
import sys

from scripts import preview_real_pvat_capability as probe


def test_fixed_capability_source_has_only_read_methods_no_signal_or_actuator_calls():
    assert set(re.findall(r'lebai:(\w+)\(',probe.PROGRAM))=={
        'get_robot_state','get_tcp','get_kin_data','kinematics_forward'}
    calls=set(re.findall(r'(?<![:.\w])(\w+)\s*\(',probe.PROGRAM))
    assert not calls.intersection({'set_signal','set_signals','movej','movel','move_pvat','stop_move','stop_sys','set_claw','start_sys'})
    assert 'io.open' not in probe.PROGRAM and '/proc/' not in probe.PROGRAM
    assert 'capability_clock_rollback' in probe.PROGRAM
    assert probe.PROGRAM.count('kinematics_forward(')==1


def test_preview_and_print_are_both_zero_connection_operations(capsys):
    imported='lebai_sdk' in sys.modules
    assert probe.main([])==0
    assert '"execute": false' in capsys.readouterr().out
    assert probe.main(['--print-program'])==0
    assert 'PVAT_CAP_DONE' in capsys.readouterr().out
    assert ('lebai_sdk' in sys.modules)==imported
    contract=probe.preview()
    assert contract['robot_writes']==contract['signal_writes']==0 and not contract['motion_admitted']
