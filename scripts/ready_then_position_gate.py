"""Pure terminal-report gate. This module has no SDK or execution entry point.

A caller still needs explicit authorization covering both actions, bound
previews and their deadlines. This check never grants robot permissions.
"""


def require_accepted_ready(report,*,exit_code,local_rpc_connections):
    rejected='ready_then_position_stage_one_not_accepted'
    if (type(exit_code) is not int or exit_code!=0
            or type(local_rpc_connections) is not int or local_rpc_connections!=0
            or type(report) is not dict or report.get('runtime')!='LIVE_SINGLE_READY_RETURN'):
        raise ValueError(rejected)
    if any(report.get(key) is not True for key in ('complete','post_preflight_ready','tail_complete',
            'recording_closed_verified','local_resources_released')):
        raise ValueError(rejected)
    if any(type(report.get(key)) is not int or report[key]!=value for key,value in
            dict(connect_entries=1,movej_entries=1,stop_entries=0,pending_tasks_at_process_exit=0).items()):
        raise ValueError(rejected)
    if (any(report.get(key) is not False for key in ('read_remote_result_unknown','movej_remote_result_unknown'))
            or 'latched_fault' not in report or report['latched_fault'] is not None
            or report.get('cleanup_errors')!=[]
            or not isinstance(report.get('speed_audit'),dict)
            or report['speed_audit'].get('status')!='within_observed_limit'):
        raise ValueError(rejected)
    connector=report.get('connection_diagnostics')
    if (type(connector) is not dict or type(connector.get('factory_entries')) is not int
            or connector['factory_entries']!=1 or connector.get('outcome')!='returned'
            or connector.get('remote_result_unknown') is not False
            or connector.get('native_sdk_init_entered') is not True
            or connector.get('native_sdk_connect_entered') is not True):
        raise ValueError(rejected)
