"""Fixed no-actuator Lua probe source and contract, no transport or SDK import.

Preview only. Printing this program does NOT authorize or send it to a
controller. A separate scoped launcher must bind its bytes and target.
"""
import argparse
import hashlib
import json

PROGRAM="""program_begin(0)
disable_auto_sync()
local ok,err=pcall(function()
 local before=lebai:get_robot_state()
 assert(before=='IDLE','capability_requires_idle')
 local tool=lebai:get_tcp()
 for _,key in ipairs({'x','y','z','rx','ry','rz'}) do
  assert(type(tool[key])=='number' and tool[key]==tool[key],'capability_tool_shape')
 end
 local first=lebai:get_kin_data()
 local q=first.actual_joint_pose
 assert(type(q)=='table' and #q==6,'capability_joint_shape')
 for i=1,6 do assert(type(q[i])=='number' and q[i]==q[i],'capability_joint_shape') end
 local fk=lebai:kinematics_forward(q)
 print('PVAT_CAP_TOOL',tool.x,tool.y,tool.z,tool.rx,tool.ry,tool.rz)
 print('PVAT_CAP_FK',fk.x,fk.y,fk.z,fk.rx,fk.ry,fk.rz)
 local previous=nil
 for i=1,5 do
  local now=timestamp()
  assert(type(now)=='number' and now==now and (not previous or now>=previous),'capability_clock_rollback')
  print('PVAT_CAP_CLOCK',i,now);previous=now
  rt_sleep(20)
 end
 local after=lebai:get_kin_data()
 for i=1,6 do assert(after.actual_joint_pose[i]==q[i],'capability_pose_changed') end
 assert(lebai:get_robot_state()=='IDLE','capability_state_changed')
 print('PVAT_CAP_APIS',type(async_task),type(wait_all),type(get_signals),type(set_signals))
end)
-- No exception stop: this program has no actuator or signal writes.
print('PVAT_CAP_DONE',ok,tostring(err))
program_end(0)
"""


def preview():
    return dict(schema=1,execute=False,source_sha256=hashlib.sha256(PROGRAM.encode()).hexdigest(),
        source_bytes=len(PROGRAM.encode()),native_program_submissions=1,
        program_sdk_queries_max=7,program_fk_calls=1,program_ik_calls=0,
        robot_writes=0,signal_writes=0,clock_samples=5,clock_sample_spacing_ms=20,
        native_port_contract=3011,automatic_retry=False,physical_verified=False,
        motion_admitted=False,scope_approved=False,
        limitations=['five_clock_samples_do_not_prove_monotonic_clock',
            'api_function_presence_does_not_prove_motion_or_signal_write_semantics',
            'native_lua_connection_requires_separate_onsite_authorization'])


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--print-program',action='store_true')
    args=p.parse_args(argv);print(PROGRAM if args.print_program else json.dumps(preview(),indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
