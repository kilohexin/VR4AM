import {it,expect} from 'vitest';
import {isFinitePositionInputMessage,isFinitePositionStatus} from '../src/protocol/messages';

it('validates optional bounded pitch without accepting a pitched release or extra fields',()=>{
 const row={v:1,type:'finite_position_input',session_id:'web',generation:1,seq:1,kind:'held',vector:[0,0,0]};
 expect(isFinitePositionInputMessage(row)).toBe(true);
 expect(isFinitePositionInputMessage({...row,pitch:1})).toBe(true);
 for(const pitch of [true,NaN,Infinity,2,'1'])expect(isFinitePositionInputMessage({...row,pitch})).toBe(false);
 expect(isFinitePositionInputMessage({...row,pitch:1,kind:'release'})).toBe(false);
 expect(isFinitePositionInputMessage({...row,pitch:0,kind:'release'})).toBe(true);
 expect(isFinitePositionInputMessage({...row,spin:1})).toBe(false);
 const status={policy:'finite_position_offline',phase:'ready',generation:1,entered_steps:0,motion_id:null,reason:null,hardware_verified:false};
 expect(isFinitePositionStatus(status)).toBe(true);
 expect(isFinitePositionStatus({...status,pitch_enabled:true})).toBe(true);
 expect(isFinitePositionStatus({...status,pitch_enabled:1})).toBe(false);
});
