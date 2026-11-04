import {expect,it} from 'vitest';
import {finiteHoldTitle} from '../src/ui/finitePositionFeedback';
it('explains recoverable input interruption without claiming a backend fault',()=>{
 expect(finiteHoldTitle({phase:'hold',reason:'position_input_expired'} as any)).toContain('松开后重新按住');
 expect(finiteHoldTitle({phase:'fault',reason:'position_input_expired'} as any)).toBeNull();
});
it('explains recovery with a fresh press in Docker pose toggle mode',()=>{
 expect(finiteHoldTitle({phase:'hold',policy:'finite_position_docker',pose_follow_enabled:true,
  reason:'position_input_expired'} as any)).toBe('手柄输入中断，请重新按 Grip 对齐');
});
