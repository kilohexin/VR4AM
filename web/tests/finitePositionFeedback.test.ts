import {expect,it} from 'vitest';
import {finiteHoldTitle} from '../src/ui/finitePositionFeedback';
it('explains recoverable input interruption without claiming a backend fault',()=>{
 expect(finiteHoldTitle({phase:'hold',reason:'position_input_expired'} as any)).toContain('松开后重新按住');
 expect(finiteHoldTitle({phase:'fault',reason:'position_input_expired'} as any)).toBeNull();
});
