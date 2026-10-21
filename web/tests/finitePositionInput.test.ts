import {it,expect,vi} from 'vitest';
import {FinitePositionInput} from '../src/ui/finitePositionInput';

it('separates normal release from abort and never replays after reset',()=>{
 const send=vi.fn(),stop=vi.fn(),input=new FinitePositionInput(send,stop);
 input.reset('first',3);input.offer([1,0,0]);input.release();
 expect(send.mock.calls.map(c=>c[0].kind)).toEqual(['held','release']);expect(stop).not.toHaveBeenCalled();
 input.offer([0,0,1]);input.abort();expect(stop).toHaveBeenCalledTimes(1);
 input.reset('second',4);input.release();expect(send).toHaveBeenCalledTimes(3);
 input.offer([0,1,0]);expect(send.mock.calls[3][0]).toMatchObject({generation:4,seq:1,session_id:'second'});
});

it('rejects nonfinite input and caps diagonal vector magnitude',()=>{
 const send=vi.fn(),input=new FinitePositionInput(send,vi.fn());input.reset('web',1);
 input.offer([NaN,0,0]);expect(send).not.toHaveBeenCalled();
 input.offer([1,1,1]);expect(Math.hypot(...send.mock.calls[0][0].vector)).toBeCloseTo(1);
});

it('can abort a draining obligation after normal release, once only',()=>{
 const send=vi.fn(),stop=vi.fn(),input=new FinitePositionInput(send,stop);
 input.reset('web',1);input.offer([1,0,0]);input.release();input.abort();input.abort();
 expect(stop).toHaveBeenCalledTimes(1);
});
