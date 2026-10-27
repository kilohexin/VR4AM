import {afterEach,expect,it,vi} from 'vitest';
import {DesktopInputScheduler} from '../src/scenes/desktopInputScheduler';
afterEach(()=>vi.useRealTimers());

it('samples the current desktop input independently of rendering, without catch-up replay',()=>{
 vi.useFakeTimers();let now=0;const samples:number[]=[];
 const scheduler=new DesktopInputScheduler(t=>samples.push(t),()=>now);
 scheduler.start();now=20;vi.advanceTimersByTime(20);
 now=200;vi.advanceTimersByTime(20);
 expect(samples).toEqual([20,200]);
 scheduler.stop();vi.advanceTimersByTime(200);expect(samples).toHaveLength(2);
});

it('never samples desktop input while XR owns it and supports clean restart',()=>{
 vi.useFakeTimers();const sample=vi.fn();const scheduler=new DesktopInputScheduler(sample,()=>20);
 scheduler.start();scheduler.start();scheduler.setXROwned(true);vi.advanceTimersByTime(60);
 expect(sample).not.toHaveBeenCalled();scheduler.setXROwned(false);vi.advanceTimersByTime(20);
 expect(sample).toHaveBeenCalledTimes(1);scheduler.stop();scheduler.start();vi.advanceTimersByTime(20);
 expect(sample).toHaveBeenCalledTimes(2);scheduler.stop();
});
