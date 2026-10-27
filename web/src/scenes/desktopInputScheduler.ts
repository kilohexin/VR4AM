/** Sample live desktop controls independently of WebGL rendering. No cached XR frames. */
export class DesktopInputScheduler {
 private timer:ReturnType<typeof setInterval>|null=null;
 private xrOwned=false;
 constructor(private sample:(nowMs:number)=>void,private now=()=>performance.now()){}
 start():void {
  if(this.timer!==null)return;
  this.timer=setInterval(()=>{if(!this.xrOwned)this.sample(this.now());},20);
 }
 setXROwned(owned:boolean):void {this.xrOwned=owned;}
 stop():void {if(this.timer!==null)clearInterval(this.timer);this.timer=null;}
}
