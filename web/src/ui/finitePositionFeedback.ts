import type {FinitePositionStatus} from '../protocol/messages';

export function enteredFiniteHold(previous:FinitePositionStatus|null|undefined,next:FinitePositionStatus|null|undefined):boolean {
 return next?.phase==='hold'&&(previous?.phase!=='hold'||previous.generation!==next.generation
  ||(next.blocked_input_seq!=null&&next.blocked_input_seq!==previous.blocked_input_seq));
}

/** Display known local boundary reasons, never arbitrary native exception text. */
export function finiteHoldTitle(status:FinitePositionStatus|null|undefined):string|null {
 if(status?.phase!=='hold')return null;
 switch(status.reason){
  case 'position_workspace':return '工作区边界';
  case 'position_joint_limit':return '关节边界';
  case 'position_pitch_limit':return '俯仰角度边界';
  case 'position_ik_unreachable':return '此方向暂不可达';
  case 'position_joint_jump':return '此方向关节变化过大';
  case 'position_estimated_duration':return '此方向小步耗时过长';
  case 'position_path_rotation':return '此方向姿态路径受限';
  case 'position_unresolvable_step':return '目标小步无法分辨';
  default:return '本次长按已暂停续发';
 }
}
