import type {FinitePositionStatus} from '../protocol/messages';

export function enteredFiniteHold(previous:FinitePositionStatus|null|undefined,next:FinitePositionStatus|null|undefined):boolean {
 return next?.phase==='hold'&&(previous?.phase!=='hold'||previous.generation!==next.generation
  ||(next.blocked_input_seq!=null&&next.blocked_input_seq!==previous.blocked_input_seq));
}

/** Display known local boundary reasons, never arbitrary native exception text. */
export function finiteHoldTitle(status:FinitePositionStatus|null|undefined):string|null {
 if(status?.phase!=='hold')return null;
 switch(status.reason){
  case 'pvat_gesture_complete':return '本次连续控制已结束，请松开 Grip 后重新握持';
  case 'pvat_reference_boundary':return '本段运动范围已到，请松开 Grip 后重新握持';
  case 'position_input_expired':return '手柄输入中断，请松开后重新按住';
  case 'position_workspace':return '工作区边界';
  case 'position_joint_limit':return '关节边界';
  case 'position_pitch_limit':return '俯仰角度边界';
  case 'position_ik_unreachable':return '此方向暂不可达';
  case 'position_joint_jump':return '此方向关节变化过大';
  case 'blend_joint_jump':return '此方向接近运动学边界，请松开后反向调整';
  case 'blend_joint_limit':return '关节边界';
  case 'blend_workspace':return '工作区边界';
  case 'blend_ik_unreachable':return '此方向暂不可达';
  case 'blend_singular_path':return '此方向接近奇异姿态，请换方向';
  case 'blend_rotation_range':return '转腕范围边界';
  case 'position_estimated_duration':return '此方向小步耗时过长';
  case 'position_path_rotation':return '此方向姿态路径受限';
  case 'position_unresolvable_step':return '目标小步无法分辨';
  default:return '本次长按已暂停续发';
 }
}
