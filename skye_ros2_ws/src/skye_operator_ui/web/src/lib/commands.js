/** Mirror backend command_allowed for UI button disable states. */

const OPERATIONAL_STATES = ['READY', 'RUNNING', 'DEGRADED'];
const ALIGN_DONE = ['ALIGNED', 'TIMEOUT_WARN'];
const TELEOP_OPS = [
  'switch_sync',
  'switch_teleop',
  'switch_stop',
  'align_start',
  'align_cancel',
];
const DAGGER_OPS = ['takeover', 'return'];

export function commandAllowed(op, snapshot) {
  if (!snapshot) {
    return { allowed: false, reason: '未连接' };
  }

  const session = snapshot.session ?? {};
  const state = session.state ?? 'IDLE';
  const mode = session.mode;
  const teleopState = snapshot.teleop?.state;
  const hitlMode = snapshot.hitl?.mode;
  const alignStatus = snapshot.align?.status;

  if (snapshot.pending_op) {
    return { allowed: false, reason: `等待命令确认：${snapshot.pending_op}` };
  }

  if (op === 'emergency_stop') {
    return { allowed: true, reason: '' };
  }

  if (!OPERATIONAL_STATES.includes(state)) {
    if (state === 'STOPPING') {
      return { allowed: false, reason: '会话正在结束，请稍候' };
    }
    if (state === 'FAILED') {
      return { allowed: false, reason: '会话启动失败，请重试或结束会话' };
    }
    return { allowed: false, reason: '会话尚未就绪，请等待启动完成' };
  }

  if (!mode) {
    return { allowed: false, reason: '会话模式未知' };
  }

  if (TELEOP_OPS.includes(op) && mode !== 'teleop_record') {
    return { allowed: false, reason: '当前为 DAgger 模式，无法执行遥操命令' };
  }
  if (DAGGER_OPS.includes(op) && mode !== 'dagger') {
    return { allowed: false, reason: '当前为遥操数采模式，无法执行接管/交还' };
  }

  if (DAGGER_OPS.includes(op) && hitlMode === 'HANDOVER_SYNC') {
    return { allowed: false, reason: '接管对齐中，请等待同步完成' };
  }

  if (op === 'return') {
    if (hitlMode !== 'HUMAN') {
      return { allowed: false, reason: '仅在人工控制（HUMAN）时可交还' };
    }
    return { allowed: true, reason: '' };
  }

  if (op === 'takeover') {
    if (hitlMode !== 'AUTONOMOUS') {
      return { allowed: false, reason: '仅在自主模式（AUTONOMOUS）时可接管' };
    }
    return { allowed: true, reason: '' };
  }

  if (op === 'switch_teleop') {
    const synced = teleopState === 'SYNCED';
    const alignDone = ALIGN_DONE.includes(alignStatus);
    if (!synced && !alignDone) {
      return { allowed: false, reason: '请先完成小臂同步或 follower 对齐' };
    }
    return { allowed: true, reason: '' };
  }

  if (op === 'align_start') {
    if (alignStatus === 'ALIGNING') {
      return { allowed: false, reason: '对齐正在进行中' };
    }
    if (teleopState !== 'SYNCED') {
      return { allowed: false, reason: '请先完成小臂同步（SYNCED）' };
    }
    return { allowed: true, reason: '' };
  }

  if (op === 'align_cancel') {
    if (alignStatus !== 'ALIGNING') {
      return { allowed: false, reason: '当前未在对齐中' };
    }
    return { allowed: true, reason: '' };
  }

  if (op === 'switch_sync') {
    if (['TELEOP_SYNCING', 'SYNCED', 'TELEOP'].includes(teleopState)) {
      return { allowed: false, reason: '遥操状态不允许再次同步' };
    }
    return { allowed: true, reason: '' };
  }

  if (op === 'recorder_start' && state === 'DEGRADED') {
    return { allowed: false, reason: '系统降级，禁止新开录制' };
  }

  if (op === 'recorder_start' && snapshot.recording?.active) {
    return { allowed: false, reason: '已在录制中' };
  }

  if (op === 'recorder_stop' && !snapshot.recording?.active) {
    return { allowed: false, reason: '当前未在录制' };
  }

  return { allowed: true, reason: '' };
}
