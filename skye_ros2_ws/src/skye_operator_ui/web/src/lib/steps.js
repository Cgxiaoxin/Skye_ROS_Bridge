/** Startup + runtime step definitions per session mode. */

export const TELEOP_STEPS = [
  { id: 'precheck', label: '预检' },
  { id: 'driver', label: '大臂 Driver', logId: 'driver' },
  { id: 'marvin', label: '小臂 Marvin', logId: 'marvin' },
  { id: 'sync', label: '小臂同步' },
  { id: 'align', label: 'Follower 对齐', logId: 'align', optional: true },
  { id: 'teleop', label: '遥操' },
  { id: 'record', label: '数采', logId: 'recorder' },
];

export const DAGGER_STEPS = [
  { id: 'precheck', label: '预检' },
  { id: 'driver', label: '大臂 Driver', logId: 'driver' },
  { id: 'marvin', label: 'HITL 小臂', logId: 'marvin' },
  { id: 'arbiter', label: 'Arbiter', logId: 'arbiter' },
  { id: 'policy', label: '策略源', logId: 'dummy_policy', optional: true },
  { id: 'control', label: '控制权' },
  { id: 'record', label: '录制', logId: 'recorder' },
];

const PLAYBOOK_IDS = {
  teleop_record: ['driver', 'marvin', 'align', 'recorder'],
  dagger: ['driver', 'marvin', 'arbiter', 'dummy_policy'],
};

export function stepsForMode(mode) {
  if (mode === 'dagger') return DAGGER_STEPS;
  return TELEOP_STEPS;
}

/**
 * @returns {'pending' | 'active' | 'done' | 'skipped'}
 */
export function stepStatus(step, ctx) {
  const {
    sessionState,
    currentStepId,
    teleopState,
    alignStatus,
    hitlMode,
    recording,
    health,
  } = ctx;

  if (sessionState === 'IDLE') return 'pending';

  if (step.id === 'precheck') {
    if (sessionState === 'PRECHECK') return 'active';
    if (['STARTING', 'READY', 'RUNNING', 'DEGRADED'].includes(sessionState)) {
      return 'done';
    }
    return 'pending';
  }

  const playbookIds = PLAYBOOK_IDS[ctx.mode] ?? [];
  const playbookIndex = playbookIds.indexOf(step.logId ?? step.id);
  const isPlaybookStep = playbookIndex >= 0;

  if (isPlaybookStep) {
    if (['FAILED', 'STOPPING'].includes(sessionState)) {
      if (sessionState === 'FAILED' && currentStepId === step.logId) return 'active';
      if (playbookIds.indexOf(currentStepId) > playbookIndex) return 'done';
      if (playbookIds.indexOf(currentStepId) === playbookIndex && sessionState === 'FAILED') {
        return 'active';
      }
      return sessionState === 'STOPPING' ? 'done' : 'pending';
    }

    if (sessionState === 'STARTING') {
      const curIdx = playbookIds.indexOf(currentStepId);
      if (curIdx < 0) return 'pending';
      if (playbookIndex < curIdx) return 'done';
      if (playbookIndex === curIdx) return 'active';
      return 'pending';
    }

    if (['READY', 'RUNNING', 'DEGRADED'].includes(sessionState)) {
      if (step.optional && step.logId && health && health[step.logId] === false) {
        return 'skipped';
      }
      return 'done';
    }
    return 'pending';
  }

  if (!['READY', 'RUNNING', 'DEGRADED'].includes(sessionState)) {
    return 'pending';
  }

  if (step.id === 'sync') {
    if (teleopState === 'TELEOP_SYNCING') return 'active';
    if (['SYNCED', 'TELEOP'].includes(teleopState)) return 'done';
    return 'pending';
  }

  if (step.id === 'align') {
    if (alignStatus === 'ALIGNING') return 'active';
    if (['ALIGNED', 'TIMEOUT_WARN'].includes(alignStatus)) return 'done';
    if (alignStatus === 'TIMEOUT_WARN') return 'done';
    return teleopState === 'SYNCED' ? 'pending' : 'pending';
  }

  if (step.id === 'teleop') {
    if (teleopState === 'TELEOP') return 'done';
    if (teleopState === 'SYNCED') return 'active';
    return 'pending';
  }

  if (step.id === 'control') {
    if (hitlMode === 'HANDOVER_SYNC') return 'active';
    if (['HUMAN', 'AUTONOMOUS'].includes(hitlMode)) return 'done';
    return 'pending';
  }

  if (step.id === 'record') {
    if (recording) return 'active';
    return 'pending';
  }

  return 'pending';
}
