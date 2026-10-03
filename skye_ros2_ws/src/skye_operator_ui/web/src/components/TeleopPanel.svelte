<script>
  import { createEventDispatcher } from 'svelte';
  import { postCommand } from '../lib/api.js';
  import { commandAllowed } from '../lib/commands.js';

  export let snapshot = null;

  const dispatch = createEventDispatcher();

  let busyOp = '';

  $: teleopState = snapshot?.teleop?.state;
  $: alignStatus = snapshot?.align?.status;
  $: recording = snapshot?.recording?.active;
  $: leftEnabled = snapshot?.leader_arms?.left_enabled !== false;
  $: rightEnabled = snapshot?.leader_arms?.right_enabled !== false;
  // Pass snapshot into the expression so Svelte re-runs when WS snapshot updates
  // (leftEnabled alone stays true null→READY and would leave buttons stuck disabled).
  $: gateLeft = commandAllowed(
    `leader_left_${leftEnabled ? 'off' : 'on'}`,
    snapshot,
  );
  $: gateRight = commandAllowed(
    `leader_right_${rightEnabled ? 'off' : 'on'}`,
    snapshot,
  );
  $: leaderGateUi = snapshot?.features?.leader_arm_gate !== false;
  $: pendingOp = snapshot?.pending_op;
  $: commandBusy = !!busyOp || !!pendingOp;

  const BUTTONS = [
    { op: 'switch_sync', label: '同步', class: 'btn-primary' },
    { op: 'align_start', label: '开始对齐', class: 'btn-neutral' },
    { op: 'align_cancel', label: '取消对齐', class: 'btn-neutral' },
    { op: 'switch_teleop', label: '开启遥操', class: 'btn-primary' },
    { op: 'switch_stop', label: '停止遥操', class: 'btn-neutral' },
    { op: 'recorder_start', label: '开始录制', class: 'btn-primary' },
    { op: 'recorder_stop', label: '停止录制', class: 'btn-amber' },
  ];

  function check(op) {
    return commandAllowed(op, snapshot);
  }

  async function run(op) {
    if (commandBusy) return;
    const { allowed } = check(op);
    if (!allowed) return;

    busyOp = op;
    try {
      await postCommand(op);
    } catch (err) {
      dispatch('error', err.message);
    } finally {
      busyOp = '';
    }
  }

  async function toggleSide(side) {
    if (commandBusy) return;
    const enabled = snapshot?.leader_arms?.[`${side}_enabled`] !== false;
    const op = enabled ? `leader_${side}_off` : `leader_${side}_on`;
    const { allowed } = check(op);
    if (!allowed) return;
    if (enabled && !confirm('请托住该侧小臂，去使能后会下落。确认关闭？')) return;

    busyOp = op;
    try {
      await postCommand(op);
    } catch (err) {
      dispatch('error', err.message);
    } finally {
      busyOp = '';
    }
  }
</script>

<h2 class="panel-title">遥操数采</h2>

<dl class="status-grid">
  <div><dt>遥操状态</dt><dd class="mono">{teleopState ?? '—'}</dd></div>
  <div><dt>对齐状态</dt><dd class="mono">{alignStatus ?? '—'}</dd></div>
  <div><dt>录制</dt><dd class="mono">{recording ? '进行中' : '未录制'}</dd></div>
</dl>

<div class="action-grid">
  {#each BUTTONS as btn}
    {@const gate = check(btn.op)}
    <button
      type="button"
      class="btn {btn.class} action-btn"
      disabled={!gate.allowed || commandBusy}
      title={gate.reason || btn.label}
      on:click={() => run(btn.op)}
    >
      {btn.label}
    </button>
    {#if btn.op === 'switch_sync' && leaderGateUi}
      <div class="leader-gate-block">
        <div class="leader-gate-row">
          <button
            type="button"
            class="btn btn-neutral action-btn"
            class:btn-amber={!leftEnabled}
            disabled={!gateLeft.allowed || !!snapshot?.leader_arms?.locked || commandBusy}
            title={gateLeft.reason || '关左臂'}
            on:click={() => toggleSide('left')}
          >
            关左臂
          </button>
          <button
            type="button"
            class="btn btn-neutral action-btn"
            class:btn-amber={!rightEnabled}
            disabled={!gateRight.allowed || !!snapshot?.leader_arms?.locked || commandBusy}
            title={gateRight.reason || '关右臂'}
            on:click={() => toggleSide('right')}
          >
            关右臂
          </button>
        </div>
        <p class="mono leader-gate-status">
          小臂：左{leftEnabled ? '开' : '关'} · 右{rightEnabled ? '开' : '关'}
        </p>
      </div>
    {/if}
  {/each}
</div>

<style>
  .leader-gate-block {
    grid-column: 1 / -1;
  }

  .leader-gate-row {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 0.45rem;
  }

  .leader-gate-status {
    margin: 0.35rem 0 0;
    font-size: 0.8rem;
    color: var(--ink-muted);
  }
</style>
