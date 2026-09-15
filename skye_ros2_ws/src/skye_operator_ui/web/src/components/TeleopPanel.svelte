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
    const { allowed, reason } = check(op);
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
      disabled={!gate.allowed || busyOp === btn.op || !!snapshot?.pending_op}
      title={gate.reason || btn.label}
      on:click={() => run(btn.op)}
    >
      {btn.label}
    </button>
  {/each}
</div>
