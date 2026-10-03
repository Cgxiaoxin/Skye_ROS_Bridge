<script>
  import { createEventDispatcher } from 'svelte';
  import { postCommand } from '../lib/api.js';
  import { commandAllowed } from '../lib/commands.js';

  export let snapshot = null;

  const dispatch = createEventDispatcher();

  let busyOp = '';

  $: hitlMode = snapshot?.hitl?.mode;
  $: hitlSource = snapshot?.hitl?.source;
  $: recording = snapshot?.recording?.active;
  $: hitlClass = hitlModeClass(hitlMode);

  function hitlModeClass(mode) {
    if (mode === 'HUMAN') return 'hitl-human';
    if (mode === 'HANDOVER_SYNC') return 'hitl-sync';
    if (mode === 'AUTONOMOUS') return 'hitl-auto';
    return 'hitl-unknown';
  }

  // Bind these in markup (not via check()) so Svelte tracks snapshot updates.
  $: gateTakeover = commandAllowed('takeover', snapshot);
  $: gateEnter = commandAllowed('enter_teleop', snapshot);
  $: gateReturn = commandAllowed('return', snapshot);
  $: gateRecStart = commandAllowed('recorder_start', snapshot);
  $: gateRecStop = commandAllowed('recorder_stop', snapshot);
  $: pendingOp = snapshot?.pending_op;

  async function run(op) {
    const gate = commandAllowed(op, snapshot);
    if (!gate.allowed) return;

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

<h2 class="panel-title">DAgger 人机协同</h2>

<p class="hitl-display mono {hitlClass}">{hitlMode ?? '—'}</p>
{#if hitlSource}
  <p class="hitl-source mono">source: {hitlSource}</p>
{/if}

<div class="action-grid action-grid-dagger">
  <button
    type="button"
    class="btn btn-primary action-btn action-btn-large"
    disabled={!gateTakeover.allowed || busyOp === 'takeover' || !!pendingOp}
    title={gateTakeover.reason || '同步：策略 hold + 小臂跟大臂'}
    on:click={() => run('takeover')}
  >
    同步
  </button>
  <button
    type="button"
    class="btn btn-primary action-btn action-btn-large"
    disabled={!gateEnter.allowed || busyOp === 'enter_teleop' || !!pendingOp}
    title={gateEnter.reason || '确认对齐后再进入遥操（HUMAN）'}
    on:click={() => run('enter_teleop')}
  >
    进入遥操
  </button>
  <button
    type="button"
    class="btn btn-amber action-btn action-btn-large"
    disabled={!gateReturn.allowed || busyOp === 'return' || !!pendingOp}
    title={gateReturn.reason || '交还控制权'}
    on:click={() => run('return')}
  >
    交还
  </button>
</div>

<h3 class="panel-subtitle">HITL 录制</h3>
<p class="rec-status mono">{recording ? '录制进行中' : '未录制'}</p>

<div class="action-grid action-grid-rec">
  <button
    type="button"
    class="btn btn-primary action-btn"
    disabled={!gateRecStart.allowed || busyOp === 'recorder_start' || !!pendingOp}
    title={gateRecStart.reason || '开始录制'}
    on:click={() => run('recorder_start')}
  >
    开始录制
  </button>
  <button
    type="button"
    class="btn btn-amber action-btn"
    disabled={!gateRecStop.allowed || busyOp === 'recorder_stop' || !!pendingOp}
    title={gateRecStop.reason || '停止录制'}
    on:click={() => run('recorder_stop')}
  >
    停止录制
  </button>
</div>
