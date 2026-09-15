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

  function check(op) {
    return commandAllowed(op, snapshot);
  }

  async function run(op) {
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
    disabled={!check('takeover').allowed || busyOp === 'takeover' || !!snapshot?.pending_op}
    title={check('takeover').reason || '接管控制权'}
    on:click={() => run('takeover')}
  >
    接管
  </button>
  <button
    type="button"
    class="btn btn-amber action-btn action-btn-large"
    disabled={!check('return').allowed || busyOp === 'return' || !!snapshot?.pending_op}
    title={check('return').reason || '交还控制权'}
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
    disabled={!check('recorder_start').allowed || busyOp === 'recorder_start' || !!snapshot?.pending_op}
    title={check('recorder_start').reason || '开始录制'}
    on:click={() => run('recorder_start')}
  >
    开始录制
  </button>
  <button
    type="button"
    class="btn btn-amber action-btn"
    disabled={!check('recorder_stop').allowed || busyOp === 'recorder_stop' || !!snapshot?.pending_op}
    title={check('recorder_stop').reason || '停止录制'}
    on:click={() => run('recorder_stop')}
  >
    停止录制
  </button>
</div>
