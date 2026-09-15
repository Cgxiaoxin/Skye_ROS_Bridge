<script>
  import { createEventDispatcher } from 'svelte';
  import { fetchStepLogs } from '../lib/api.js';

  export let stepId = '';
  export let stepLabel = '';

  const dispatch = createEventDispatcher();

  let lines = [];
  let loading = false;
  let error = '';

  $: if (stepId) {
    loadLogs(stepId);
  }

  async function loadLogs(id) {
    loading = true;
    error = '';
    try {
      lines = await fetchStepLogs(id);
    } catch (err) {
      error = err.message;
      lines = [];
    } finally {
      loading = false;
    }
  }

  function handleClose() {
    dispatch('close');
  }

  function handleBackdropClick(event) {
    if (event.target === event.currentTarget) {
      handleClose();
    }
  }
</script>

<div class="drawer-backdrop" on:click={handleBackdropClick} role="presentation">
  <aside class="log-drawer" aria-label="步骤日志">
    <header class="drawer-header">
      <h3 class="drawer-title">{stepLabel || stepId}</h3>
      <button type="button" class="btn btn-neutral drawer-close" on:click={handleClose}>
        关闭
      </button>
    </header>

    <div class="drawer-body">
      {#if loading}
        <p class="drawer-status">加载中…</p>
      {:else if error}
        <p class="drawer-error">{error}</p>
      {:else if lines.length === 0}
        <p class="drawer-status">暂无日志</p>
      {:else}
        <pre class="log-content mono">{lines.join('\n')}</pre>
      {/if}
    </div>
  </aside>
</div>
