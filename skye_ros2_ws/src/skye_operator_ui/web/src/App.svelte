<script>
  import { onMount } from 'svelte';
  import TopBar from './components/TopBar.svelte';
  import StepRail from './components/StepRail.svelte';
  import TeleopPanel from './components/TeleopPanel.svelte';
  import DaggerPanel from './components/DaggerPanel.svelte';
  import ArmStrip from './components/ArmStrip.svelte';
  import LogDrawer from './components/LogDrawer.svelte';
  import { connectStateStream } from './lib/ws.js';
  import { retryStep } from './lib/api.js';

  let snapshot = null;
  let wsStatus = 'disconnected';
  let toastError = '';
  let logDrawer = null;

  $: wsConnected = wsStatus === 'connected';
  $: displaySnapshot = wsConnected ? snapshot : null;

  $: session = displaySnapshot?.session ?? {};
  $: sessionState = session.state ?? 'IDLE';
  $: isIdle = sessionState === 'IDLE';
  $: mode = session.mode;
  $: banner = displaySnapshot?.banner;
  $: nextHint = displaySnapshot?.next_hint ?? (wsConnected ? '等待连接…' : '连接断开，正在重连…');

  onMount(() => {
    const stream = connectStateStream(
      (data) => {
        snapshot = data;
      },
      (status) => {
        wsStatus = status;
        if (status !== 'connected') {
          snapshot = null;
        }
      },
    );
    return () => stream.close();
  });

  function handleTopBarError(event) {
    toastError = event.detail;
  }

  function handlePanelError(event) {
    toastError = event.detail;
  }

  function handleOpenLogs(event) {
    logDrawer = event.detail;
  }

  function handleCloseLogs() {
    logDrawer = null;
  }

  async function handleRetry() {
    toastError = '';
    try {
      await retryStep();
    } catch (err) {
      toastError = err.message;
    }
  }
</script>

<div class="app-shell">
  <TopBar snapshot={displaySnapshot} {wsStatus} on:error={handleTopBarError} />

  {#if !wsConnected}
    <div class="disconnect-banner" role="status">
      WebSocket 断开，正在重连 — 会话状态暂不可信
    </div>
  {/if}

  {#if banner}
    <div class="banner banner-{banner.level}">
      {banner.text}
    </div>
  {/if}

  <main class="main-grid">
    <aside class="panel panel-side">
      <h2 class="panel-title">步骤 / 健康</h2>
      <StepRail snapshot={displaySnapshot} on:openLogs={handleOpenLogs} />
    </aside>

    <section class="panel panel-main">
      <p class="next-hint">{nextHint}</p>

      {#if mode === 'teleop_record'}
        <div class="mode-panel">
          <TeleopPanel snapshot={displaySnapshot} on:error={handlePanelError} />
        </div>
      {:else if mode === 'dagger'}
        <div class="mode-panel">
          <DaggerPanel snapshot={displaySnapshot} on:error={handlePanelError} />
        </div>
      {:else if isIdle}
        <div class="mode-panel idle-panel">
          <p>在顶栏选择机台与模式，点击「启动会话」。</p>
        </div>
      {:else}
        <div class="mode-panel">
          <p class="panel-placeholder">等待模式就绪…</p>
        </div>
      {/if}

      {#if sessionState === 'FAILED'}
        <button type="button" class="btn btn-amber retry-btn" on:click={handleRetry}>
          重试当前步骤
        </button>
      {/if}
    </section>

    <aside class="panel panel-side">
      <h2 class="panel-title">关节 / 夹爪</h2>
      <ArmStrip snapshot={displaySnapshot} />
    </aside>
  </main>

  <footer class="status-bar">
    <span class="mono">
      WS: {wsStatus === 'connected' ? '已连接' : wsStatus === 'reconnecting' ? '重连中' : '已断开'}
    </span>
    {#if displaySnapshot?.pending_op}
      <span class="mono pending">pending: {displaySnapshot.pending_op}</span>
    {/if}
    {#if toastError}
      <span class="status-error">{toastError}</span>
    {/if}
  </footer>
</div>

{#if logDrawer}
  <LogDrawer
    stepId={logDrawer.stepId}
    stepLabel={logDrawer.stepLabel}
    on:close={handleCloseLogs}
  />
{/if}
