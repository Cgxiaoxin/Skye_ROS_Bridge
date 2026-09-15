<script>
  import { onMount } from 'svelte';
  import TopBar from './components/TopBar.svelte';
  import { connectStateStream } from './lib/ws.js';
  import { retryStep } from './lib/api.js';

  let snapshot = null;
  let wsStatus = 'disconnected';
  let toastError = '';

  $: wsConnected = wsStatus === 'connected';
  $: displaySnapshot = wsConnected ? snapshot : null;

  $: session = displaySnapshot?.session ?? {};
  $: sessionState = session.state ?? 'IDLE';
  $: isIdle = sessionState === 'IDLE';
  $: mode = session.mode;
  $: banner = displaySnapshot?.banner;
  $: nextHint = displaySnapshot?.next_hint ?? (wsConnected ? '等待连接…' : '连接断开，正在重连…');
  $: hitlMode = displaySnapshot?.hitl?.mode;
  $: teleopState = displaySnapshot?.teleop?.state;
  $: alignStatus = displaySnapshot?.align?.status;
  $: recording = displaySnapshot?.recording?.active;

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
      <p class="panel-placeholder">Task 8：步骤列表与健康指示</p>
      {#if displaySnapshot?.health?.length}
        <ul class="health-list">
          {#each displaySnapshot.health as item}
            <li class:ok={item.ok} class:bad={!item.ok}>
              <span class="health-dot"></span>
              <span class="mono">{item.key}</span>
            </li>
          {/each}
        </ul>
      {/if}
    </aside>

    <section class="panel panel-main">
      <p class="next-hint">{nextHint}</p>

      {#if mode === 'teleop_record'}
        <div class="mode-panel">
          <h2 class="panel-title">遥操数采</h2>
          <dl class="status-grid">
            <div><dt>遥操状态</dt><dd class="mono">{teleopState ?? '—'}</dd></div>
            <div><dt>对齐状态</dt><dd class="mono">{alignStatus ?? '—'}</dd></div>
            <div><dt>录制</dt><dd class="mono">{recording ? '进行中' : '未录制'}</dd></div>
          </dl>
          <p class="panel-placeholder">Task 8：同步 / 对齐 / 遥操 / 录制按钮</p>
        </div>
      {:else if mode === 'dagger'}
        <div class="mode-panel">
          <h2 class="panel-title">DAgger 人机协同</h2>
          <p class="hitl-display mono">{hitlMode ?? '—'}</p>
          <p class="panel-placeholder">Task 8：接管 / 交还 / 录制控制</p>
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
      <p class="panel-placeholder">Task 8：左右臂与夹爪反馈</p>
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
