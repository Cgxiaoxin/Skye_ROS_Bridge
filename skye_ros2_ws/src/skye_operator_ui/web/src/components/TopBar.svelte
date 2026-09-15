<script>
  import { createEventDispatcher, onDestroy } from 'svelte';
  import { postCommand, startSession, stopSession } from '../lib/api.js';

  export let snapshot = null;
  export let wsStatus = 'disconnected';

  const dispatch = createEventDispatcher();

  let profile = 'thor';
  let mode = 'teleop_record';
  let clock = '';
  let apiError = '';
  let showStopConfirm = false;

  const PROFILE_LABELS = { thor: 'THOR', orin: 'ORIN' };
  const MODE_LABELS = {
    teleop_record: '遥操数采',
    dagger: 'DAgger',
  };
  const STATE_LABELS = {
    IDLE: '空闲',
    PRECHECK: '预检',
    STARTING: '启动中',
    READY: '就绪',
    RUNNING: '运行中',
    FAILED: '失败',
    STOPPING: '结束中',
    DEGRADED: '降级',
  };

  $: session = snapshot?.session ?? {};
  $: sessionState = session.state ?? 'IDLE';
  $: isIdle = sessionState === 'IDLE';
  $: badgeClass = stateBadgeClass(sessionState);

  $: if (isIdle && session.profile) {
    profile = session.profile;
  }
  $: if (isIdle && session.mode) {
    mode = session.mode;
  }

  function stateBadgeClass(state) {
    if (state === 'READY' || state === 'RUNNING') return 'badge-green';
    if (state === 'FAILED' || state === 'DEGRADED') return 'badge-red';
    if (state === 'IDLE') return 'badge-neutral';
    return 'badge-amber';
  }

  function tickClock() {
    const now = new Date();
    clock = now.toLocaleTimeString('zh-CN', { hour12: false });
  }

  tickClock();
  const clockTimer = setInterval(tickClock, 1000);

  async function handleStart() {
    apiError = '';
    try {
      await startSession(profile, mode);
    } catch (err) {
      apiError = err.message;
      dispatch('error', apiError);
    }
  }

  async function handleStop() {
    apiError = '';
    showStopConfirm = false;
    try {
      await stopSession();
    } catch (err) {
      apiError = err.message;
      dispatch('error', apiError);
    }
  }

  async function handleEmergencyStop() {
    apiError = '';
    try {
      await postCommand('emergency_stop');
    } catch (err) {
      apiError = err.message;
      dispatch('error', apiError);
    }
  }

  onDestroy(() => clearInterval(clockTimer));
</script>

<header class="topbar">
  <div class="topbar-left">
    <div class="brand">SKYE</div>

    {#if isIdle}
      <div class="control-group" aria-label="机台选择">
        <span class="control-label">机台</span>
        <div class="segmented">
          {#each ['thor', 'orin'] as p}
            <button
              type="button"
              class:selected={profile === p}
              on:click={() => (profile = p)}
            >
              {PROFILE_LABELS[p]}
            </button>
          {/each}
        </div>
      </div>

      <div class="control-group" aria-label="模式选择">
        <span class="control-label">模式</span>
        <div class="segmented">
          {#each ['teleop_record', 'dagger'] as m}
            <button
              type="button"
              class:selected={mode === m}
              on:click={() => (mode = m)}
            >
              {MODE_LABELS[m]}
            </button>
          {/each}
        </div>
      </div>

      <button type="button" class="btn btn-primary" on:click={handleStart}>
        启动会话
      </button>
    {:else}
      <div class="session-lock">
        <span class="lock-label">机台</span>
        <span class="lock-value">{PROFILE_LABELS[session.profile] ?? session.profile ?? '—'}</span>
        <span class="lock-sep">|</span>
        <span class="lock-label">模式</span>
        <span class="lock-value">{MODE_LABELS[session.mode] ?? session.mode ?? '—'}</span>
      </div>
    {/if}
  </div>

  <div class="topbar-center">
    <span class="badge {badgeClass}">
      {STATE_LABELS[sessionState] ?? sessionState}
    </span>
    {#if session.step}
      <span class="step-id mono">{session.step}</span>
    {/if}
  </div>

  <div class="topbar-right">
    <span class="ws-dot" class:connected={wsStatus === 'connected'} title={wsStatus === 'connected' ? '已连接' : '断线重连中'}></span>
    <time class="clock mono">{clock}</time>

    {#if !isIdle}
      {#if showStopConfirm}
        <div class="confirm-inline">
          <span>确认结束会话？</span>
          <button type="button" class="btn btn-amber" on:click={handleStop}>确认</button>
          <button type="button" class="btn btn-neutral" on:click={() => (showStopConfirm = false)}>取消</button>
        </div>
      {:else}
        <button type="button" class="btn btn-neutral" on:click={() => (showStopConfirm = true)}>
          结束会话
        </button>
      {/if}
    {/if}

    <button type="button" class="btn btn-estop" on:click={handleEmergencyStop}>
      急停
    </button>
  </div>
</header>

{#if apiError}
  <div class="topbar-error">{apiError}</div>
{/if}
