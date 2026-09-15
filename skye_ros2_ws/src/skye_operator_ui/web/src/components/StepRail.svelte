<script>
  import { createEventDispatcher } from 'svelte';
  import { stepsForMode, stepStatus } from '../lib/steps.js';

  export let snapshot = null;

  const dispatch = createEventDispatcher();

  $: session = snapshot?.session ?? {};
  $: mode = session.mode;
  $: steps = stepsForMode(mode);
  $: healthMap = healthByKey(snapshot?.health);

  function healthByKey(healthList) {
    const map = {};
    for (const item of healthList ?? []) {
      map[item.key] = item.ok;
    }
    return map;
  }

  function ctx() {
    return {
      mode,
      sessionState: session.state ?? 'IDLE',
      currentStepId: session.step,
      teleopState: snapshot?.teleop?.state,
      alignStatus: snapshot?.align?.status,
      hitlMode: snapshot?.hitl?.mode,
      recording: snapshot?.recording?.active,
      health: healthMap,
    };
  }

  function openLogs(step) {
    if (!step.logId) return;
    dispatch('openLogs', { stepId: step.logId, stepLabel: step.label });
  }
</script>

{#if !mode}
  <p class="panel-placeholder">选择模式并启动会话后显示步骤</p>
{:else}
  <ol class="step-rail">
    {#each steps as step (step.id)}
      {@const status = stepStatus(step, ctx())}
      <li class="step-item step-{status}">
        <button
          type="button"
          class="step-btn"
          class:clickable={!!step.logId}
          disabled={!step.logId}
          on:click={() => openLogs(step)}
          title={step.logId ? '点击查看日志' : ''}
        >
          <span class="step-marker" aria-hidden="true"></span>
          <span class="step-label">{step.label}</span>
          {#if step.optional}
            <span class="step-optional">可选</span>
          {/if}
        </button>
      </li>
    {/each}
  </ol>
{/if}

{#if snapshot?.health?.length}
  <h3 class="panel-subtitle">健康</h3>
  <ul class="health-list">
    {#each snapshot.health as item}
      <li class:ok={item.ok} class:bad={!item.ok}>
        <span class="health-dot"></span>
        <span class="mono">{item.key}</span>
      </li>
    {/each}
  </ul>
{/if}
