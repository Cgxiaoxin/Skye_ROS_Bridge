<script>
  export let coach = null;
  export let enterLabel = '开遥操';

  $: visible = !!coach?.visible;
  $: left = coach?.left;
  $: right = coach?.right;

  function sideTitle(side) {
    return side === 'left' ? 'LEFT' : 'RIGHT';
  }

  function sideFailed(side) {
    return side && !side.disabled && side.outcome === 'failed';
  }

  function failureMessage(side) {
    if (!side?.raw_text) return '同步失败；已停止并关闭力矩';
    const firstLine = side.raw_text.split('\n')[0]?.trim() || '';
    if (firstLine.includes('同步失败')) return firstLine;
    return '同步失败；已停止并关闭力矩';
  }

  function formatBlocker(b) {
    const kind = b.kind === 'vel' ? '|vel|' : '|pos err|';
    const unit = b.kind === 'vel' ? 'rad/s' : 'rad';
    return `${b.axis} · ${kind} ${Number(b.value).toFixed(3)} > ${Number(b.limit).toFixed(3)} ${unit}`;
  }

  $: anyFailed = sideFailed(left) || sideFailed(right);
  $: failBanner = sideFailed(left)
    ? failureMessage(left)
    : sideFailed(right)
      ? failureMessage(right)
      : '';
  $: pillLabel = anyFailed ? 'FAILED' : left?.mode || right?.mode || 'SYNCING';
</script>

{#if visible}
  <section class="sync-coach" aria-live="polite">
    <div class="sync-coach-head">
      <strong>同步校正</strong>
      <span class="sync-coach-pill mono" class:sync-coach-pill-fail={anyFailed}>
        {pillLabel}
      </span>
    </div>
    {#if anyFailed}
      <p class="sync-coach-fail">{failBanner}</p>
    {/if}
    <div class="sync-coach-grid">
      {#each [['left', left], ['right', right]] as [name, side] (name)}
        <div
          class="sync-coach-side"
          class:sync-coach-side-warn={side &&
            !side.disabled &&
            (side.blockers?.length || side.stale || side.outcome === 'failed')}
          class:sync-coach-side-ok={side &&
            !side.disabled &&
            !side.stale &&
            !(side.blockers?.length) &&
            side.outcome !== 'failed'}
          class:sync-coach-side-off={side?.disabled}
        >
          <div class="sync-coach-side-head">
            <span>{sideTitle(name)}</span>
            {#if side?.disabled}
              <span class="mono">关</span>
            {:else if side?.within != null && side?.total != null}
              <span class="mono">{side.within}/{side.total}</span>
            {/if}
          </div>
          {#if side?.disabled}
            <p class="sync-coach-muted">已关闭</p>
          {:else if side?.outcome === 'failed'}
            <p class="sync-coach-fail-side">{failureMessage(side)}</p>
          {:else if side?.stale}
            <p class="sync-coach-muted">反馈超时</p>
          {:else if !side?.parse_ok && side?.raw_text}
            <pre class="sync-coach-raw mono">{side.raw_text}</pre>
          {:else if side?.blockers?.length}
            <ul class="sync-coach-blockers mono">
              {#each side.blockers as b, i (i)}
                <li>{formatBlocker(b)}</li>
              {/each}
            </ul>
          {:else}
            <p class="sync-coach-ok">瞬时达标，保持中…</p>
          {/if}
          {#if side?.settling && !side?.disabled && side?.outcome !== 'failed'}
            <p class="sync-coach-muted mono">
              settling {Number(side.settling.current_s).toFixed(2)} / {Number(side.settling.need_s).toFixed(2)} s
            </p>
          {/if}
        </div>
      {/each}
    </div>
    <p class="sync-coach-hint">手动微调超限关节，左右均 SYNCED 后再{enterLabel}</p>
  </section>
{/if}
