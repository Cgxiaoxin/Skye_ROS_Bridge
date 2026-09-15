<script>
  /** v1: display-only joint bars; fixed ±π rad scale. */

  export let snapshot = null;

  const JOINT_MIN = -3.14159;
  const JOINT_MAX = 3.14159;
  const JOINT_LABELS = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7'];

  $: leftJoints = snapshot?.arms?.left ?? [];
  $: rightJoints = snapshot?.arms?.right ?? [];
  $: leftGripper = snapshot?.grippers?.left;
  $: rightGripper = snapshot?.grippers?.right;

  function barPercent(value) {
    if (value == null || Number.isNaN(value)) return 50;
    const clamped = Math.max(JOINT_MIN, Math.min(JOINT_MAX, value));
    return ((clamped - JOINT_MIN) / (JOINT_MAX - JOINT_MIN)) * 100;
  }

  function fmt(value) {
    if (value == null || Number.isNaN(value)) return '—';
    return Number(value).toFixed(3);
  }
</script>

<section class="arm-block">
  <h3 class="arm-title">左臂</h3>
  <div class="joint-grid">
    {#each JOINT_LABELS as label, i}
      {@const val = leftJoints[i]}
      <div class="joint-row">
        <span class="joint-label mono">{label}</span>
        <div class="joint-bar-track">
          <div class="joint-bar-fill" style="width: {barPercent(val)}%"></div>
        </div>
        <span class="joint-val mono">{fmt(val)}</span>
      </div>
    {/each}
  </div>
  <p class="gripper-line mono">夹爪: {fmt(leftGripper)}</p>
</section>

<section class="arm-block">
  <h3 class="arm-title">右臂</h3>
  <div class="joint-grid">
    {#each JOINT_LABELS as label, i}
      {@const val = rightJoints[i]}
      <div class="joint-row">
        <span class="joint-label mono">{label}</span>
        <div class="joint-bar-track">
          <div class="joint-bar-fill" style="width: {barPercent(val)}%"></div>
        </div>
        <span class="joint-val mono">{fmt(val)}</span>
      </div>
    {/each}
  </div>
  <p class="gripper-line mono">夹爪: {fmt(rightGripper)}</p>
</section>
