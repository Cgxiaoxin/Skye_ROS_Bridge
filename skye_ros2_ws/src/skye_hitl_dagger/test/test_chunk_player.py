import math

from skye_hitl_dagger.chunk_player import ChunkPlayer


def _flat(steps, dof, fill):
    return [float(fill)] * (steps * dof)


def test_step0_at_t0():
    p = ChunkPlayer()
    ok = p.load(16, 0.1, 10.0, _flat(16, 7, 1.0), _flat(16, 7, 2.0),
                [0.0]*16, [1.0]*16)
    assert ok
    s = p.sample(10.0)
    assert s["left"][0] == 1.0
    assert s["holding_tail"] is False


def test_hold_last_step_after_horizon():
    p = ChunkPlayer()
    left = _flat(16, 7, 0.0)
    left[15*7] = 3.14  # last step j1
    p.load(16, 0.1, 0.0, left, _flat(16, 7, 0.0), [0.0]*16, [0.0]*16)
    s = p.sample(10.0)
    assert s["left"][0] == 3.14
    assert s["holding_tail"] is True


def test_reject_bad_size_keeps_previous():
    p = ChunkPlayer()
    p.load(16, 0.1, 0.0, _flat(16, 7, 1.0), _flat(16, 7, 1.0), [0.0]*16, [0.0]*16)
    assert p.load(16, 0.1, 1.0, [0.0]*7, _flat(16, 7, 1.0), [0.0]*16, [0.0]*16) is False
    assert p.sample(0.0)["left"][0] == 1.0


def test_mid_interval_stays_on_current_step():
    p = ChunkPlayer()
    left = _flat(16, 7, 0.0)
    left[7] = 2.5  # step 1 marker (should not be selected)
    p.load(16, 0.1, 10.0, left, _flat(16, 7, 0.0), [0.0] * 16, [0.0] * 16)
    s = p.sample(10.051)
    assert s["left"][0] == 0.0
    assert s["holding_tail"] is False


def test_just_before_step1_boundary_stays_on_step0():
    p = ChunkPlayer()
    left = _flat(16, 7, 0.0)
    left[7] = 2.5  # step 1 marker
    p.load(16, 0.1, 10.0, left, _flat(16, 7, 0.0), [0.0] * 16, [0.0] * 16)
    s = p.sample(10.1 - 1e-15)
    assert s["left"][0] == 0.0
    assert s["holding_tail"] is False


def test_just_before_next_step_stays_on_current_step():
    p = ChunkPlayer()
    left = _flat(16, 7, 0.0)
    left[7] = 2.5  # step 1 marker
    p.load(16, 0.1, 10.0, left, _flat(16, 7, 0.0), [0.0] * 16, [0.0] * 16)
    s = p.sample(10.0 + 0.099999999)
    assert s["left"][0] == 0.0
    assert s["holding_tail"] is False


def test_exact_last_step_start_at_15dt():
    dt = 0.03
    p = ChunkPlayer()
    left = _flat(16, 7, 0.0)
    left[15 * 7] = 3.14
    p.load(16, dt, 0.0, left, _flat(16, 7, 0.0), [0.0] * 16, [0.0] * 16)
    s = p.sample(15 * 0.03)
    assert s["left"][0] == 3.14
    assert s["holding_tail"] is False


def test_exact_horizon_last_step_holding():
    dt = 0.03
    p = ChunkPlayer()
    left = _flat(16, 7, 0.0)
    left[15 * 7] = 9.9
    p.load(16, dt, 0.0, left, _flat(16, 7, 0.0), [0.0] * 16, [0.0] * 16)
    s = p.sample(16 * 0.03)
    assert s["left"][0] == 9.9
    assert s["holding_tail"] is True


def test_just_after_horizon_holding_tail():
    p = ChunkPlayer()
    left = _flat(16, 7, 0.0)
    left[15 * 7] = 3.14
    dt = 0.1
    p.load(16, dt, 0.0, left, _flat(16, 7, 0.0), [0.0] * 16, [0.0] * 16)
    s = p.sample(16 * dt + 1e-6)
    assert s["left"][0] == 3.14
    assert s["holding_tail"] is True


def test_nonzero_t0_last_step_at_15dt():
    dt = 0.03
    for t0 in (10.0, 1e6):
        p = ChunkPlayer()
        left = _flat(16, 7, 0.0)
        left[15 * 7] = 3.14
        p.load(16, dt, t0, left, _flat(16, 7, 0.0), [0.0] * 16, [0.0] * 16)
        s = p.sample(t0 + 15 * dt)
        assert s["left"][0] == 3.14
        assert s["holding_tail"] is False


def test_nonzero_t0_mid_interval_step0():
    for t0 in (10.0, 1e6):
        p = ChunkPlayer()
        left = _flat(16, 7, 0.0)
        left[7] = 2.5
        p.load(16, 0.1, t0, left, _flat(16, 7, 0.0), [0.0] * 16, [0.0] * 16)
        s = p.sample(t0 + 0.051)
        assert s["left"][0] == 0.0
        assert s["holding_tail"] is False


def test_unix_epoch_t0_last_step_holding():
    t0 = 1e9
    dt = 0.1
    p = ChunkPlayer()
    left = _flat(16, 7, 0.0)
    left[15 * 7] = 3.14
    p.load(16, dt, t0, left, _flat(16, 7, 0.0), [0.0] * 16, [0.0] * 16)
    s = p.sample(t0 + 15 * dt)
    assert s["left"][0] == 3.14
    assert s["holding_tail"] is False
    s = p.sample(t0 + 16 * dt)
    assert s["left"][0] == 3.14
    assert s["holding_tail"] is True


def test_reject_non_finite_dt_t0_keeps_previous():
    p = ChunkPlayer()
    p.load(16, 0.1, 0.0, _flat(16, 7, 1.0), _flat(16, 7, 1.0), [0.0] * 16, [0.0] * 16)
    assert p.load(16, math.nan, 0.0, _flat(16, 7, 2.0), _flat(16, 7, 2.0),
                  [0.0] * 16, [0.0] * 16) is False
    assert p.load(16, 0.1, math.inf, _flat(16, 7, 2.0), _flat(16, 7, 2.0),
                  [0.0] * 16, [0.0] * 16) is False
    assert p.sample(0.0)["left"][0] == 1.0
