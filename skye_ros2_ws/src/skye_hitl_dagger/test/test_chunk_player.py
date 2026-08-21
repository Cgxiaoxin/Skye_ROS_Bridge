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
