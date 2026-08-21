from skye_hitl_dagger.hitl_keyboard_node import map_key


def test_map_key_takeover():
    assert map_key("q") == "takeover"
    assert map_key("Q") == "takeover"
    assert map_key("q\n") == "takeover"


def test_map_key_return():
    assert map_key("w") == "return"
    assert map_key("W") == "return"


def test_map_key_ignores_other_keys():
    assert map_key("a") is None
    assert map_key("") is None
    assert map_key("   ") is None
