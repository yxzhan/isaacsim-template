from cup_to_sink import success

CFG = {
    "xy_threshold": 0.06,
    "z_min_offset": -0.03,
    "z_max_offset": 0.15,
}


def test_success_true():
    ok, info = success.check([1.0, 1.0, 0.85], [1.01, 1.0, 0.80], 0.08, CFG, 0.08)
    assert ok and info["xy_ok"] and info["z_ok"] and info["gripper_ok"]


def test_fail_xy():
    ok, _ = success.check([1.5, 1.0, 0.85], [1.0, 1.0, 0.80], 0.08, CFG, 0.08)
    assert not ok


def test_fail_gripper_closed():
    ok, info = success.check([1.0, 1.0, 0.85], [1.0, 1.0, 0.80], 0.0, CFG, 0.08)
    assert not ok and not info["gripper_ok"]
