"""Tests for cup_to_sink.skills — pure numpy, no Isaac Sim."""

import numpy as np
import pytest

from cup_to_sink.actors import CupActor
from cup_to_sink.skills import grasp_actor, move_by_displacement, place_actor


# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

class FakeCup:
    """Minimal stand-in for env.cup; cup at (1, 2, 0.8) with identity orient."""
    def get_world_pose(self):
        return (np.array([1.0, 2.0, 0.8]), np.array([1.0, 0.0, 0.0, 0.0]))


CFG = {
    "contact_points": [
        {
            "name": "side_grasp",
            "position_obj": [0.045, 0.0, 0.08],
            "quat_obj": [1.0, 0.0, 0.0, 0.0],
            "pregrasp_distance": 0.10,
        }
    ],
    "functional_points": [
        {"name": "cup_center", "position_obj": [0.0, 0.0, 0.0], "quat_obj": [1.0, 0.0, 0.0, 0.0]}
    ],
}

PRE_GRASP_DIS = 0.12
GRASP_DIS = 0.0
GRIPPER_TARGET = 0.03


# ---------------------------------------------------------------------------
# grasp_actor tests
# ---------------------------------------------------------------------------

def test_grasp_actor_action_types():
    """grasp_actor must return exactly 4 actions with types [gripper,move,move,gripper]."""
    actor = CupActor(FakeCup(), CFG)
    actions = grasp_actor(actor, PRE_GRASP_DIS, GRASP_DIS, GRIPPER_TARGET, contact_point_id=0)
    assert len(actions) == 4
    types = [a["type"] for a in actions]
    assert types == ["gripper", "move", "move", "gripper"], f"Got types: {types}"


def test_grasp_actor_phases():
    actor = CupActor(FakeCup(), CFG)
    actions = grasp_actor(actor, PRE_GRASP_DIS, GRASP_DIS, GRIPPER_TARGET)
    assert actions[0]["phase"] == "pregrasp"
    assert actions[1]["phase"] == "pregrasp"
    assert actions[2]["phase"] == "grasp"
    assert actions[3]["phase"] == "grasp_close"


def test_grasp_actor_pregrasp_farther_than_grasp():
    """Pregrasp pose must be farther from the cup contact point than the grasp pose."""
    actor = CupActor(FakeCup(), CFG)
    actions = grasp_actor(actor, PRE_GRASP_DIS, GRASP_DIS, GRIPPER_TARGET, contact_point_id=0)

    contact_pos = actor.get_contact_point(0)[:3]
    pregrasp_pos = actions[1]["ee_pose7d"][:3]
    grasp_pos = actions[2]["ee_pose7d"][:3]

    dist_pregrasp = np.linalg.norm(pregrasp_pos - contact_pos)
    dist_grasp = np.linalg.norm(grasp_pos - contact_pos)

    assert dist_pregrasp > dist_grasp, (
        f"Pregrasp ({dist_pregrasp:.4f} m) should be farther than grasp ({dist_grasp:.4f} m)"
    )


def test_grasp_actor_pregrasp_offset_magnitude():
    """Pregrasp should be offset by pre_grasp_dis along the approach axis."""
    actor = CupActor(FakeCup(), CFG)
    actions = grasp_actor(actor, PRE_GRASP_DIS, GRASP_DIS, GRIPPER_TARGET)

    pregrasp_pos = actions[1]["ee_pose7d"][:3]
    grasp_pos = actions[2]["ee_pose7d"][:3]

    offset = np.linalg.norm(pregrasp_pos - grasp_pos)
    np.testing.assert_allclose(offset, PRE_GRASP_DIS, atol=1e-6)


def test_grasp_actor_gripper_open_width():
    actor = CupActor(FakeCup(), CFG)
    actions = grasp_actor(actor, PRE_GRASP_DIS, GRASP_DIS, GRIPPER_TARGET)
    assert actions[0]["width"] == 0.08, "First gripper action should be fully open (0.08 m)"


def test_grasp_actor_gripper_close_width():
    actor = CupActor(FakeCup(), CFG)
    actions = grasp_actor(actor, PRE_GRASP_DIS, GRASP_DIS, GRIPPER_TARGET)
    assert actions[3]["width"] == GRIPPER_TARGET


# ---------------------------------------------------------------------------
# move_by_displacement tests
# ---------------------------------------------------------------------------

def test_move_by_displacement_returns_one_action():
    pose = np.array([1.0, 2.0, 0.5, 1.0, 0.0, 0.0, 0.0])
    result = move_by_displacement(pose, 0.1, -0.2, 0.05)
    assert len(result) == 1
    assert result[0]["type"] == "move"


def test_move_by_displacement_position():
    pose = np.array([1.0, 2.0, 0.5, 1.0, 0.0, 0.0, 0.0])
    result = move_by_displacement(pose, 0.1, -0.2, 0.05)
    target = result[0]["ee_pose7d"]
    np.testing.assert_allclose(target[:3], [1.1, 1.8, 0.55], atol=1e-9)


def test_move_by_displacement_orientation_unchanged():
    pose = np.array([1.0, 2.0, 0.5, 0.707, 0.0, 0.707, 0.0])
    result = move_by_displacement(pose, 0.0, 0.0, 0.1)
    np.testing.assert_allclose(result[0]["ee_pose7d"][3:], pose[3:], atol=1e-9)


def test_move_by_displacement_no_mutation():
    pose = np.array([1.0, 2.0, 0.5, 1.0, 0.0, 0.0, 0.0])
    original = pose.copy()
    move_by_displacement(pose, 1.0, 1.0, 1.0)
    np.testing.assert_array_equal(pose, original)


# ---------------------------------------------------------------------------
# place_actor tests
# ---------------------------------------------------------------------------

TARGET_POSE = np.array([1.5, -0.4, 0.70, 1.0, 0.0, 0.0, 0.0])
PRE_DIS = 0.06
RETREAT_H = 0.08


def test_place_actor_returns_4_actions():
    actions = place_actor(None, TARGET_POSE, PRE_DIS, RETREAT_H, is_open=True)
    assert len(actions) == 4


def test_place_actor_action_types_open():
    actions = place_actor(None, TARGET_POSE, PRE_DIS, RETREAT_H, is_open=True)
    types = [a["type"] for a in actions]
    assert types == ["move", "move", "gripper", "move"]


def test_place_actor_action_types_no_open():
    actions = place_actor(None, TARGET_POSE, PRE_DIS, RETREAT_H, is_open=False)
    # Without gripper open: [preplace, place, retreat]
    assert len(actions) == 3
    types = [a["type"] for a in actions]
    assert types == ["move", "move", "move"]


def test_place_actor_phases():
    actions = place_actor(None, TARGET_POSE, PRE_DIS, RETREAT_H, is_open=True)
    assert actions[0]["phase"] == "preplace"
    assert actions[1]["phase"] == "place"
    assert actions[2]["phase"] == "release"
    assert actions[3]["phase"] == "retreat"


def test_place_actor_preplace_above_target():
    actions = place_actor(None, TARGET_POSE, PRE_DIS, RETREAT_H, is_open=True)
    preplace_z = actions[0]["ee_pose7d"][2]
    place_z = actions[1]["ee_pose7d"][2]
    np.testing.assert_allclose(preplace_z, place_z + PRE_DIS, atol=1e-9)


def test_place_actor_retreat_above_place():
    actions = place_actor(None, TARGET_POSE, PRE_DIS, RETREAT_H, is_open=True)
    retreat_action = actions[3]
    assert retreat_action["phase"] == "retreat"
    retreat_z = retreat_action["ee_pose7d"][2]
    place_z = actions[1]["ee_pose7d"][2]
    assert retreat_z > place_z, f"retreat z ({retreat_z}) must be above place z ({place_z})"
    np.testing.assert_allclose(retreat_z - place_z, RETREAT_H, atol=1e-9)


def test_place_actor_no_mutation():
    target_copy = TARGET_POSE.copy()
    place_actor(None, TARGET_POSE, PRE_DIS, RETREAT_H)
    np.testing.assert_array_equal(TARGET_POSE, target_copy)


def test_grasp_actor_pregrasp_direction():
    """Verify pregrasp world direction: position must be along the approach axis.

    Setup: identity quat, contact point at [0.045, 0, 0.08], so world contact at
    [1.045, 2.0, 0.88]. With identity grasp orientation, approach axis is world +Z.
    With pre_grasp_dis=0.12, pregrasp position should be [1.045, 2.0, 0.88 - 0.12]
    = [1.045, 2.0, 0.76].
    """
    actor = CupActor(FakeCup(), CFG)
    actions = grasp_actor(actor, pre_grasp_dis=0.12, grasp_dis=0.0,
                          gripper_width_target=0.0, contact_point_id=0)

    # Locate pregrasp and grasp actions by their phase field
    pregrasp_action = None
    grasp_action = None
    for action in actions:
        if action.get("phase") == "pregrasp" and action.get("type") == "move":
            pregrasp_action = action
        elif action.get("phase") == "grasp" and action.get("type") == "move":
            grasp_action = action

    assert pregrasp_action is not None, "No pregrasp move action found"
    assert grasp_action is not None, "No grasp move action found"

    # Verify pregrasp position: should be [1.045, 2.0, 0.76]
    pregrasp_pos = pregrasp_action["ee_pose7d"][:3]
    np.testing.assert_allclose(pregrasp_pos, np.array([1.045, 2.0, 0.76]), atol=1e-6)
