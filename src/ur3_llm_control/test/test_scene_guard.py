"""Tests for camera-state checks performed immediately before robot skills."""

from copy import deepcopy

import pytest

from ur3_llm_control.scene_guard import SceneChangedError, SceneGuard
from ur3_llm_control.task_validator import PlanStep


def camera_state():
    return {
        'version': 3,
        'source': 'camera',
        'valid': True,
        'objects': {
            'red_cube': {'visible': True, 'location': 'table'},
            'blue_cube': {'visible': True, 'location': 'zone_b'},
        },
        'zones': {
            'zone_a': {'occupied': False, 'object': None},
            'zone_b': {'occupied': True, 'object': 'blue_cube'},
        },
        'temporary_slots': {
            'temp_1': {'occupied': False, 'object': None},
        },
    }


def test_guard_accepts_expected_relocation_transitions():
    state = camera_state()
    guard = SceneGuard(state)
    pick = PlanStep('pick', 'blue_cube')
    place = PlanStep('place_temp', 'blue_cube', 'temp_1')
    guard.check_before(pick, state)
    guard.apply_success(pick)

    held_state = deepcopy(state)
    held_state['valid'] = False
    held_state['objects']['blue_cube'] = {
        'visible': False, 'location': 'unknown'
    }
    held_state['zones']['zone_b'] = {'occupied': False, 'object': None}
    guard.check_before(place, held_state)
    guard.apply_success(place)
    assert not guard.holding_object


def test_guard_rejects_version_change_before_first_skill():
    state = camera_state()
    changed = deepcopy(state)
    changed['version'] = 4
    with pytest.raises(SceneChangedError, match='changed from v3 to v4'):
        SceneGuard(state).check_before(PlanStep('pick', 'blue_cube'), changed)


def test_guard_rejects_destination_that_becomes_occupied():
    state = camera_state()
    guard = SceneGuard(state)
    guard.check_before(PlanStep('pick', 'red_cube'), state)
    guard.apply_success(PlanStep('pick', 'red_cube'))
    changed = deepcopy(state)
    changed['zones']['zone_a'] = {
        'occupied': True, 'object': 'blue_cube'
    }
    with pytest.raises(SceneChangedError, match='changed occupancy'):
        guard.check_before(PlanStep('place', 'red_cube', 'zone_a'), changed)
