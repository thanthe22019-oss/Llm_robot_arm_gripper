"""Tests for deterministic planning from camera occupancy."""

import pytest

from ur3_llm_control.state_aware_planning import build_state_aware_plan
from ur3_llm_control.task_validator import PlanValidationError


def scene(zone_b_object=None, temp_object=None):
    locations = {
        'red_cube': 'table',
        'yellow_cube': 'table',
        'blue_cube': 'zone_b' if zone_b_object == 'blue_cube' else 'table',
        'green_cube': 'table',
        'purple_cube': 'temp_1' if temp_object == 'purple_cube' else 'table',
    }
    return {
        'version': 4,
        'source': 'camera',
        'valid': True,
        'objects': {
            name: {
                'visible': True,
                'location': location,
                'confidence': 0.96,
            }
            for name, location in locations.items()
        },
        'zones': {
            'zone_a': {'occupied': False, 'object': None},
            'zone_b': {
                'occupied': zone_b_object is not None,
                'object': zone_b_object,
            },
            'zone_c': {'occupied': False, 'object': None},
        },
        'temporary_slots': {
            'temp_1': {
                'occupied': temp_object is not None,
                'object': temp_object,
            },
        },
    }


def test_empty_zone_uses_direct_pick_place_plan():
    document = build_state_aware_plan(scene(), 'red_cube', 'zone_b')
    assert document['scene_version'] == 4
    assert [step['skill'] for step in document['plan']] == [
        'check_zone', 'pick', 'place', 'home'
    ]


def test_occupied_zone_moves_occupant_to_free_temp_first():
    document = build_state_aware_plan(
        scene(zone_b_object='blue_cube'), 'red_cube', 'zone_b'
    )
    assert document['plan'] == [
        {'skill': 'check_zone', 'zone': 'zone_b'},
        {'skill': 'pick', 'object': 'blue_cube'},
        {'skill': 'place_temp', 'object': 'blue_cube', 'slot': 'temp_1'},
        {'skill': 'home'},
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'},
        {'skill': 'home'},
    ]


def test_occupied_zone_without_free_temp_stops_safely():
    with pytest.raises(PlanValidationError) as caught:
        build_state_aware_plan(
            scene(zone_b_object='blue_cube', temp_object='purple_cube'),
            'red_cube',
            'zone_b',
        )
    assert caught.value.code == 'NO_FREE_TEMP_SLOT'
