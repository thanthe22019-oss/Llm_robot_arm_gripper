"""Tests for the Milestone 4 JSON plan validator."""

import json
from pathlib import Path

import pytest

from ur3_llm_control.student_config import (
    load_student_config,
    single_pick_place_plan,
)
from ur3_llm_control.task_validator import (
    PlanValidationError,
    parse_and_validate_plan,
    validate_state_aware_plan,
    validate_plan_document,
)


STUDENT_FILE = Path(__file__).parents[1] / 'config' / 'student_config.yaml'


def assert_rejected(document, code):
    """Assert a decoded plan is rejected atomically with the expected code."""
    with pytest.raises(PlanValidationError) as caught:
        validate_plan_document(document)
    assert caught.value.code == code


def test_valid_pick_place_home_plan():
    plan = parse_and_validate_plan(json.dumps({
        'plan': [
            {'skill': 'pick', 'object': 'red_cube'},
            {
                'skill': 'place',
                'object': 'red_cube',
                'zone': 'zone_a',
            },
            {'skill': 'home'},
        ],
    }))
    assert [step.label() for step in plan] == [
        'pick(red_cube)',
        'place(red_cube, zone_a)',
        'home()',
    ]


def test_student_23020730_plan_is_personalised_and_valid():
    config = load_student_config(str(STUDENT_FILE))
    document = single_pick_place_plan(config, 'red_cube')
    plan = validate_plan_document(document)
    assert [step.as_dict() for step in plan] == [
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_a'},
        {'skill': 'home'},
    ]


@pytest.mark.parametrize('raw_json', ['', '{', '{"plan": [}'])
def test_invalid_json_is_rejected(raw_json):
    with pytest.raises(PlanValidationError) as caught:
        parse_and_validate_plan(raw_json)
    assert caught.value.code == 'INVALID_JSON'


@pytest.mark.parametrize(
    ('document', 'code'),
    [
        ({}, 'MISSING_PLAN'),
        ({'plan': 'pick'}, 'INVALID_PLAN'),
        ({'plan': []}, 'EMPTY_PLAN'),
        ({'plan': ['pick']}, 'INVALID_STEP'),
        ({'plan': [{}]}, 'MISSING_FIELD'),
        ({'plan': [{'skill': 'destroy_robot'}]}, 'INVALID_SKILL'),
        (
            {'plan': [{'skill': 'pick', 'object': 'orange_cube'}]},
            'INVALID_OBJECT',
        ),
        (
            {
                'plan': [
                    {'skill': 'pick', 'object': 'red_cube'},
                    {
                        'skill': 'place',
                        'object': 'red_cube',
                        'zone': 'zone_x',
                    },
                ],
            },
            'INVALID_ZONE',
        ),
        (
            {
                'plan': [
                    {
                        'skill': 'pick',
                        'object': 'red_cube',
                        'zone': 'zone_a',
                    },
                ],
            },
            'UNEXPECTED_FIELD',
        ),
        (
            {
                'plan': [
                    {
                        'skill': 'place',
                        'object': 'red_cube',
                        'zone': 'zone_a',
                    },
                ],
            },
            'INVALID_SEQUENCE',
        ),
        (
            {
                'plan': [
                    {'skill': 'pick', 'object': 'red_cube'},
                    {'skill': 'home'},
                ],
            },
            'INVALID_SEQUENCE',
        ),
        (
            {
                'plan': [
                    {'skill': 'pick', 'object': 'red_cube'},
                    {
                        'skill': 'place',
                        'object': 'blue_cube',
                        'zone': 'zone_c',
                    },
                ],
            },
            'INVALID_SEQUENCE',
        ),
    ],
)
def test_invalid_documents_are_rejected(document, code):
    assert_rejected(document, code)


@pytest.mark.parametrize(
    'forbidden_field',
    [
        'joint_position',
        'joint_trajectory',
        'velocity',
        'torque',
        'effort',
        'controller_command',
    ],
)
def test_direct_motion_control_is_rejected(forbidden_field):
    assert_rejected(
        {
            'plan': [
                {
                    'skill': 'home',
                    forbidden_field: [0.0],
                },
            ],
        },
        'FORBIDDEN_CONTROL',
    )


def test_plan_cannot_end_while_holding_an_object():
    assert_rejected(
        {'plan': [{'skill': 'pick', 'object': 'blue_cube'}]},
        'INVALID_SEQUENCE',
    )


def test_plan_may_move_multiple_objects_sequentially():
    plan = validate_plan_document({
        'plan': [
            {'skill': 'pick', 'object': 'red_cube'},
            {
                'skill': 'place',
                'object': 'red_cube',
                'zone': 'zone_a',
            },
            {'skill': 'home'},
            {'skill': 'pick', 'object': 'yellow_cube'},
            {
                'skill': 'place',
                'object': 'yellow_cube',
                'zone': 'zone_b',
            },
            {'skill': 'home'},
        ],
    })
    assert len(plan) == 6


def test_plan_must_return_home_between_pick_place_tasks():
    assert_rejected(
        {
            'plan': [
                {'skill': 'pick', 'object': 'red_cube'},
                {
                    'skill': 'place',
                    'object': 'red_cube',
                    'zone': 'zone_a',
                },
                {'skill': 'pick', 'object': 'yellow_cube'},
                {
                    'skill': 'place',
                    'object': 'yellow_cube',
                    'zone': 'zone_b',
                },
                {'skill': 'home'},
            ],
        },
        'INVALID_SEQUENCE',
    )


def test_plan_must_return_home_after_final_place():
    assert_rejected(
        {
            'plan': [
                {'skill': 'pick', 'object': 'blue_cube'},
                {
                    'skill': 'place',
                    'object': 'blue_cube',
                    'zone': 'zone_c',
                },
            ],
        },
        'INVALID_SEQUENCE',
    )


def blocked_scene(version=7):
    """Return a valid camera snapshot with blue_cube occupying zone_b."""
    positions = {
        'red_cube': 'table',
        'yellow_cube': 'table',
        'blue_cube': 'zone_b',
        'green_cube': 'table',
        'purple_cube': 'table',
    }
    return {
        'version': version,
        'source': 'camera',
        'valid': True,
        'fresh': True,
        'objects': {
            name: {
                'visible': True,
                'location': location,
                'confidence': 0.95,
            }
            for name, location in positions.items()
        },
        'zones': {
            'zone_a': {'occupied': False, 'object': None},
            'zone_b': {'occupied': True, 'object': 'blue_cube'},
            'zone_c': {'occupied': False, 'object': None},
        },
        'temporary_slots': {
            'temp_1': {'occupied': False, 'object': None},
        },
    }


def blocked_relocation_plan(version=7):
    return {
        'scene_version': version,
        'goal': {'object': 'red_cube', 'destination': 'zone_b'},
        'plan': [
            {'skill': 'check_zone', 'zone': 'zone_b'},
            {'skill': 'pick', 'object': 'blue_cube'},
            {'skill': 'place_temp', 'object': 'blue_cube', 'slot': 'temp_1'},
            {'skill': 'home'},
            {'skill': 'pick', 'object': 'red_cube'},
            {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'},
            {'skill': 'home'},
        ],
    }


def test_state_aware_validator_accepts_blocked_zone_relocation():
    plan = validate_state_aware_plan(blocked_relocation_plan(), blocked_scene())
    assert [step.label() for step in plan] == [
        'check_zone(zone_b)',
        'pick(blue_cube)',
        'place_temp(blue_cube, temp_1)',
        'home()',
        'pick(red_cube)',
        'place(red_cube, zone_b)',
        'home()',
    ]


def test_state_aware_validator_accepts_empty_zone_plan():
    scene = blocked_scene()
    scene['objects']['blue_cube']['location'] = 'table'
    scene['zones']['zone_b'] = {'occupied': False, 'object': None}
    document = {
        'scene_version': 7,
        'goal': {'object': 'red_cube', 'destination': 'zone_a'},
        'plan': [
            {'skill': 'check_zone', 'zone': 'zone_a'},
            {'skill': 'pick', 'object': 'red_cube'},
            {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_a'},
            {'skill': 'home'},
        ],
    }
    plan = validate_state_aware_plan(document, scene)
    assert plan[-1].skill == 'home'


def test_state_aware_validator_rejects_direct_place_into_occupied_zone():
    document = {
        'scene_version': 7,
        'goal': {'object': 'red_cube', 'destination': 'zone_b'},
        'plan': [
            {'skill': 'check_zone', 'zone': 'zone_b'},
            {'skill': 'pick', 'object': 'red_cube'},
            {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'},
            {'skill': 'home'},
        ],
    }
    with pytest.raises(PlanValidationError) as caught:
        validate_state_aware_plan(document, blocked_scene())
    assert caught.value.code == 'OCCUPIED_DESTINATION'


def test_state_aware_validator_rejects_stale_scene_version():
    with pytest.raises(PlanValidationError) as caught:
        validate_state_aware_plan(blocked_relocation_plan(6), blocked_scene(7))
    assert caught.value.code == 'STALE_SCENE'


def test_state_aware_validator_rejects_low_confidence_pick():
    scene = blocked_scene()
    scene['objects']['blue_cube']['confidence'] = 0.40
    with pytest.raises(PlanValidationError) as caught:
        validate_state_aware_plan(blocked_relocation_plan(), scene)
    assert caught.value.code == 'LOW_CONFIDENCE'


def test_state_aware_validator_rejects_when_no_temp_slot_is_free():
    scene = blocked_scene()
    scene['objects']['purple_cube']['location'] = 'temp_1'
    scene['temporary_slots']['temp_1'] = {
        'occupied': True,
        'object': 'purple_cube',
    }
    with pytest.raises(PlanValidationError) as caught:
        validate_state_aware_plan(blocked_relocation_plan(), scene)
    assert caught.value.code == 'NO_FREE_TEMP_SLOT'


def test_state_aware_validator_rejects_missing_zone_check():
    document = blocked_relocation_plan()
    document['plan'] = document['plan'][1:]
    with pytest.raises(PlanValidationError) as caught:
        validate_state_aware_plan(document, blocked_scene())
    assert caught.value.code == 'MISSING_ZONE_CHECK'


def test_state_aware_validator_rejects_stale_camera_flag():
    scene = blocked_scene()
    scene['fresh'] = False
    with pytest.raises(PlanValidationError) as caught:
        validate_state_aware_plan(blocked_relocation_plan(), scene)
    assert caught.value.code == 'STALE_SCENE'
