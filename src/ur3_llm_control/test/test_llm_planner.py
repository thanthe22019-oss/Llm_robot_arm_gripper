"""Tests for the Milestone 6 mock natural-language planner."""

import json
from pathlib import Path

import pytest

from ur3_llm_control.llm_planner import MockLLMPlanner, PlannerError
from ur3_llm_control.student_config import load_student_config
from ur3_llm_control.task_validator import (
    parse_and_validate_plan,
    validate_state_aware_plan,
)


PACKAGE_DIR = Path(__file__).parents[1]
STUDENT_FILE = PACKAGE_DIR / 'config' / 'student_config.yaml'
PROMPT_FILE = PACKAGE_DIR / 'prompts' / 'planner_prompt.txt'


@pytest.fixture
def planner():
    return MockLLMPlanner(load_student_config(str(STUDENT_FILE)))


@pytest.mark.parametrize(
    ('command', 'object_name', 'zone_name'),
    [
        ('Đưa vật màu đỏ sang vùng B.', 'red_cube', 'zone_b'),
        (
            'Hãy lấy khối màu vàng và đặt nó vào ô A.',
            'yellow_cube',
            'zone_a',
        ),
        ('Move the blue cube to zone C.', 'blue_cube', 'zone_c'),
        ('Đưa khối xanh lá sang vùng A.', 'green_cube', 'zone_a'),
        ('Move the purple cube to zone B.', 'purple_cube', 'zone_b'),
        ('đưa KHỐI ĐỎ tới KHU-C', 'red_cube', 'zone_c'),
    ],
)
def test_required_commands_generate_valid_plans(
    planner,
    command,
    object_name,
    zone_name,
):
    raw_json = planner.generate_plan(command)
    steps = parse_and_validate_plan(raw_json)

    assert [step.as_dict() for step in steps] == [
        {'skill': 'pick', 'object': object_name},
        {
            'skill': 'place',
            'object': object_name,
            'zone': zone_name,
        },
        {'skill': 'home'},
    ]


def test_planner_returns_raw_json_without_markdown(planner):
    raw_json = planner.generate_plan('Move the red cube to zone A.')
    assert raw_json.startswith('{"plan":[')
    assert chr(96) * 3 not in raw_json
    assert json.loads(raw_json)['plan'][0]['skill'] == 'pick'


@pytest.mark.parametrize(
    'command',
    [
        '',
        'Đưa một vật sang vùng A',
        'Đưa khối đỏ đi đâu đó',
        'Move the orange cube to zone A',
        'Move red and blue cubes to zone A',
        'Move the red cube to zone A and zone B',
    ],
)
def test_ambiguous_or_incomplete_commands_are_rejected(planner, command):
    with pytest.raises(PlannerError):
        planner.generate_plan(command)


@pytest.mark.parametrize(
    'command',
    [
        'Arrange all objects according to my student ID.',
        'Sắp xếp tất cả vật theo MSSV.',
        'Sắp xếp theo mã sinh viên của tôi.',
    ],
)
def test_student_command_uses_real_23020730_mapping(planner, command):
    steps = parse_and_validate_plan(planner.generate_plan(command))
    assert [step.as_dict() for step in steps] == [
        {'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_a'},
        {'skill': 'home'},
        {'skill': 'pick', 'object': 'yellow_cube'},
        {'skill': 'place', 'object': 'yellow_cube', 'zone': 'zone_b'},
        {'skill': 'home'},
        {'skill': 'pick', 'object': 'blue_cube'},
        {'skill': 'place', 'object': 'blue_cube', 'zone': 'zone_c'},
        {'skill': 'home'},
    ]


def test_prompt_defines_safe_json_contract():
    prompt = PROMPT_FILE.read_text(encoding='utf-8').lower()
    for value in (
        'pick(object)',
        'place(object, zone)',
        'home()',
        'red_cube',
        'yellow_cube',
        'blue_cube',
        'green_cube',
        'purple_cube',
        'zone_a',
        'zone_b',
        'zone_c',
        'raw json only',
        'joint trajectories',
        'controller commands',
        'camera scene state',
        'place_temp(object, slot)',
    ):
        assert value in prompt


def test_mock_planner_uses_camera_occupancy_for_blocked_zone(planner):
    scene = {
        'version': 9,
        'source': 'camera',
        'valid': True,
        'objects': {
            name: {
                'visible': True,
                'location': 'zone_b' if name == 'blue_cube' else 'table',
                'confidence': 0.95,
            }
            for name in (
                'red_cube', 'yellow_cube', 'blue_cube',
                'green_cube', 'purple_cube'
            )
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
    document = json.loads(
        planner.generate_plan('Đưa khối đỏ sang vùng B.', scene)
    )
    steps = validate_state_aware_plan(document, scene)
    assert [step.skill for step in steps] == [
        'check_zone', 'pick', 'place_temp', 'home', 'pick', 'place', 'home'
    ]
