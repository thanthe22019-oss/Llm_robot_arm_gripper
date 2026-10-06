"""Shared structured-output contract for external LLM planners."""

import json
from pathlib import Path

from ur3_llm_control.llm_planner import PlannerError
from ur3_llm_control.task_validator import (
    parse_and_validate_plan,
    validate_state_aware_plan,
)


# External providers receive the same closed schema. Schema-only null values
# are removed before the plan reaches the independent Plan Validator.
PLAN_RESPONSE_SCHEMA = {
    'type': 'object',
    'properties': {
        'scene_version': {'type': 'integer', 'minimum': 1},
        'goal': {
            'type': 'object',
            'properties': {
                'object': {
                    'type': 'string',
                    'enum': [
                        'red_cube', 'yellow_cube', 'blue_cube',
                        'green_cube', 'purple_cube',
                    ],
                },
                'destination': {
                    'type': 'string',
                    'enum': ['zone_a', 'zone_b', 'zone_c'],
                },
            },
            'required': ['object', 'destination'],
            'additionalProperties': False,
        },
        'plan': {
            'type': 'array',
            'minItems': 1,
            'maxItems': 30,
            'items': {
                'type': 'object',
                'properties': {
                    'skill': {
                        'type': 'string',
                        'enum': [
                            'observe_scene', 'check_zone', 'pick', 'place',
                            'place_temp', 'home',
                        ],
                    },
                    'object': {
                        'type': ['string', 'null'],
                        'enum': [
                            'red_cube',
                            'yellow_cube',
                            'blue_cube',
                            'green_cube',
                            'purple_cube',
                            None,
                        ],
                    },
                    'zone': {
                        'type': ['string', 'null'],
                        'enum': ['zone_a', 'zone_b', 'zone_c', None],
                    },
                    'slot': {
                        'type': ['string', 'null'],
                        'enum': ['temp_1', None],
                    },
                },
                'required': ['skill', 'object', 'zone', 'slot'],
                'additionalProperties': False,
            },
        },
    },
    'required': ['scene_version', 'goal', 'plan'],
    'additionalProperties': False,
}


def default_prompt() -> Path:
    """Return the installed prompt path, or the source-tree fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return (
            Path(get_package_share_directory('ur3_llm_control'))
            / 'prompts'
            / 'planner_prompt.txt'
        )
    except Exception:
        return Path(__file__).parents[1] / 'prompts' / 'planner_prompt.txt'


def canonical_json(raw_output: str, provider: str, scene_state=None) -> str:
    """Remove schema-only nulls and validate the canonical robot plan."""
    if not isinstance(raw_output, str) or not raw_output.strip():
        raise PlannerError(f'{provider} returned an empty response')
    try:
        document = json.loads(raw_output)
    except json.JSONDecodeError as error:
        raise PlannerError(
            f'{provider} returned invalid JSON: {error.msg}'
        ) from error

    if not isinstance(document, dict) or not isinstance(
        document.get('plan'), list
    ):
        raise PlannerError(f'{provider} response does not contain a plan list')

    canonical_steps = []
    for raw_step in document['plan']:
        if not isinstance(raw_step, dict):
            raise PlannerError(f'{provider} returned a non-object plan step')
        canonical_steps.append(
            {key: value for key, value in raw_step.items() if value is not None}
        )

    canonical_document = {'plan': canonical_steps}
    for field in ('scene_version', 'goal'):
        if field in document:
            canonical_document[field] = document[field]
    canonical = json.dumps(canonical_document, separators=(',', ':'))
    if scene_state is None:
        parse_and_validate_plan(canonical)
    else:
        validate_state_aware_plan(canonical_document, scene_state)
    return canonical
