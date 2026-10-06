"""Validate LLM-style JSON plans before any robot skill can run."""

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple


VALID_SKILLS = frozenset({
    'observe_scene',
    'check_zone',
    'pick',
    'place',
    'place_temp',
    'home',
})
VALID_OBJECTS = frozenset({
    'red_cube',
    'yellow_cube',
    'blue_cube',
    'green_cube',
    'purple_cube',
})
VALID_ZONES = frozenset({'zone_a', 'zone_b', 'zone_c'})
VALID_TEMPORARY_SLOTS = frozenset({'temp_1'})
MINIMUM_PERCEPTION_CONFIDENCE = 0.70
FORBIDDEN_CONTROL_TOKENS = (
    'joint',
    'trajectory',
    'velocity',
    'torque',
    'effort',
    'controller',
    'command',
)


class PlanValidationError(ValueError):
    """A stable validation code plus a human-readable explanation."""

    def __init__(
        self,
        code: str,
        message: str,
        step_index: Optional[int] = None,
    ) -> None:
        self.code = code
        self.message = message
        self.step_index = step_index
        location = '' if step_index is None else f'step {step_index}: '
        super().__init__(f'{code}: {location}{message}')


@dataclass(frozen=True)
class PlanStep:
    """Canonical skill call accepted by the validator."""

    skill: str
    object_name: Optional[str] = None
    zone_name: Optional[str] = None

    def as_dict(self) -> Dict[str, str]:
        """Return the JSON-compatible representation used by the executor."""
        result = {'skill': self.skill}
        if self.object_name is not None:
            result['object'] = self.object_name
        if self.zone_name is not None:
            result['slot' if self.skill == 'place_temp' else 'zone'] = (
                self.zone_name
            )
        return result

    def label(self) -> str:
        """Format a concise terminal representation."""
        arguments = [
            value
            for value in (self.object_name, self.zone_name)
            if value is not None
        ]
        return f"{self.skill}({', '.join(arguments)})"


def _normalise_key(key: str) -> str:
    return key.lower().replace('-', '_').replace(' ', '_')


def _reject_forbidden_controls(value: Any, path: str = '$') -> None:
    if isinstance(value, dict):
        for key, nested in value.items():
            normalised = _normalise_key(str(key))
            if any(token in normalised for token in FORBIDDEN_CONTROL_TOKENS):
                raise PlanValidationError(
                    'FORBIDDEN_CONTROL',
                    f"field '{path}.{key}' may not control robot motion directly",
                )
            _reject_forbidden_controls(nested, f'{path}.{key}')
    elif isinstance(value, list):
        for index, nested in enumerate(value):
            _reject_forbidden_controls(nested, f'{path}[{index}]')


def _require_string(
    step: Dict[str, Any],
    field: str,
    step_index: int,
) -> str:
    value = step[field]
    if not isinstance(value, str) or not value:
        raise PlanValidationError(
            'INVALID_PARAMETER',
            f"'{field}' must be a non-empty string",
            step_index,
        )
    return value


def _validate_step(step: Any, step_index: int) -> PlanStep:
    if not isinstance(step, dict):
        raise PlanValidationError(
            'INVALID_STEP',
            'each plan step must be an object',
            step_index,
        )
    if 'skill' not in step:
        raise PlanValidationError(
            'MISSING_FIELD',
            "required field 'skill' is missing",
            step_index,
        )

    skill = _require_string(step, 'skill', step_index)
    if skill not in VALID_SKILLS:
        raise PlanValidationError(
            'INVALID_SKILL',
            f"unsupported skill '{skill}'",
            step_index,
        )

    expected_fields = {
        'home': {'skill'},
        'observe_scene': {'skill'},
        'check_zone': {'skill', 'zone'},
        'pick': {'skill', 'object'},
        'place': {'skill', 'object', 'zone'},
        'place_temp': {'skill', 'object', 'slot'},
    }[skill]
    missing = expected_fields - set(step)
    if missing:
        field = sorted(missing)[0]
        raise PlanValidationError(
            'MISSING_FIELD',
            f"required field '{field}' is missing for {skill}",
            step_index,
        )
    unexpected = set(step) - expected_fields
    if unexpected:
        fields = ', '.join(sorted(unexpected))
        raise PlanValidationError(
            'UNEXPECTED_FIELD',
            f'unexpected field(s) for {skill}: {fields}',
            step_index,
        )

    if skill in {'home', 'observe_scene'}:
        return PlanStep(skill=skill)

    if skill == 'check_zone':
        zone_name = _require_string(step, 'zone', step_index)
        if zone_name not in VALID_ZONES | VALID_TEMPORARY_SLOTS:
            raise PlanValidationError(
                'INVALID_ZONE',
                f"unknown zone or temporary slot '{zone_name}'",
                step_index,
            )
        return PlanStep(skill='check_zone', zone_name=zone_name)

    object_name = _require_string(step, 'object', step_index)
    if object_name not in VALID_OBJECTS:
        raise PlanValidationError(
            'INVALID_OBJECT',
            f"unknown object '{object_name}'",
            step_index,
        )
    if skill == 'pick':
        return PlanStep(skill='pick', object_name=object_name)

    destination_field = 'slot' if skill == 'place_temp' else 'zone'
    zone_name = _require_string(step, destination_field, step_index)
    valid_destinations = (
        VALID_TEMPORARY_SLOTS if skill == 'place_temp' else VALID_ZONES
    )
    if zone_name not in valid_destinations:
        raise PlanValidationError(
            'INVALID_ZONE',
            f"unknown {destination_field} '{zone_name}'",
            step_index,
        )
    return PlanStep(
        skill=skill,
        object_name=object_name,
        zone_name=zone_name,
    )


def _validate_sequence(steps: Sequence[PlanStep]) -> None:
    held_object = None
    home_required = False
    for index, step in enumerate(steps):
        if home_required and step.skill != 'home':
            raise PlanValidationError(
                'INVALID_SEQUENCE',
                'home is required after place before the next task',
                index,
            )

        if step.skill == 'pick':
            if held_object is not None:
                raise PlanValidationError(
                    'INVALID_SEQUENCE',
                    f"cannot pick '{step.object_name}' while holding "
                    f"'{held_object}'",
                    index,
                )
            held_object = step.object_name
        elif step.skill in {'place', 'place_temp'}:
            if held_object is None:
                raise PlanValidationError(
                    'INVALID_SEQUENCE',
                    'place requires a previously picked object',
                    index,
                )
            if step.object_name != held_object:
                raise PlanValidationError(
                    'INVALID_SEQUENCE',
                    f"cannot place '{step.object_name}' while holding "
                    f"'{held_object}'",
                    index,
                )
            held_object = None
            home_required = True
        elif held_object is not None:
            raise PlanValidationError(
                'INVALID_SEQUENCE',
                f"{step.skill} is unsafe while holding '{held_object}'",
                index,
            )
        elif step.skill == 'home':
            home_required = False

    if held_object is not None:
        raise PlanValidationError(
            'INVALID_SEQUENCE',
            f"plan ends while holding '{held_object}'",
            len(steps) - 1,
        )
    if home_required:
        raise PlanValidationError(
            'INVALID_SEQUENCE',
            'plan must return home after the final place',
            len(steps) - 1,
        )


def validate_plan_document(document: Any) -> Tuple[PlanStep, ...]:
    """Validate a decoded JSON document and return canonical immutable steps."""
    _reject_forbidden_controls(document)
    if not isinstance(document, dict):
        raise PlanValidationError(
            'INVALID_ROOT',
            'the JSON root must be an object',
        )
    if 'plan' not in document:
        raise PlanValidationError(
            'MISSING_PLAN',
            "top-level field 'plan' is required",
        )
    unexpected = set(document) - {'plan', 'scene_version', 'goal'}
    if unexpected:
        fields = ', '.join(sorted(unexpected))
        raise PlanValidationError(
            'UNEXPECTED_FIELD',
            f'unexpected top-level field(s): {fields}',
        )

    if 'scene_version' in document and (
        not isinstance(document['scene_version'], int)
        or isinstance(document['scene_version'], bool)
        or document['scene_version'] < 1
    ):
        raise PlanValidationError(
            'INVALID_SCENE_VERSION',
            "'scene_version' must be a positive integer",
        )
    if 'goal' in document:
        goal = document['goal']
        if not isinstance(goal, dict) or set(goal) != {'object', 'destination'}:
            raise PlanValidationError(
                'INVALID_GOAL',
                "'goal' must contain exactly 'object' and 'destination'",
            )
        if goal['object'] not in VALID_OBJECTS:
            raise PlanValidationError(
                'INVALID_GOAL',
                f"unknown goal object '{goal['object']}'",
            )
        if goal['destination'] not in VALID_ZONES:
            raise PlanValidationError(
                'INVALID_GOAL',
                f"unknown goal destination '{goal['destination']}'",
            )

    raw_plan = document['plan']
    if not isinstance(raw_plan, list):
        raise PlanValidationError(
            'INVALID_PLAN',
            "'plan' must be a list",
        )
    if not raw_plan:
        raise PlanValidationError(
            'EMPTY_PLAN',
            "'plan' must contain at least one step",
        )

    steps = tuple(
        _validate_step(step, index)
        for index, step in enumerate(raw_plan)
    )
    _validate_sequence(steps)
    return steps


def _require_scene_mapping(
    scene_state: Mapping[str, Any], field: str
) -> Mapping[str, Any]:
    value = scene_state.get(field)
    if not isinstance(value, dict):
        raise PlanValidationError(
            'INVALID_SCENE', f"camera SceneState field '{field}' must be an object"
        )
    return value


def _validated_scene_state(
    scene_state: Any,
) -> Tuple[int, Mapping[str, Any], Dict[str, Optional[str]]]:
    """Validate the camera snapshot and return objects plus occupancy."""
    if not isinstance(scene_state, dict):
        raise PlanValidationError(
            'INVALID_SCENE', 'camera SceneState must be a JSON object'
        )
    if scene_state.get('source') != 'camera':
        raise PlanValidationError(
            'INVALID_SCENE', "SceneState source must be 'camera'"
        )
    if scene_state.get('fresh') is False:
        raise PlanValidationError(
            'STALE_SCENE', 'camera SceneState is marked stale'
        )
    if scene_state.get('valid') is not True:
        raise PlanValidationError(
            'INVALID_SCENE', 'camera SceneState is not valid'
        )
    version = scene_state.get('version')
    if (
        not isinstance(version, int)
        or isinstance(version, bool)
        or version < 1
    ):
        raise PlanValidationError(
            'INVALID_SCENE', 'camera SceneState version must be positive'
        )

    objects = _require_scene_mapping(scene_state, 'objects')
    missing_objects = VALID_OBJECTS - set(objects)
    if missing_objects:
        raise PlanValidationError(
            'INVALID_SCENE',
            'camera SceneState is missing: ' + ', '.join(sorted(missing_objects)),
        )

    occupancy: Dict[str, Optional[str]] = {}
    for field, valid_names in (
        ('zones', VALID_ZONES),
        ('temporary_slots', VALID_TEMPORARY_SLOTS),
    ):
        regions = _require_scene_mapping(scene_state, field)
        missing_regions = valid_names - set(regions)
        if missing_regions:
            raise PlanValidationError(
                'INVALID_SCENE',
                f"camera SceneState {field} is missing: "
                + ', '.join(sorted(missing_regions)),
            )
        for name in valid_names:
            region = regions[name]
            if not isinstance(region, dict):
                raise PlanValidationError(
                    'INVALID_SCENE', f"SceneState region '{name}' is invalid"
                )
            occupied = region.get('occupied')
            occupant = region.get('object')
            if not isinstance(occupied, bool):
                raise PlanValidationError(
                    'INVALID_SCENE',
                    f"SceneState region '{name}' has no occupancy boolean",
                )
            if occupied:
                if occupant not in VALID_OBJECTS:
                    raise PlanValidationError(
                        'INVALID_SCENE',
                        f"SceneState region '{name}' has an invalid occupant",
                    )
                occupancy[name] = occupant
            elif occupant is not None:
                raise PlanValidationError(
                    'INVALID_SCENE',
                    f"empty SceneState region '{name}' names an occupant",
                )
            else:
                occupancy[name] = None
    return version, objects, occupancy


def validate_state_aware_plan(
    document: Any,
    scene_state: Any,
    *,
    minimum_confidence: float = MINIMUM_PERCEPTION_CONFIDENCE,
) -> Tuple[PlanStep, ...]:
    """Validate and simulate a plan against one immutable camera snapshot."""
    steps = validate_plan_document(document)
    if not isinstance(document, dict) or 'scene_version' not in document:
        raise PlanValidationError(
            'MISSING_SCENE_VERSION',
            "state-aware plans require top-level 'scene_version'",
        )
    if 'goal' not in document:
        raise PlanValidationError(
            'MISSING_GOAL', "state-aware plans require top-level 'goal'"
        )

    version, objects, occupancy = _validated_scene_state(scene_state)
    if document['scene_version'] != version:
        raise PlanValidationError(
            'STALE_SCENE',
            f"plan uses SceneState v{document['scene_version']} but camera is v{version}",
        )

    goal_object = document['goal']['object']
    goal_destination = document['goal']['destination']
    goal_occupant = occupancy[goal_destination]
    if (
        goal_occupant not in (None, goal_object)
        and all(occupancy[slot] is not None for slot in VALID_TEMPORARY_SLOTS)
    ):
        raise PlanValidationError(
            'NO_FREE_TEMP_SLOT',
            f"'{goal_destination}' is occupied by '{goal_occupant}' and no "
            'temporary slot is free',
        )

    locations = {}
    for object_name in VALID_OBJECTS:
        observed = objects[object_name]
        if not isinstance(observed, dict):
            raise PlanValidationError(
                'INVALID_SCENE',
                f"camera state for '{object_name}' must be an object",
            )
        locations[object_name] = observed.get('location', 'unknown')

    held_object: Optional[str] = None
    checked_destinations = set()
    for index, step in enumerate(steps):
        if step.skill == 'observe_scene':
            continue
        if step.skill == 'check_zone':
            checked_destinations.add(step.zone_name)
            continue
        if step.skill == 'pick':
            observed = objects[step.object_name]
            if observed.get('visible') is not True:
                raise PlanValidationError(
                    'OBJECT_NOT_VISIBLE',
                    f"camera cannot see '{step.object_name}'",
                    index,
                )
            confidence = observed.get('confidence')
            if not isinstance(confidence, (int, float)) or confidence < minimum_confidence:
                raise PlanValidationError(
                    'LOW_CONFIDENCE',
                    f"camera confidence for '{step.object_name}' is below "
                    f'{minimum_confidence:.2f}',
                    index,
                )
            held_object = step.object_name
            previous = locations[step.object_name]
            if previous in occupancy and occupancy[previous] == step.object_name:
                occupancy[previous] = None
            locations[step.object_name] = 'held'
            continue
        if step.skill in {'place', 'place_temp'}:
            destination = step.zone_name
            occupant = occupancy[destination]
            if occupant is not None:
                raise PlanValidationError(
                    'OCCUPIED_DESTINATION',
                    f"cannot place '{step.object_name}' in '{destination}'; it is "
                    f"occupied by '{occupant}'",
                    index,
                )
            if destination == goal_destination and destination not in checked_destinations:
                raise PlanValidationError(
                    'MISSING_ZONE_CHECK',
                    f"'{goal_destination}' must be checked before placement",
                    index,
                )
            occupancy[destination] = held_object
            locations[held_object] = destination
            held_object = None

    if locations.get(goal_object) != goal_destination:
        raise PlanValidationError(
            'GOAL_NOT_REACHED',
            f"plan leaves '{goal_object}' in '{locations.get(goal_object)}', not "
            f"'{goal_destination}'",
        )
    if not steps or steps[-1].skill != 'home':
        raise PlanValidationError(
            'INVALID_SEQUENCE', 'state-aware plan must end with home'
        )
    return steps


def parse_and_validate_plan(raw_json: str) -> Tuple[PlanStep, ...]:
    """Parse raw JSON, reject malformed input, then validate the whole plan."""
    if not isinstance(raw_json, str):
        raise PlanValidationError(
            'INVALID_JSON',
            'plan input must be a JSON string',
        )
    try:
        document = json.loads(raw_json)
    except json.JSONDecodeError as error:
        raise PlanValidationError(
            'INVALID_JSON',
            f'{error.msg} at line {error.lineno}, column {error.colno}',
        ) from error
    return validate_plan_document(document)


def main() -> int:
    """Validate JSON from the command line without contacting the robot."""
    parser = argparse.ArgumentParser(
        description='Validate a UR3 skill plan without executing it.'
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--json', help='raw JSON plan')
    source.add_argument('--file', type=Path, help='path to a JSON plan')
    parser.add_argument(
        '--scene-state',
        type=Path,
        help='camera SceneState JSON file for state-aware validation',
    )
    arguments = parser.parse_args()

    raw_json = (
        arguments.file.read_text(encoding='utf-8')
        if arguments.file is not None
        else arguments.json
    )
    try:
        if arguments.scene_state is None:
            plan = parse_and_validate_plan(raw_json)
        else:
            try:
                document = json.loads(raw_json)
                scene_state = json.loads(
                    arguments.scene_state.read_text(encoding='utf-8')
                )
            except json.JSONDecodeError as error:
                raise PlanValidationError(
                    'INVALID_JSON',
                    f'{error.msg} at line {error.lineno}, column {error.colno}',
                ) from error
            plan = validate_state_aware_plan(document, scene_state)
    except (OSError, PlanValidationError) as error:
        print('VALIDATION: REJECTED')
        print(error)
        return 2

    print('VALIDATION: SUCCESS')
    for index, step in enumerate(plan, start=1):
        print(f'{index}. {step.label()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
