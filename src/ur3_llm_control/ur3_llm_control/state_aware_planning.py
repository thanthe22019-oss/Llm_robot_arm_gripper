"""Deterministic scene-aware planning used by tests and the offline planner."""

from typing import Any, Dict

from ur3_llm_control.task_validator import (
    PlanValidationError,
    VALID_OBJECTS,
    VALID_TEMPORARY_SLOTS,
    VALID_ZONES,
    validate_state_aware_plan,
)


def build_state_aware_plan(
    scene_state: Dict[str, Any], object_name: str, destination: str
) -> Dict[str, Any]:
    """Build a safe skill plan from current camera occupancy.

    This function never creates Cartesian poses or joint commands.  It only
    orders the closed robot-skill vocabulary and validates its own output.
    """
    if object_name not in VALID_OBJECTS:
        raise PlanValidationError(
            'INVALID_OBJECT', f"unknown object '{object_name}'"
        )
    if destination not in VALID_ZONES:
        raise PlanValidationError(
            'INVALID_ZONE', f"unknown zone '{destination}'"
        )
    if not isinstance(scene_state, dict):
        raise PlanValidationError(
            'INVALID_SCENE', 'camera SceneState must be a JSON object'
        )
    version = scene_state.get('version')
    zones = scene_state.get('zones')
    slots = scene_state.get('temporary_slots')
    objects = scene_state.get('objects')
    if not isinstance(zones, dict) or destination not in zones:
        raise PlanValidationError(
            'INVALID_SCENE', f"camera SceneState has no '{destination}'"
        )
    if not isinstance(slots, dict) or not isinstance(objects, dict):
        raise PlanValidationError(
            'INVALID_SCENE', 'camera SceneState lacks objects or temporary slots'
        )

    steps = [{'skill': 'check_zone', 'zone': destination}]
    if (
        isinstance(objects.get(object_name), dict)
        and objects[object_name].get('location') == destination
    ):
        steps.append({'skill': 'home'})
    else:
        occupant = zones[destination].get('object')
        if occupant is not None and occupant != object_name:
            free_slots = sorted(
                slot
                for slot in VALID_TEMPORARY_SLOTS
                if isinstance(slots.get(slot), dict)
                and slots[slot].get('occupied') is False
            )
            if not free_slots:
                raise PlanValidationError(
                    'NO_FREE_TEMP_SLOT',
                    f"'{destination}' is occupied by '{occupant}' and no "
                    'temporary slot is free',
                )
            slot = free_slots[0]
            steps.extend([
                {'skill': 'pick', 'object': occupant},
                {'skill': 'place_temp', 'object': occupant, 'slot': slot},
                {'skill': 'home'},
            ])
        steps.extend([
            {'skill': 'pick', 'object': object_name},
            {
                'skill': 'place',
                'object': object_name,
                'zone': destination,
            },
            {'skill': 'home'},
        ])

    document = {
        'scene_version': version,
        'goal': {'object': object_name, 'destination': destination},
        'plan': steps,
    }
    validate_state_aware_plan(document, scene_state)
    return document
