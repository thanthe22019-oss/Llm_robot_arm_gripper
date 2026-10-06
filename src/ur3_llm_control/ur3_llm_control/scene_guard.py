"""Runtime checks that a camera scene still matches a validated plan."""

from copy import deepcopy
from typing import Any, Mapping

from ur3_llm_control.task_validator import PlanStep


class SceneChangedError(RuntimeError):
    """Raised before a skill when observed state differs from expected state."""


class SceneGuard:
    """Track expected occupancy while a validated plan is executing."""

    def __init__(self, scene_state: Mapping[str, Any]) -> None:
        self._initial_version = scene_state.get('version')
        self._objects = deepcopy(scene_state.get('objects', {}))
        self._regions = {}
        for field in ('zones', 'temporary_slots'):
            for name, value in scene_state.get(field, {}).items():
                self._regions[name] = deepcopy(value)
        self._held = None
        self._first_check = True

    def check_before(self, step: PlanStep, scene_state: Mapping[str, Any]) -> None:
        """Reject changed source/object/occupancy before a stateful skill."""
        if scene_state.get('source') != 'camera':
            raise SceneChangedError('latest SceneState is not camera-derived')
        if self._first_check:
            if scene_state.get('version') != self._initial_version:
                raise SceneChangedError(
                    f"SceneState changed from v{self._initial_version} to "
                    f"v{scene_state.get('version')} before execution"
                )
            self._first_check = False

        if step.skill in {'pick', 'check_zone'}:
            if scene_state.get('valid') is not True:
                raise SceneChangedError('latest camera SceneState is invalid')
            for field in ('zones', 'temporary_slots'):
                actual_regions = scene_state.get(field, {})
                for name, expected in self._regions.items():
                    if name not in actual_regions:
                        continue
                    actual = actual_regions[name]
                    if actual.get('object') != expected.get('object'):
                        raise SceneChangedError(
                            f"occupancy of '{name}' changed from "
                            f"'{expected.get('object')}' to '{actual.get('object')}'"
                        )
            if step.skill == 'pick':
                actual = scene_state.get('objects', {}).get(step.object_name, {})
                expected_location = self._objects[step.object_name].get('location')
                if (
                    actual.get('visible') is not True
                    or actual.get('location') != expected_location
                ):
                    raise SceneChangedError(
                        f"'{step.object_name}' is no longer at "
                        f"'{expected_location}'"
                    )

        if step.skill in {'place', 'place_temp'}:
            field = 'temporary_slots' if step.skill == 'place_temp' else 'zones'
            actual = scene_state.get(field, {}).get(step.zone_name, {})
            expected = self._regions[step.zone_name]
            if actual.get('object') != expected.get('object'):
                raise SceneChangedError(
                    f"destination '{step.zone_name}' changed occupancy"
                )

    def apply_success(self, step: PlanStep) -> None:
        """Advance the expected state after one successful robot skill."""
        if step.skill == 'pick':
            location = self._objects[step.object_name].get('location')
            if location in self._regions:
                self._regions[location] = {'occupied': False, 'object': None}
            self._objects[step.object_name]['location'] = 'held'
            self._held = step.object_name
        elif step.skill in {'place', 'place_temp'}:
            self._regions[step.zone_name] = {
                'occupied': True,
                'object': step.object_name,
            }
            self._objects[step.object_name]['location'] = step.zone_name
            self._held = None

    @property
    def holding_object(self) -> bool:
        return self._held is not None
