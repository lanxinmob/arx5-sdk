from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Sequence

import numpy as np
import numpy.typing as npt


FloatArray = npt.NDArray[np.float64]
_NUMERIC_TOLERANCE = 1e-12


class ActionGuardError(ValueError):
    """Raised when an action cannot be accepted by the safety guard."""


@dataclass(frozen=True)
class ValidatedAction:
    ee_pose: FloatArray
    gripper_pos: float
    generation: int


def wrapped_rpy_delta(target_rpy: Any, reference_rpy: Any) -> FloatArray:
    """Return the component-wise shortest signed RPY delta in [-pi, pi)."""
    target = _finite_vector(target_rpy, 3, "target_rpy")
    reference = _finite_vector(reference_rpy, 3, "reference_rpy")
    return (target - reference + np.pi) % (2.0 * np.pi) - np.pi


def _finite_vector(value: Any, size: int, name: str) -> FloatArray:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ActionGuardError(f"{name} must be numeric") from exc
    if array.shape != (size,):
        raise ActionGuardError(f"{name} must have shape ({size},), got {array.shape}")
    if not np.all(np.isfinite(array)):
        raise ActionGuardError(f"{name} must contain only finite values")
    return np.array(array, dtype=np.float64, copy=True)


def _finite_scalar(value: Any, name: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise ActionGuardError(f"{name} must be a finite scalar")
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ActionGuardError(f"{name} must be a finite scalar") from exc
    if array.shape != () or not np.isfinite(array.item()):
        raise ActionGuardError(f"{name} must be a finite scalar")
    return float(array.item())


def _nonnegative_limit(value: Any, name: str) -> float:
    result = _finite_scalar(value, name)
    if result < 0.0:
        raise ActionGuardError(f"{name} must be nonnegative")
    return result


class AgentActionGuard:
    """Stateful validator for absolute ARX5 end-effector commands.

    Validation and acceptance are deliberately separate. A caller should first
    call ``validate_payload``, submit the returned values to the robot SDK, and
    call ``accept`` only after the SDK call succeeds.
    """

    def __init__(
        self,
        home_pose: Any,
        *,
        max_position_delta: float = 0.002,
        max_rpy_delta: float = 0.02,
        gripper_min: float = 0.0,
        gripper_max: float = 0.086,
        max_gripper_delta: float = 0.002,
        max_home_translation_radius: float = 0.45,
        workspace_min: Optional[Sequence[float]] = None,
        workspace_max: Optional[Sequence[float]] = None,
    ) -> None:
        self._home_pose = _finite_vector(home_pose, 6, "home_pose")
        self.max_position_delta = _nonnegative_limit(
            max_position_delta, "max_position_delta"
        )
        self.max_rpy_delta = _nonnegative_limit(max_rpy_delta, "max_rpy_delta")
        self.gripper_min = _finite_scalar(gripper_min, "gripper_min")
        self.gripper_max = _finite_scalar(gripper_max, "gripper_max")
        if self.gripper_min > self.gripper_max:
            raise ActionGuardError("gripper_min must not exceed gripper_max")
        self.max_gripper_delta = _nonnegative_limit(
            max_gripper_delta, "max_gripper_delta"
        )
        self.max_home_translation_radius = _nonnegative_limit(
            max_home_translation_radius, "max_home_translation_radius"
        )
        self.workspace_min = (
            None
            if workspace_min is None
            else _finite_vector(workspace_min, 3, "workspace_min")
        )
        self.workspace_max = (
            None
            if workspace_max is None
            else _finite_vector(workspace_max, 3, "workspace_max")
        )
        if (
            self.workspace_min is not None
            and self.workspace_max is not None
            and np.any(self.workspace_min > self.workspace_max)
        ):
            raise ActionGuardError(
                "workspace_min must not exceed workspace_max on any axis"
            )
        if self.workspace_min is not None:
            self.workspace_min.setflags(write=False)
        if self.workspace_max is not None:
            self.workspace_max.setflags(write=False)

        self._reference_pose: Optional[FloatArray] = None
        self._reference_gripper: Optional[float] = None
        self._generation = 0

    @property
    def armed(self) -> bool:
        return self._reference_pose is not None

    @property
    def home_pose(self) -> FloatArray:
        return self._home_pose.copy()

    @property
    def last_accepted_pose(self) -> Optional[FloatArray]:
        if self._reference_pose is None:
            return None
        return self._reference_pose.copy()

    @property
    def last_accepted_gripper(self) -> Optional[float]:
        return self._reference_gripper

    def arm(self, reference_pose: Any, reference_gripper: Any) -> None:
        """Arm after RESET_TO_HOME using the expected reset command as reference."""
        pose = _finite_vector(reference_pose, 6, "reference_pose")
        gripper = _finite_scalar(reference_gripper, "reference_gripper")
        self._reference_pose = pose
        self._reference_gripper = gripper
        self._generation += 1

    def disarm(self) -> None:
        self._reference_pose = None
        self._reference_gripper = None
        self._generation += 1

    def validate_payload(self, payload: Any) -> ValidatedAction:
        if not self.armed:
            raise ActionGuardError("RESET_TO_HOME is required before SET_EE_POSE")
        if not isinstance(payload, dict):
            raise ActionGuardError("SET_EE_POSE data must be a dict")

        expected_keys = {"ee_pose", "gripper_pos"}
        actual_keys = set(payload.keys())
        missing = expected_keys - actual_keys
        unexpected = actual_keys - expected_keys
        if missing:
            raise ActionGuardError(f"missing action fields: {sorted(missing)}")
        if unexpected:
            raise ActionGuardError(f"unexpected action fields: {sorted(unexpected)}")

        pose = _finite_vector(payload["ee_pose"], 6, "ee_pose")
        gripper_value = payload["gripper_pos"]
        if gripper_value is None:
            assert self._reference_gripper is not None
            gripper = self._reference_gripper
        else:
            gripper = _finite_scalar(gripper_value, "gripper_pos")

        assert self._reference_pose is not None
        assert self._reference_gripper is not None

        position_delta = float(np.linalg.norm(pose[:3] - self._reference_pose[:3]))
        if position_delta > self.max_position_delta + _NUMERIC_TOLERANCE:
            raise ActionGuardError(
                f"position delta {position_delta:.6f} m exceeds "
                f"{self.max_position_delta:.6f} m"
            )

        rpy_delta = wrapped_rpy_delta(pose[3:], self._reference_pose[3:])
        rpy_delta_norm = float(np.linalg.norm(rpy_delta))
        if rpy_delta_norm > self.max_rpy_delta + _NUMERIC_TOLERANCE:
            raise ActionGuardError(
                f"wrapped RPY delta {rpy_delta_norm:.6f} rad exceeds "
                f"{self.max_rpy_delta:.6f} rad"
            )
        # Preserve the same physical Euler rotation without forwarding an
        # arbitrarily large, numerically fragile 2*pi-equivalent angle.
        pose[3:] = self._reference_pose[3:] + rpy_delta

        if not (
            self.gripper_min - _NUMERIC_TOLERANCE
            <= gripper
            <= self.gripper_max + _NUMERIC_TOLERANCE
        ):
            raise ActionGuardError(
                f"gripper_pos {gripper:.6f} m is outside "
                f"[{self.gripper_min:.6f}, {self.gripper_max:.6f}] m"
            )
        gripper_delta = abs(gripper - self._reference_gripper)
        if gripper_delta > self.max_gripper_delta + _NUMERIC_TOLERANCE:
            raise ActionGuardError(
                f"gripper delta {gripper_delta:.6f} m exceeds "
                f"{self.max_gripper_delta:.6f} m"
            )

        home_translation = float(np.linalg.norm(pose[:3] - self._home_pose[:3]))
        if (
            home_translation
            > self.max_home_translation_radius + _NUMERIC_TOLERANCE
        ):
            raise ActionGuardError(
                f"home translation radius {home_translation:.6f} m exceeds "
                f"{self.max_home_translation_radius:.6f} m"
            )

        if self.workspace_min is not None and np.any(
            pose[:3] < self.workspace_min - _NUMERIC_TOLERANCE
        ):
            raise ActionGuardError(
                f"position {pose[:3].tolist()} is below workspace_min "
                f"{self.workspace_min.tolist()}"
            )
        if self.workspace_max is not None and np.any(
            pose[:3] > self.workspace_max + _NUMERIC_TOLERANCE
        ):
            raise ActionGuardError(
                f"position {pose[:3].tolist()} is above workspace_max "
                f"{self.workspace_max.tolist()}"
            )

        pose.setflags(write=False)
        return ValidatedAction(
            ee_pose=pose,
            gripper_pos=gripper,
            generation=self._generation,
        )

    def accept(self, action: ValidatedAction) -> None:
        if not isinstance(action, ValidatedAction):
            raise ActionGuardError("accept requires a ValidatedAction")
        if not self.armed or action.generation != self._generation:
            raise ActionGuardError("validated action is stale")
        self._reference_pose = np.array(action.ee_pose, dtype=np.float64, copy=True)
        self._reference_gripper = float(action.gripper_pos)
        self._generation += 1
