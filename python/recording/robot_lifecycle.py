"""Recorder-specific gravity compensation, slow engagement and homing.

The teleop worker must be stopped before calling any transition method.
"""

import time

import numpy as np

from examples.leader_follower_teleop import LeaderFollowerTeleop, arx5


def smoothstep(u):
    u = float(np.clip(u, 0, 1))
    return u ** 3 * (10 + u * (-15 + 6 * u))


class RecordingTeleop(LeaderFollowerTeleop):
    home_seconds = 3.0
    home_speed = 1.5  # rad/s maximum planned joint speed
    home_gripper_speed = 0.1  # m/s
    alignment_speed = 0.35
    alignment_gripper_speed = 0.025
    follower_idle_kd_scale = 0.5
    transition_seconds = 0.5

    def __init__(self, *args, **kwargs):
        try:
            super().__init__(*args, **kwargs)
        except Exception:
            # A follower connection failure can leave a fully constructed leader.
            if getattr(self, "leader", None) is not None:
                self.controller_dt = self.leader_controller_config.controller_dt
                try:
                    self.slow_home()
                except Exception as recovery_error:
                    print(f"初始化失败后的回零也未完成：{recovery_error}", flush=True)
            raise

    def _command(self, controller, position, gripper):
        position = np.asarray(position)
        if position.shape != (self.dof,) or not np.all(np.isfinite(position)) or not np.isfinite(gripper):
            raise RuntimeError("Invalid joint/gripper target; refusing to send motion")
        cmd = arx5.JointState(self.dof)
        cmd.pos()[:] = position
        cmd.gripper_pos = float(gripper)
        # An immediate command also cancels stale interpolation/force feedback.
        cmd.timestamp = controller.get_timestamp()
        controller.set_joint_cmd(cmd)

    def _default_gain(self, config):
        return arx5.Gain(config.default_kp, config.default_kd,
                         config.default_gripper_kp, config.default_gripper_kd)

    def enter_idle(self):
        self.teleop_started = False
        self.command_enabled = True
        for controller, config in ((self.leader, self.leader_controller_config),
                                   (self.follower, self.follower_controller_config)):
            state = controller.get_joint_state()
            self._command(controller, state.pos(), state.gripper_pos)
            gain = controller.get_gain()
            gain.kp()[:] = 0
            kd_scale = self.follower_idle_kd_scale if controller is self.follower else self.drag_kd_scale
            gain.kd()[:] = config.default_kd * kd_scale
            gain.gripper_kp = 0
            gain.gripper_kd = self.drag_gripper_kd
            controller.set_gain(gain)

    def _ramp_position_gain(self, controller, config):
        state = controller.get_joint_state()
        position = state.pos().copy()
        gripper = state.gripper_pos
        initial = controller.get_gain()
        target = self._default_gain(config)
        start = time.monotonic()
        while True:
            alpha = smoothstep((time.monotonic() - start) / self.transition_seconds)
            self._command(controller, position, gripper)
            controller.set_gain(initial * (1 - alpha) + target * alpha)
            if alpha >= 1:
                break
            time.sleep(self.controller_dt)

    def begin_episode(self, check=lambda: None):
        """Engage follower at its current pose, then slew toward the dragged leader."""
        self._setup_leader_drag_mode()
        self._ramp_position_gain(self.follower, self.follower_controller_config)
        state = self.follower.get_joint_state()
        position, gripper = state.pos().copy(), state.gripper_pos
        started = previous = time.monotonic()
        while True:
            check()
            now = time.monotonic()
            dt = min(now - previous, 0.02)
            previous = now
            leader = self.leader.get_joint_state()
            target = np.clip(leader.pos(), self.follower_pos_min, self.follower_pos_max)
            target_gripper = self._follower_gripper_target(leader.gripper_pos)
            position += np.clip(target - position, -self.alignment_speed * dt, self.alignment_speed * dt)
            gripper += float(np.clip(target_gripper - gripper,
                                    -self.alignment_gripper_speed * dt, self.alignment_gripper_speed * dt))
            self._command(self.follower, position, gripper)
            self._apply_gripper_force_feedback(leader, 0.0)
            actual = self.follower.get_joint_state()
            if (np.max(np.abs(target - position)) < 0.005 and abs(target_gripper - gripper) < 0.001
                    and np.max(np.abs(actual.pos() - target)) < 0.05
                    and abs(actual.gripper_pos - target_gripper) < 0.005):
                break
            if now - started > 30:
                raise RuntimeError("Follower alignment timed out; hold the leader still before starting")
            time.sleep(self.controller_dt)

    def slow_home(self):
        """Drive both arms on one clock; an arm fault must not skip the other arm."""
        self.teleop_started = False
        self.command_enabled = False
        errors = []
        arms = {}
        approach = np.zeros(self.dof)
        approach[2] = 0.03  # SDK's near-home approach waypoint.
        gain_seconds, final_seconds = 0.3, 0.35
        move_seconds = max(0.1, self.home_seconds - gain_seconds - final_seconds)
        for name in ("follower", "leader"):
            controller = getattr(self, name, None)
            if controller is None:
                continue
            try:
                state = controller.get_joint_state()
                if not np.all(np.isfinite(state.pos())) or not np.isfinite(state.gripper_pos):
                    raise RuntimeError("Invalid robot state; cannot plan home trajectory")
                config = getattr(self, f"{name}_controller_config")
                robot = controller.get_robot_config()
                joint_speed = np.minimum(self.home_speed, robot.joint_vel_max)
                grip_speed = min(self.home_gripper_speed, robot.gripper_vel_max)
                move_seconds = max(move_seconds, float(np.max(1.875 * np.abs(approach - state.pos()) / joint_speed)),
                                   1.875 * abs(robot.gripper_width - state.gripper_pos) / grip_speed)
                final_seconds = max(final_seconds, float(np.max(1.875 * np.abs(approach) / joint_speed)))
                arms[name] = dict(controller=controller, position=state.pos().copy(), gripper=state.gripper_pos,
                                  target_gripper=robot.gripper_width, initial_gain=controller.get_gain(),
                                  target_gain=self._default_gain(config))
                self._command(controller, state.pos(), state.gripper_pos)
            except Exception as exc:
                errors.append(f"{name}: {exc}")
                arms.pop(name, None)
        total = gain_seconds + move_seconds + final_seconds
        print(f"双臂同时回 home：规划 {total:.2f} 秒", flush=True)
        started = time.monotonic()
        while arms:
            elapsed = time.monotonic() - started
            for name, arm in list(arms.items()):
                try:
                    if elapsed < gain_seconds:
                        alpha = smoothstep(elapsed / gain_seconds)
                        self._command(arm['controller'], arm['position'], arm['gripper'])
                        arm['controller'].set_gain(arm['initial_gain'] * (1 - alpha) + arm['target_gain'] * alpha)
                    else:
                        if not arm.get('gain_ready'):
                            arm['controller'].set_gain(arm['target_gain'])
                            arm['gain_ready'] = True
                        if elapsed < gain_seconds + move_seconds:
                            alpha = smoothstep((elapsed - gain_seconds) / move_seconds)
                            position = arm['position'] + alpha * (approach - arm['position'])
                            gripper = arm['gripper'] + alpha * (arm['target_gripper'] - arm['gripper'])
                        else:
                            alpha = smoothstep((elapsed - gain_seconds - move_seconds) / final_seconds)
                            position = approach * (1 - alpha)
                            gripper = arm['target_gripper']
                        self._command(arm['controller'], position, gripper)
                    if elapsed >= total:
                        actual = arm['controller'].get_joint_state()
                        if (np.max(np.abs(actual.pos())) < 0.08
                                and abs(actual.gripper_pos - arm['target_gripper']) < 0.01):
                            del arms[name]
                        elif elapsed > total + 3:
                            raise RuntimeError("Home target was sent but measured position did not reach home")
                except Exception as exc:
                    errors.append(f"{name}: {exc}")
                    del arms[name]
            if arms:
                time.sleep(self.controller_dt)
        if errors:
            raise RuntimeError("回零未完成，请检查机械臂状态：" + "; ".join(errors))
        print("两臂已到达 home。", flush=True)
