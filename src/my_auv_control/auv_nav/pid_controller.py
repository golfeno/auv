from .models import VehicleState, ActuatorCommands, ControlConfig

class PIDController:
    def __init__(self):
        self.curr_cmd_base = 0.0
        self.curr_rv = 0.0; self.curr_hl = 0.0; self.curr_hr = 0.0

    @staticmethod
    def constrain_slew(current: float, target: float, max_rate: float, dt: float) -> float:
        max_change = max_rate * dt
        return current + max(-max_change, min(max_change, target - current))

    def compute(self, state: VehicleState, phase: str, target_params: dict, dt: float) -> ActuatorCommands:
        vel_abs = max(0.1, abs(state.vel))
        vel_scale = max(0.4, min(1.0, vel_abs / 2.5))
        Kp_z = ControlConfig.Kp_z_base * (0.8 if phase == 'XY_FINAL' else vel_scale)
        Kd_z = ControlConfig.Kd_z_base * (1.4 if phase == 'XY_FINAL' else vel_scale)
        roll_damping = 1.0 - max(0.0, min(0.6, state.roll_abs * 2.0))
        target_rudder_h = -(Kp_z * state.z_err + Kd_z * state.dz_dt) * roll_damping
        pitch_limit = 0.25 if phase == 'XY_FINAL' else 0.45
        if state.pitch_curr > pitch_limit: target_rudder_h = min(target_rudder_h, -0.2)
        elif state.pitch_curr < -pitch_limit: target_rudder_h = max(target_rudder_h, 0.2)
        target_rudder_h = max(-0.55, min(0.55, target_rudder_h))
        target_rudder_v = ControlConfig.Kp_yaw * state.yaw_err + ControlConfig.Kd_yaw * state.yaw_d
        if state.roll_abs > 0.18 and phase == 'XY_FINAL': target_rudder_v *= 0.35
        target_rudder_v = max(-0.5, min(0.5, target_rudder_v))
        roll_pid = ControlConfig.Kp_roll * state.rpy[0] + ControlConfig.Kd_roll * state.pitch_d
        pitch_priority = max(0.15, 1.0 - (abs(state.z_err) / 5.0))
        roll_pid *= pitch_priority
        raw_hl = max(-0.95, min(0.95, target_rudder_h - roll_pid - ControlConfig.roll_bias))
        raw_hr = max(-0.95, min(0.95, target_rudder_h + roll_pid + ControlConfig.roll_bias))
        self.curr_cmd_base = self.constrain_slew(self.curr_cmd_base, target_params['base_speed'], 4.0 if phase != 'HOVER_STAB' else 0.0, dt)
        self.curr_rv = self.constrain_slew(self.curr_rv, target_rudder_v, ControlConfig.max_rudder_speed, dt)
        self.curr_hl = self.constrain_slew(self.curr_hl, raw_hl, ControlConfig.max_rudder_speed, dt)
        self.curr_hr = self.constrain_slew(self.curr_hr, raw_hr, ControlConfig.max_rudder_speed, dt)
        cmd = ActuatorCommands()
        if phase == 'HOVER_STAB': cmd.lt = 0.0; cmd.rt = 0.0
        else: cmd.lt = self.curr_cmd_base + target_params['yaw_diff']; cmd.rt = self.curr_cmd_base - target_params['yaw_diff']
        cmd.rv, cmd.hl, cmd.hr = self.curr_rv, self.curr_hl, self.curr_hr
        return cmd
