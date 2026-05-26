from typing import List, Tuple, Dict
from .models import VehicleState, ControlConfig

class PhaseManager:
    def __init__(self, waypoints: List[Tuple[float, float, float]]):
        self.waypoints = waypoints
        self.current_wp_idx = 0
        self.state = 'INIT'
        self.is_re_stabilizing = False
        self.in_back_off_maneuver = False
        self.reset_performed_for_current_wp = False
        self.xy_final_start_time = 0.0
        self.back_off_start_time = 0.0
        self.target = list(waypoints[0])

    def init_waypoint(self, clock_now: float, current_pos: List[float]):
        self.target = list(self.waypoints[self.current_wp_idx])
        self.state = 'Z_STAB' if abs(current_pos[2] - self.target[2]) > 2.2 else 'NAV'
        self.is_re_stabilizing = False
        self.in_back_off_maneuver = False
        self.reset_performed_for_current_wp = False
        self.xy_final_start_time = clock_now

    def evaluate(self, state: VehicleState, clock_now: float) -> str:
        if self.state in ['XY_FINAL', 'HOVER_STAB'] and abs(state.z_err) > ControlConfig.altitude_breach_threshold:
            self.state = 'Z_STAB'; self.is_re_stabilizing = True
            self.in_back_off_maneuver = False; self.xy_final_start_time = clock_now
        if self.state == 'NAV':
            if state.dist_2d < 2.5: self.state = 'Z_STAB'
        elif self.state == 'Z_STAB':
            predicted = state.z_err + (state.dz_dt * 1.0)
            if abs(predicted) < 1.2 and abs(state.z_err) < 1.0 and abs(state.dz_dt) < 0.14:
                if state.dist_2d > 3.5 and not self.is_re_stabilizing: self.state = 'NAV'
                else:
                    self.state = 'XY_FINAL'; self.is_re_stabilizing = False
                    self.xy_final_start_time = clock_now; self.in_back_off_maneuver = False
        elif self.state == 'XY_FINAL':
            if not self.in_back_off_maneuver and not self.reset_performed_for_current_wp and (clock_now - self.xy_final_start_time) > 9.0:
                self.in_back_off_maneuver = True; self.reset_performed_for_current_wp = True
                self.back_off_start_time = clock_now
            if self.in_back_off_maneuver:
                if state.dist_2d > 3.0 or (clock_now - self.back_off_start_time) > 2.5:
                    self.in_back_off_maneuver = False; self.xy_final_start_time = clock_now
            else:
                if state.dist_2d <= ControlConfig.success_radius and abs(state.z_err) <= 0.85:
                    self.state = 'HOVER_STAB'
        return self.state

    def get_target_params(self, state: VehicleState) -> Dict:
        params = {'base_speed': 0.0, 'yaw_diff': 0.0}
        if self.state == 'NAV':
            scale = min(1.0, state.dist_2d / 15.0)
            target = ControlConfig.max_cruise_speed * scale
            if state.roll_abs > 0.15: target *= 0.55
            params['base_speed'] = target; params['yaw_diff'] = 5.0 * state.yaw_err
        elif self.state == 'Z_STAB':
            target = -18.0 if (self.is_re_stabilizing and abs(state.z_err) > 1.4) else -16.0
            if state.roll_abs > 0.15: target *= 0.6
            params['base_speed'] = target; params['yaw_diff'] = 5.0 * state.yaw_err
        elif self.state == 'XY_FINAL':
            if self.in_back_off_maneuver:
                params['base_speed'] = 20.0; params['yaw_diff'] = 0.0
            else:
                target = -11.0
                if state.roll_abs > 0.10: target *= 0.5
                params['base_speed'] = target
                params['yaw_diff'] = min(4.5, max(-4.5, 5.0 * state.yaw_err))
        return params
