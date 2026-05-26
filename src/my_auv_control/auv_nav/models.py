from dataclasses import dataclass, field
from typing import List, Dict, Tuple

@dataclass
class VehicleState:
    pos: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    vel: float = 0.0
    rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    baro_z: float = 0.0
    dz_dt: float = 0.0
    dist_2d: float = 1000.0
    dist_3d: float = 1000.0
    bearing: float = 0.0
    z_err: float = 0.0
    yaw_err: float = 0.0
    roll_abs: float = 0.0
    pitch_curr: float = 0.0
    pitch_d: float = 0.0
    yaw_d: float = 0.0

@dataclass
class ActuatorCommands:
    lt: float = 0.0
    rt: float = 0.0
    rv: float = 0.0
    hl: float = 0.0
    hr: float = 0.0

PHASE_TRANSLATION = {
    'NAV': 'Круиз', 'Z_STAB': 'Коррекция высоты', 'XY_FINAL': 'Сближение',
    'HOVER_STAB': 'Стабилизация', 'FINISH': 'Готово'
}

class ControlConfig:
    P_Z0 = 101325.0; RHO_G = 9810.0
    Kp_z_base = 18.0; Kd_z_base = 22.0
    Kp_yaw = 5.0; Kd_yaw = 2.8
    Kp_roll = 50.0; Kd_roll = 22.0
    roll_bias = 0.04
    max_rudder_speed = 2.4
    max_cruise_speed = -5.0; min_cruise_speed = -1.0
    success_radius = 0.85
    altitude_breach_threshold = 1.7
    dt = 0.05
