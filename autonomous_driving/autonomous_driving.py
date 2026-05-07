#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import annotations

from .localization.path_manager import PathManager
from .planning.adaptive_cruise_control import AdaptiveCruiseControl
from .control.pure_pursuit import PurePursuit
from .control.pid import Pid
from .control.control_input import ControlInput
from .config.config import Config
from .mgeo.calc_mgeo_path import mgeo_dijkstra_path



class AutonomousDriving:
    def __init__(self, path_file_name=None, map_name=None, max_speed_kph=None):
        config = Config()
        self._velocity_profile_cfg = dict(config['planning']['velocity_profile'])
        self._max_speed_kph = None

        if config["map"]["use_mgeo_path"]:
            mgeo_path = mgeo_dijkstra_path(config["map"]["name"])
            self.path = mgeo_path.calc_dijkstra_path(config["map"]["mgeo"]["start_node"], config["map"]["mgeo"]["end_node"])
            self.path_manager = PathManager(
                self.path, config["map"]["is_closed_path"], config["map"]["local_path_size"]
            )
        else:
            if path_file_name:
                self.path = config.load_path(path_file_name, map_name=map_name)
            else:
                self.path = config["map"]["path"]
            self.path_manager = PathManager(
                self.path, config["map"]["is_closed_path"], config["map"]["local_path_size"]
            )
        self.set_max_speed_kph(max_speed_kph)



        self.adaptive_cruise_control = AdaptiveCruiseControl(
            vehicle_length=config['common']['vehicle_length'], **config['planning']['adaptive_cruise_control']
        )
        self.pid = Pid(sampling_time=1/float(config['common']['sampling_rate']), **config['control']['pid'])
        self.pure_pursuit = PurePursuit(
            wheelbase=config['common']['wheelbase'], **config['control']['pure_pursuit']
        )

    def set_max_speed_kph(self, max_speed_kph=None):
        velocity_profile_cfg = dict(self._velocity_profile_cfg)
        self._max_speed_kph = None
        if max_speed_kph is not None:
            self._max_speed_kph = float(max_speed_kph)
            velocity_profile_cfg["max_velocity"] = self._max_speed_kph
        self.path_manager.set_velocity_profile(**velocity_profile_cfg)
        if hasattr(self, "pid"):
            self.pid.reset()

    def execute(self, vehicle_state):
        # 현재 위치 기반으로 local path과 planned velocity 추출
        local_path, planned_velocity = self.path_manager.get_local_path(vehicle_state)



        # adaptive cruise control를 활용한 속도 계획
        target_velocity = self.adaptive_cruise_control.get_target_velocity(vehicle_state.velocity, planned_velocity)
        if self._max_speed_kph is not None:
            target_velocity = min(target_velocity, self._max_speed_kph / 3.6)
        # 속도 제어를 위한 PID control
        acc_cmd = self.pid.get_output(target_velocity, vehicle_state.velocity)
        # 경로 추종을 위한 pure pursuit control
        self.pure_pursuit.path = local_path
        self.pure_pursuit.vehicle_state = vehicle_state
        steering_cmd = self.pure_pursuit.calculate_steering_angle()

        return ControlInput(acc_cmd, steering_cmd), local_path
