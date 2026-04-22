#!/usr/bin/env python
# -*- coding: utf-8 -*-

from .localization.path_manager import PathManager
from .planning.adaptive_cruise_control import AdaptiveCruiseControl
from .control.pure_pursuit import PurePursuit
from .control.pid import Pid
from .control.control_input import ControlInput
from .config.config import Config
from .mgeo.calc_mgeo_path import mgeo_dijkstra_path



class AutonomousDriving:
    def __init__(self, path_file_name=None, map_name=None):
        config = Config()

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
        self.path_manager.set_velocity_profile(**config['planning']['velocity_profile'])



        self.adaptive_cruise_control = AdaptiveCruiseControl(
            vehicle_length=config['common']['vehicle_length'], **config['planning']['adaptive_cruise_control']
        )
        self.pid = Pid(sampling_time=1/float(config['common']['sampling_rate']), **config['control']['pid'])
        self.pure_pursuit = PurePursuit(
            wheelbase=config['common']['wheelbase'], **config['control']['pure_pursuit']
        )

    def execute(self, vehicle_state):
        # 현재 위치 기반으로 local path과 planned velocity 추출
        local_path, planned_velocity = self.path_manager.get_local_path(vehicle_state)



        # adaptive cruise control를 활용한 속도 계획
        target_velocity = self.adaptive_cruise_control.get_target_velocity(vehicle_state.velocity, planned_velocity)
        # 속도 제어를 위한 PID control
        acc_cmd = self.pid.get_output(target_velocity, vehicle_state.velocity)
        # 경로 추종을 위한 pure pursuit control
        self.pure_pursuit.path = local_path
        self.pure_pursuit.vehicle_state = vehicle_state
        steering_cmd = self.pure_pursuit.calculate_steering_angle()

        return ControlInput(acc_cmd, steering_cmd), local_path
