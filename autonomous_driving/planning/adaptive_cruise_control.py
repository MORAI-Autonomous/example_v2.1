#!/usr/bin/env python
# -*- coding: utf-8 -*-
class AdaptiveCruiseControl:
    def __init__(self, velocity_gain, distance_gain, time_gap, vehicle_length):
        self.velocity_gain = velocity_gain
        self.distance_gain = distance_gain
        self.time_gap = time_gap
        self.vehicle_length = vehicle_length



    def get_target_velocity(self, ego_vel, target_vel):
        return target_vel
