#!/usr/bin/env python
# -*- coding: utf-8 -*-
from ..vehicle_state import VehicleState
import numpy as np


class PurePursuit(object):
    def __init__(self, lfd_gain, wheelbase, min_lfd, max_lfd):
        """Pure Pursuit 알고리즘을 이용한 Steering 계산"""
        self.lfd_gain = lfd_gain
        self.wheelbase = wheelbase
        self.min_lfd = min_lfd
        self.max_lfd = max_lfd

        self._path = []
        self._vehicle_state = VehicleState()

    @property
    def path(self):
        return self._path

    @property
    def vehicle_state(self):
        return self._vehicle_state

    @path.setter
    def path(self, path):
        self._path = path

    @vehicle_state.setter
    def vehicle_state(self, vehicle_state):
        self._vehicle_state = vehicle_state

    def calculate_steering_angle(self):
        lfd = self.lfd_gain * self._vehicle_state.velocity
        lfd = np.clip(lfd, self.min_lfd, self.max_lfd)

        steering_angle = 0.
        lookahead_found = False
        for i, point in enumerate(self._path):
            diff = point - self._vehicle_state.position
            rotated_diff = diff.rotate(-self._vehicle_state.yaw)
            
            #print(f"  Path Point {i}: x={point.x:.2f}, y={point.y:.2f}")
            #print(f"  Rotated Diff: x={rotated_diff.x:.2f}, y={rotated_diff.y:.2f}")

            if rotated_diff.x > 0: # 차량 전방의 경로점만 고려
                dis = rotated_diff.distance()
                if dis >= lfd: # lookahead distance 이상 떨어진 경로점
                    theta = rotated_diff.angle
                    steering_angle = np.arctan2(2*self.wheelbase*np.sin(theta), lfd)
                    lookahead_found = True
                    #print(f"  Lookahead Point Found (Idx {i}): Dis={dis:.2f}, Theta={np.rad2deg(theta):.2f} deg, Calculated Steering={np.rad2deg(steering_angle):.2f} deg")
                    break
        
        if not lookahead_found:
            print("  No suitable lookahead point found. Steering angle remains 0.")

        print(f"Final Steering Angle (rad): {steering_angle:.4f}, (deg): {np.rad2deg(steering_angle):.4f}")
        return steering_angle
