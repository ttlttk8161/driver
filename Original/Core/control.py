#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

class Control:
    def __init__(self):
        self.target_speed = 10.0
        self.target_angle = 0.0
        self.max_speed = 50.0
        self.min_speed = -10.0
        self.max_angle = 50.0
        self.emergency_stop = False
        
        # PID 제어 파라미터 (추후 튜닝)
        self.kp_lateral = 1.0
        self.ki_lateral = 0.0
        self.kd_lateral = 0.0
        self.lateral_error_sum = 0.0
        self.prev_lateral_error = 0.0
        
    def calculate_control(self, lane_data, obstacle_data):
        """
        Perception 데이터를 바탕으로 제어 명령 계산
        Args:
            lane_data: 차선 정보
            obstacle_data: 장애물 정보
        Returns:
            tuple: (조향각, 속도)
        """
        # 1. 장애물+차선 기반 속도 제어
        self.target_speed = self._calculate_speed_control(obstacle_data, lane_data)
        
        # 2. 차선 기반 조향 제어
        self.target_angle = self._calculate_steering_control(lane_data)
        
        # 3. 안전 검사
        self._safety_check(obstacle_data)
        
        # 4. 제한값 적용
        self.target_speed = self._limit_speed(self.target_speed)
        self.target_angle = self._limit_angle(self.target_angle)
        
        return self.target_angle, self.target_speed
    
    def _calculate_speed_control(self, obstacle_data, lane_data=None):
        """
        장애물 정보 + 차선 인식 기반 속도 제어
        Args:
            obstacle_data: 장애물 분석 결과
            lane_data: 차선 인식 결과(waypoint 리스트)
        Returns:
            float: 목표 속도
        """
        # 장애물 우선
        if obstacle_data is not None:
            front_distance = obstacle_data.get('front_distance', float('inf'))
            if front_distance < 0.5:
                return self.min_speed * 0.5
            elif front_distance < 1.0:
                return self.min_speed
            elif front_distance < 2.0:
                return self.target_speed * 0.7
        # 차선 인식이 잘 되고 장애물 없을 때 속도 50으로 점진적 가속
        if lane_data is not None and isinstance(lane_data, list) and len(lane_data) >= 6:
            # 현재 속도가 50 미만이면 점진적으로 증가
            accel = 5  # 가속도 (프레임당 증가량, 필요시 조정)
            if self.target_speed < 50.0:
                self.target_speed = min(self.target_speed + accel, 50.0)
            return self.target_speed
        # 기본값
        return self.target_speed
    
    def _calculate_steering_control(self, lane_data):
        """
        차선 정보 기반 조향 제어 (waypoint 기반, 최하단 바로 위 waypoint 기준)
        Args:
            lane_data: waypoint 리스트
        Returns:
            float: 목표 조향각
        """
        # 차선 검출이 3개 미만일 때도 마지막 정상값을 사용하여 계속 시도
        if lane_data is None or not isinstance(lane_data, list) or len(lane_data) < 3:
            # 마지막 정상값이 perception에서 제공된다고 가정
            if hasattr(self, 'get_valid_waypoints'):
                lane_data = self.get_valid_waypoints()
            # 그래도 없으면 0 반환
            if lane_data is None or len(lane_data) < 3:
                return 0.0

        # 이미지 중심값 (카메라 해상도에 맞게 수정, 예: 640)
        IMAGE_WIDTH = 640
        center_x = IMAGE_WIDTH // 2

        # y값 기준 내림차순 정렬
        sorted_wps = sorted(lane_data, key=lambda p: p[1], reverse=True)
        target_wp = sorted_wps[2]
        lateral_error = target_wp[0] - center_x

        # PID 제어
        self.lateral_error_sum += lateral_error
        lateral_error_diff = lateral_error - self.prev_lateral_error

        pid_output = (self.kp_lateral * lateral_error +
                      self.ki_lateral * self.lateral_error_sum +
                      self.kd_lateral * lateral_error_diff)

        self.prev_lateral_error = lateral_error

        return pid_output
    
    def _safety_check(self, obstacle_data):
        """
        안전 검사 및 비상 정지 판단
        Args:
            obstacle_data: 장애물 정보
        """
        if obstacle_data is None:
            return
            
        # 매우 가까운 장애물 감지 시 비상 정지
        front_distance = obstacle_data.get('front_distance', float('inf'))
        if front_distance < 0.2:  # 20cm 이내
            self.emergency_stop = True
        else:
            self.emergency_stop = False
    
    def _limit_speed(self, speed):
        """속도 제한 적용"""
        if self.emergency_stop:
            return 0.0
        return max(self.min_speed, min(self.max_speed, speed))
    
    def _limit_angle(self, angle):
        """조향각 제한 적용"""
        return max(-self.max_angle, min(self.max_angle, angle))
    
    def set_pid_gains(self, kp, ki, kd):
        """PID 게인 설정"""
        self.kp_lateral = kp
        self.ki_lateral = ki
        self.kd_lateral = kd
    
    def set_speed_limits(self, min_speed, max_speed):
        """속도 제한 설정"""
        self.min_speed = min_speed
        self.max_speed = max_speed
    
    def set_target_speed(self, speed):
        """목표 속도 설정"""
        self.target_speed = speed
    
    def reset_pid(self):
        """PID 상태 초기화"""
        self.lateral_error_sum = 0.0
        self.prev_lateral_error = 0.0
    
    def is_emergency_stop(self):
        """비상 정지 상태 반환"""
        return self.emergency_stop
