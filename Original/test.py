#!/usr/bin/env python
# -*- coding: utf-8 -*-

#======================================================
# 2025 제8회 국민대 자율주행 경진대회 예선 과제용 파일
# 지능형 후진 시스템 개선 버전
#======================================================

import numpy as np
import cv2, rospy, time, math, gc, os
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import psutil
from dataclasses import dataclass
from typing import Tuple, Optional
from sklearn.cluster import DBSCAN

# ──────────────────────────────────────────
# 환경 최적화
# ──────────────────────────────────────────
os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'
cv2.setNumThreads(2)

# ──────────────────────────────────────────
# 상수
# ──────────────────────────────────────────
MIN_PASS_WIDTH = 8.0   # 라바콘 사이 최소 통과 가능 폭 (m)

# === 지능형 백업(후진) 로직 ===
DEADLOCK_DISTANCE = 2.0      # 막다른 길 감지 거리
DEADLOCK_ANGLE_RANGE = 120   # 전방 감지 각도 범위 (±60도)
MIN_SIDE_CLEARANCE = 1.5     # 측면 최소 여유 공간
BACKUP_FRAMES = 30           # 후진 지속 프레임
BACKUP_SPEED = -10.0         # 후진 속도

# === TopView 시각화 ===
TOPVIEW_SIZE = 500           # TopView 이미지 크기
TOPVIEW_RANGE = 15.0         # 표시할 최대 거리 (m)

# === S 코스 감지 및 처리 ===
S_COURSE_DETECT_FRAMES = 5
S_COURSE_MIN_OBSTACLES = 4
S_COURSE_WIDE_TURN_ANGLE = 35.0

# ──────────────────────────────────────────
# TopView 시각화 및 공간 분석 클래스
# ──────────────────────────────────────────
class IntelligentBackupSystem:
    def __init__(self, size=TOPVIEW_SIZE, max_range=TOPVIEW_RANGE):
        self.size = size
        self.max_range = max_range
        self.center = size // 2
        self.pixels_per_meter = size / (2 * max_range)
        
        # 시각화용 이미지
        self.topview_img = np.zeros((size, size, 3), dtype=np.uint8)
        self.space_analysis = {}
        
        # 후진 상태
        self.backup_needed = False
        self.backup_direction = 0.0
        self.backup_confidence = 0.0
        
    def world_to_pixel(self, x, y):
        """월드 좌표를 픽셀 좌표로 변환"""
        px = int(self.center + x * self.pixels_per_meter)
        py = int(self.center - y * self.pixels_per_meter)
        return px, py
    
    def analyze_space_distribution(self, ranges):
        """360도 공간 분포 분석"""
        if ranges is None or ranges.size == 0:
            return {}
        
        # 8방향으로 나누어 분석
        directions = {
            'front': (330, 30),      # 전방 ±30도
            'front_left': (30, 90),   # 전방 좌측
            'left': (90, 150),        # 좌측
            'back_left': (150, 210),  # 후방 좌측
            'back': (210, 330),       # 후방
            'back_right': (270, 330), # 후방 우측 (210~270을 270~330으로 수정)
            'right': (210, 270),      # 우측 (270~330을 210~270으로 수정)
            'front_right': (270, 330) # 전방 우측 (330~30을 270~330으로 수정)
        }
        
        # 수정된 방향 정의
        directions = {
            'front': (330, 30),       # 전방 ±30도  
            'front_right': (300, 330), # 전방 우측
            'right': (240, 300),      # 우측
            'back_right': (180, 240), # 후방 우측
            'back': (120, 180),       # 후방
            'back_left': (60, 120),   # 후방 좌측
            'left': (30, 90),         # 좌측
            'front_left': (30, 60)    # 전방 좌측
        }
        
        analysis = {}
        
        for direction, (start_angle, end_angle) in directions.items():
            if start_angle > end_angle:  # 0도를 넘나드는 경우
                indices = list(range(start_angle, 360)) + list(range(0, end_angle))
            else:
                indices = list(range(start_angle, end_angle))
            
            sector_ranges = ranges[indices]
            valid_ranges = sector_ranges[(sector_ranges > 0.1) & (sector_ranges < self.max_range)]
            
            if valid_ranges.size > 0:
                avg_dist = np.mean(valid_ranges)
                min_dist = np.min(valid_ranges)
                max_dist = np.max(valid_ranges)
                clearance = np.sum(valid_ranges > 2.0) / valid_ranges.size  # 2m 이상 여유공간 비율
            else:
                avg_dist = self.max_range
                min_dist = self.max_range
                max_dist = self.max_range
                clearance = 1.0
            
            analysis[direction] = {
                'avg_distance': avg_dist,
                'min_distance': min_dist,
                'max_distance': max_dist,
                'clearance_ratio': clearance,
                'angle_center': (start_angle + end_angle) / 2 if start_angle <= end_angle else 
                               ((start_angle + end_angle + 360) / 2) % 360
            }
        
        return analysis
    
    def detect_deadlock_situation(self, ranges):
        """막다른 상황 감지"""
        if ranges is None or ranges.size == 0:
            return False, "No LiDAR data"
        
        analysis = self.analyze_space_distribution(ranges)
        
        # 전방이 막혔는지 확인
        front_blocked = analysis['front']['min_distance'] < DEADLOCK_DISTANCE
        front_left_blocked = analysis['front_left']['min_distance'] < DEADLOCK_DISTANCE
        front_right_blocked = analysis['front_right']['min_distance'] < DEADLOCK_DISTANCE
        
        # 좌우 공간 확인
        left_clear = analysis['left']['avg_distance'] > MIN_SIDE_CLEARANCE
        right_clear = analysis['right']['avg_distance'] > MIN_SIDE_CLEARANCE
        
        # 막다른 상황: 전방이 막히고 좌우 중 적어도 하나가 막힌 상황
        deadlock = front_blocked and front_left_blocked and front_right_blocked
        
        # 추가 조건: 회전할 공간도 부족한 경우
        no_turn_space = not left_clear and not right_clear
        
        reason = ""
        if deadlock:
            if no_turn_space:
                reason = "Complete deadlock - no turning space"
            else:
                reason = "Front blocked - backup needed"
        
        return deadlock, reason
    
    def calculate_optimal_backup_direction(self, ranges):
        """최적 후진 방향 계산"""
        analysis = self.analyze_space_distribution(ranges)
        
        # 후방 방향들만 고려 (back, back_left, back_right)
        backup_directions = ['back', 'back_left', 'back_right']
        
        best_direction = None
        best_score = 0.0
        
        for direction in backup_directions:
            if direction in analysis:
                data = analysis[direction]
                # 점수 = 평균거리 * 여유공간비율 * 최소거리 가중치
                score = (data['avg_distance'] * 0.4 + 
                        data['clearance_ratio'] * 3.0 + 
                        data['min_distance'] * 0.6)
                
                if score > best_score:
                    best_score = score
                    best_direction = direction
        
        if best_direction:
            angle = analysis[best_direction]['angle_center']
            # 후진을 위해 조향각 계산 (후진시에는 반대로 조향)
            if best_direction == 'back_left':
                steering_angle = 25.0  # 후진시 우측으로 조향하여 좌측 후방으로 이동
            elif best_direction == 'back_right':
                steering_angle = -25.0  # 후진시 좌측으로 조향하여 우측 후방으로 이동
            else:  # back
                steering_angle = 0.0
            
            return steering_angle, best_score, best_direction
        
        return 0.0, 0.0, "none"
    
    def update_topview(self, ranges):
        """TopView 업데이트"""
        # 이미지 초기화
        self.topview_img.fill(0)
        
        # 격자 그리기
        self.draw_grid()
        
        # 로봇 위치 표시
        cv2.circle(self.topview_img, (self.center, self.center), 12, (255, 255, 255), -1)
        cv2.circle(self.topview_img, (self.center, self.center), 12, (0, 0, 0), 2)
        cv2.putText(self.topview_img, "ROBOT", (self.center-20, self.center+25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        if ranges is None or ranges.size == 0:
            return self.topview_img
        
        # 공간 분석
        analysis = self.analyze_space_distribution(ranges)
        self.space_analysis = analysis
        
        # LiDAR 점들 그리기
        for i, dist in enumerate(ranges):
            if 0.1 < dist < self.max_range:
                angle = np.deg2rad(i)
                x = dist * np.cos(angle)
                y = dist * np.sin(angle)
                
                px, py = self.world_to_pixel(x, y)
                if 0 <= px < self.size and 0 <= py < self.size:
                    # 거리에 따른 색상
                    if dist < 2.0:
                        color = (0, 0, 255)  # 빨강 - 위험
                    elif dist < 5.0:
                        color = (0, 165, 255)  # 주황 - 주의
                    else:
                        color = (0, 255, 0)  # 초록 - 안전
                    
                    cv2.circle(self.topview_img, (px, py), 2, color, -1)
        
        # 방향별 공간 표시
        self.draw_space_analysis(analysis)
        
        # 막다른 상황 및 후진 방향 분석
        deadlock, reason = self.detect_deadlock_situation(ranges)
        
        if deadlock:
            steering, confidence, best_dir = self.calculate_optimal_backup_direction(ranges)
            self.backup_needed = True
            self.backup_direction = steering
            self.backup_confidence = confidence
            
            # 후진 방향 표시
            self.draw_backup_direction(best_dir, confidence)
            
            # 상태 텍스트
            cv2.putText(self.topview_img, "DEADLOCK DETECTED", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(self.topview_img, f"Backup: {best_dir}", (10, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(self.topview_img, f"Confidence: {confidence:.2f}", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        else:
            self.backup_needed = False
            cv2.putText(self.topview_img, "NORMAL OPERATION", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return self.topview_img
    
    def draw_grid(self):
        """격자 그리기"""
        # 중심선
        cv2.line(self.topview_img, (self.center, 0), (self.center, self.size), (50, 50, 50), 1)
        cv2.line(self.topview_img, (0, self.center), (self.size, self.center), (50, 50, 50), 1)
        
        # 거리 원 (2m, 5m, 10m)
        for r in [2, 5, 10]:
            if r <= self.max_range:
                radius = int(r * self.pixels_per_meter)
                cv2.circle(self.topview_img, (self.center, self.center), radius, (30, 30, 30), 1)
                # 거리 표시
                cv2.putText(self.topview_img, f"{r}m", (self.center + radius - 15, self.center - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 100, 100), 1)
    
    def draw_space_analysis(self, analysis):
        """방향별 공간 분석 결과 표시"""
        for direction, data in analysis.items():
            angle_center = np.deg2rad(data['angle_center'])
            avg_dist = min(data['avg_distance'], self.max_range)
            
            # 방향벡터 계산
            x = avg_dist * np.cos(angle_center) * 0.7  # 70% 지점에 표시
            y = avg_dist * np.sin(angle_center) * 0.7
            
            px, py = self.world_to_pixel(x, y)
            
            # 공간 상태에 따른 색상
            if data['clearance_ratio'] > 0.8:
                color = (0, 255, 0)  # 초록 - 여유공간 충분
            elif data['clearance_ratio'] > 0.5:
                color = (0, 255, 255)  # 노랑 - 보통
            else:
                color = (0, 0, 255)  # 빨강 - 공간 부족
            
            # 방향 표시
            cv2.circle(self.topview_img, (px, py), 8, color, -1)
            cv2.putText(self.topview_img, direction[:2].upper(), (px-8, py+3), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    
    def draw_backup_direction(self, best_direction, confidence):
        """후진 방향 화살표 그리기"""
        if best_direction == "none":
            return
        
        direction_angles = {
            'back': 180,
            'back_left': 135,
            'back_right': 225
        }
        
        if best_direction in direction_angles:
            angle = np.deg2rad(direction_angles[best_direction])
            arrow_length = 80
            
            end_x = int(self.center + arrow_length * np.cos(angle))
            end_y = int(self.center - arrow_length * np.sin(angle))
            
            # 화살표 색상 (신뢰도에 따라)
            if confidence > 2.0:
                color = (0, 255, 0)    # 초록 - 높은 신뢰도
            elif confidence > 1.0:
                color = (0, 255, 255)  # 노랑 - 중간 신뢰도
            else:
                color = (0, 0, 255)    # 빨강 - 낮은 신뢰도
            
            # 화살표 그리기
            cv2.arrowedLine(self.topview_img, (self.center, self.center), 
                           (end_x, end_y), color, 4, tipLength=0.3)

# ──────────────────────────────────────────
# 기존 보조 함수들
# ──────────────────────────────────────────
def safe_mean(arr: np.ndarray, default: Optional[float] = np.nan) -> float:
    return float(np.mean(arr)) if arr.size else default

def get_dynamic_steer_limit(dist: float, s_course_mode: bool = False) -> float:
    if s_course_mode:
        if dist < 6.0:
            return 120.0
        elif dist < 12.0:
            return 100.0
        return 80.0
    else:
        if dist < 9.0:
            return 100.0
        elif dist < 14.0:
            return 80.0
        return 50.0

def detect_s_course(ranges: np.ndarray) -> bool:
    if ranges is None or ranges.size == 0:
        return False
    
    front = ranges[330:].tolist() + ranges[:30].tolist()
    left = ranges[45:135]
    right = ranges[225:315]
    
    front_obstacles = np.sum((np.array(front) > 0.1) & (np.array(front) < 3.0))
    left_obstacles = np.sum((left > 0.1) & (left < 4.0))
    right_obstacles = np.sum((right > 0.1) & (right < 4.0))
    
    total_obstacles = front_obstacles + left_obstacles + right_obstacles
    
    return (total_obstacles > S_COURSE_MIN_OBSTACLES and 
            left_obstacles > 1 and right_obstacles > 1)

# ──────────────────────────────────────────
# ACO 알고리즘 (기존 코드 유지)
# ──────────────────────────────────────────
@dataclass
class ACOParams:
    num_ants:        int   = 20
    alpha:           float = 3.0
    beta:            float = 3.0
    rho:             float = 0.4
    Q:               float = 100.0
    max_iterations:  int   = 60
    grid_size:       float = 10.0
    safety_distance: float = 4.0

class LidarACOAvoidance:
    __slots__ = ("params", "grid_map", "pheromone_map", "obstacle_map",
                 "current_pos", "goal_pos", "directions", "s_course_counter")

    MAX_LIDAR_DIST = 40.0

    def __init__(self, params: ACOParams):
        self.params = params
        self.grid_map = None
        self.pheromone_map = None
        self.obstacle_map = None
        self.current_pos = (0, 0)
        self.goal_pos = (0, 0)
        self.directions = [(-1, -1), (-1, 0), (-1, 1),
                           (0, -1),          (0, 1),
                           (1, -1),  (1, 0), (1, 1)]
        self.s_course_counter = 0

    # =================================================================
    # === 수정된 부분: 근접 장애물에 대한 조향 반응성 강화 ===
    # =================================================================
    @staticmethod
    def distance_weight(dist: float, s_course_mode: bool = False) -> float:
        """
        장애물과의 거리에 따라 조향 가중치를 동적으로 계산합니다.
        거리가 가까울수록 더 큰 가중치를 반환하여 회피 기동을 공격적으로 만듭니다.
        근접 장애물에 대한 반응성을 크게 향상시켰습니다.
        """
        if s_course_mode:
            # S코스 모드: 약간 더 부드럽게 반응하지만 근접 시에는 여전히 강하게 조향
            if dist < 2.0:
                # 2m 미만: 매우 강한 조향
                return 2.8
            elif dist < 4.0:
                # 2m ~ 4m: 거리에 반비례하여 강하게 조향
                # dist=2.0 -> 2.8, dist=4.0 -> 1.5
                return 1.5 + 1.3 * (4.0 - dist) / 2.0
            elif dist < 8.0:
                # 4m ~ 8m: 완만한 조향 강화
                # dist=4.0 -> 1.8, dist=8.0 -> 1.0
                return 1.0 + 0.8 * (8.0 - dist) / 4.0
            else:
                return 1.0
        else:
            # 일반 모드: 매우 공격적으로 반응
            if dist < 1.5:
                # 1.5m 미만: 최대 가중치로 급격한 조향
                return 4.0
            elif dist < 3.0:
                # 1.5m ~ 3.0m: 거리에 따라 가중치를 급격히 증가
                # dist=1.5 -> 4.0, dist=3.0 -> 2.0
                return 2.0 + 2.0 * (3.0 - dist) / 1.5
            elif dist < 6.0:
                # 3.0m ~ 6.0m: 기존과 유사하게 점진적으로 가중치 증가
                # dist=3.0 -> 2.0, dist=6.0 -> 1.0
                return 1.0 + 1.0 * (6.0 - dist) / 3.0
            else:
                return 1.0
    # =================================================================
    # === 수정 완료 ===
    # =================================================================

    def update_lidar_data(self, ranges: np.ndarray,
                          robot_x: float = 0.0, robot_y: float = 0.0,
                          s_course_mode: bool = False):
        if ranges is None or ranges.size == 0:
            return

        map_size = 100
        if self.grid_map is None:
            self.grid_map = np.zeros((map_size, map_size), np.uint8)
            self.obstacle_map = np.zeros_like(self.grid_map)
            self.pheromone_map = np.ones((map_size, map_size), np.float32) * 0.1
        else:
            self.grid_map.fill(0)
            self.obstacle_map.fill(0)
            self.pheromone_map *= (1 - self.params.rho)

        c = map_size // 2
        self.current_pos = (c, c)

        safety_dist = self.params.safety_distance * (0.7 if s_course_mode else 1.0)

        for i, d in enumerate(ranges):
            if 0.1 < d < self.MAX_LIDAR_DIST:
                ang = (i * 2 * np.pi / len(ranges)) - np.pi
                ox, oy = d * np.cos(ang), d * np.sin(ang)
                gx = int(c + ox / self.params.grid_size)
                gy = int(c + oy / self.params.grid_size)
                if 0 <= gx < map_size and 0 <= gy < map_size:
                    self.obstacle_map[gx, gy] = 1
                    safety = int(safety_dist / self.params.grid_size)
                    for dx in range(-safety, safety+1):
                        for dy in range(-safety, safety+1):
                            if dx*dx + dy*dy <= safety*safety:
                                nx, ny = gx+dx, gy+dy
                                if 0 <= nx < map_size and 0 <= ny < map_size:
                                    self.obstacle_map[nx, ny] = 1

    def calculate_heuristic(self, p, g):
        return math.hypot(p[0]-g[0], p[1]-g[1])

    def get_valid_neighbors(self, pos):
        neigh = []
        x, y = pos
        for dx, dy in self.directions:
            nx, ny = x+dx, y+dy
            if (0 <= nx < self.obstacle_map.shape[0] and
                    0 <= ny < self.obstacle_map.shape[1] and
                    self.obstacle_map[nx, ny] == 0):
                neigh.append((nx, ny))
        return neigh

    def ant_walk(self, start, goal):
        path, visited = [start], {start}
        cur = start
        for _ in range(200):
            if cur == goal:
                break
            neigh = self.get_valid_neighbors(cur)
            if not neigh:
                break
            unv = [n for n in neigh if n not in visited]
            if unv:
                neigh = unv
            probs = []
            for nx, ny in neigh:
                pher = self.pheromone_map[nx, ny]
                heur = 1.0 / (1.0 + self.calculate_heuristic((nx, ny), goal))
                probs.append((pher ** self.params.alpha) *
                             (heur ** self.params.beta))
            if sum(probs) > 0:
                probs = np.asarray(probs) / sum(probs)
                cur = neigh[np.random.choice(len(neigh), p=probs)]
            else:
                cur = neigh[0]
            path.append(cur)
            visited.add(cur)
        return path

    def update_pheromones(self, paths):
        self.pheromone_map *= (1 - self.params.rho)
        for p in paths:
            if len(p) < 2:
                continue
            d_pher = self.params.Q / len(p)
            for x, y in p:
                self.pheromone_map[x, y] += d_pher

    def find_local_goal(self, heading_rad, s_course_mode: bool = False):
        m = self.obstacle_map.shape[0]
        rx, ry = self.current_pos
        
        d = 35 if s_course_mode else 25
        
        tx = int(np.clip(rx + d * np.cos(heading_rad), 0, m-1))
        ty = int(np.clip(ry + d * np.sin(heading_rad), 0, m-1))
        
        if self.obstacle_map[tx, ty] == 0:
            return (tx, ty)
        
        search_range = 15 if s_course_mode else 10
        for r in range(1, search_range):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    nx, ny = tx+dx, ty+dy
                    if (0 <= nx < m and 0 <= ny < m and
                            self.obstacle_map[nx, ny] == 0):
                        return (nx, ny)
        return (rx, ry)

    def aco_path_planning(self, heading_rad, min_dist, s_course_mode: bool = False):
        if self.obstacle_map is None:
            return 0.0, False
            
        self.goal_pos = self.find_local_goal(heading_rad, s_course_mode)
        best, best_len = None, float('inf')

        max_itr = self.params.max_iterations * (2 if s_course_mode else 1)
        
        for itr in range(max_itr):
            paths = []
            for _ in range(self.params.num_ants):
                p = self.ant_walk(self.current_pos, self.goal_pos)
                if p[-1] == self.goal_pos:
                    paths.append(p)
                    if len(p) < best_len:
                        best, best_len = p.copy(), len(p)
            if paths:
                self.update_pheromones(paths)
            if best and itr > (10 if s_course_mode else 5):
                break

        if best and len(best) >= 2:
            ang = self.calculate_steering_angle(best, min_dist, s_course_mode)
            return ang, True
        return 0.0, False

    def calculate_steering_angle(self, path, min_dist=5.0, s_course_mode: bool = False):
        if len(path) < 2:
            return 0.0
            
        look = min(10 if s_course_mode else 6, len(path) - 1)
        cx, cy = path[0]
        tx, ty = path[look]
        dx, dy = tx - cx, ty - cy
        
        if dx == 0 and dy == 0:
            return 0.0
            
        limit = get_dynamic_steer_limit(min_dist, s_course_mode)
        base = math.degrees(math.atan2(dy, dx))
        base = np.clip(base, -limit, limit)
        
        steer = base * self.distance_weight(min_dist, s_course_mode)
        return float(np.clip(steer, -limit, limit))

# ──────────────────────────────────────────
# GAP 탐색 함수들 (기존 코드 유지)
# ──────────────────────────────────────────
PP_LOOK = 6.0

def pure_pursuit_angle(mid_xy: Tuple[float, float]) -> float:
    tx, ty = mid_xy
    if math.hypot(tx, ty) < 0.001:
        return 0.0
    return math.degrees(math.atan2(ty, tx))

def find_widest_or_center_gap(ranges: np.ndarray,
                              min_width: float = MIN_PASS_WIDTH,
                              obs_side_thr: float = 1.5,
                              prefer_wide: bool = False):
    if ranges is None:
        return None
    left = ranges[45:135]
    right = ranges[225:315]
    lv = left[(left > 0.1) & (left < 25.0)]
    rv = right[(right > 0.1) & (right < 25.0)]
    l_min = np.min(lv) if lv.size else np.inf
    r_min = np.min(rv) if rv.size else np.inf
    
    if l_min < obs_side_thr and r_min < obs_side_thr:
        center_width = l_min + r_min
        if center_width >= min_width:
            if not prefer_wide:
                return (0.0, center_width, (PP_LOOK, 0.0))

    pts = []
    for i, d in enumerate(ranges):
        if 0.15 < d < 25.0:
            ang = np.deg2rad(i)
            pts.append([d * np.cos(ang), d * np.sin(ang)])
    if len(pts) < 5:
        return None

    pts = np.asarray(pts)
    lbls = DBSCAN(eps=0.4, min_samples=3).fit_predict(pts)
    clusters = []
    for lbl in set(lbls):
        if lbl == -1:
            continue
        cl = pts[lbls == lbl]
        cx, cy = np.mean(cl[:, 0]), np.mean(cl[:, 1])
        clusters.append((math.atan2(cy, cx), cx, cy))

    if len(clusters) < 2:
        return None

    clusters.sort(key=lambda x: x[0])
    gaps = []
    
    for i in range(len(clusters) - 1):
        a1, x1, y1 = clusters[i]
        a2, x2, y2 = clusters[i + 1]
        gap_w = math.hypot(x2 - x1, y2 - y1)
        if gap_w >= min_width:
            gaps.append(((a1 + a2) / 2, gap_w, ((x1 + x2) / 2, (y1 + y2) / 2)))
    
    if not gaps:
        return None
    
    if prefer_wide:
        return max(gaps, key=lambda x: x[1])
    else:
        return max(gaps, key=lambda x: x[1])

# ──────────────────────────────────────────
# 전역 변수
# ──────────────────────────────────────────
image = np.empty(shape=[0])
ranges = None
motor = None
motor_msg = XycarMotor()
bridge = CvBridge()

# 시스템 초기화
aco_params = ACOParams(
    num_ants=12,
    alpha=1.9,
    beta=2.5,
    rho=0.2,
    Q=120.0,
    max_iterations=30,
    grid_size=0.15,
    safety_distance=0.45
)
lidar_aco = LidarACOAvoidance(aco_params)

# 지능형 후진 시스템 초기화
backup_system = IntelligentBackupSystem()

# ──────────────────────────────────────────
# ROS 콜백
# ──────────────────────────────────────────
def cam_cb(msg):
    global image
    image = bridge.imgmsg_to_cv2(msg, 'bgr8')

def lidar_cb(msg):
    global ranges
    ranges = np.asarray(msg.ranges[:360], np.float32)

# ──────────────────────────────────────────
# 모터 제어
# ──────────────────────────────────────────
def drive(angle: float, speed: float):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

# ──────────────────────────────────────────
# 유틸리티 함수들
# ──────────────────────────────────────────
def get_line_params(x1, y1, x2, y2):
    if x2 == x1:
        return float('inf'), x1
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    return m, b

def print_mem():
    rss = psutil.Process().memory_info().rss / (1024 * 1024)
    rospy.loginfo(f"[MEM] RSS {rss:.1f} MB")

# ──────────────────────────────────────────
# 메인 함수
# ──────────────────────────────────────────
def start():
    global motor, image, ranges
    rospy.init_node('Intelligent_Backup_Driver')
    rospy.Subscriber('/usb_cam/image_raw', Image, cam_cb, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, lidar_cb, queue_size=1)
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)

    rospy.wait_for_message('/usb_cam/image_raw', Image)
    rospy.wait_for_message('/scan', LaserScan)

    frame = 0
    prev_angle = 0.0
    heading = 0.0
    ROI_RATIO = 0.5
    P_GAIN = 0.40
    MAX_DELTA = 120.0

    aco_mode = False
    trans_cnt = 0
    TRANSITION_FRAMES = 10

    OBSTACLE_THR = 3.0
    SAFE_THR = 5.0

    # === S 코스 감지 상태 ===
    s_course_mode = False
    s_course_counter = 0

    # === 지능형 백업 상태 ===
    backup_mode = False
    backup_cnt = 0

    gc_intv = 300

    lower_y = np.array([15, 80, 80], np.uint8)
    upper_y = np.array([35, 255, 255], np.uint8)

    gray = blur = white = ymask = cmask = hsv = None
    disp = None

    rospy.loginfo("▶ 지능형 후진 시스템 시작")

    while not rospy.is_shutdown():
        try:
            if image.size == 0 or ranges is None:
                continue

            h, w, _ = image.shape
            roi_row = int(h * ROI_RATIO)

            # ───── 1) TopView 업데이트 및 지능형 분석 ─────
            topview_img = backup_system.update_topview(ranges)
            cv2.imshow("Intelligent Backup TopView", topview_img)

            # ───── 2) S 코스 감지 ─────
            if detect_s_course(ranges):
                s_course_counter = min(s_course_counter + 1, S_COURSE_DETECT_FRAMES * 2)
            else:
                s_course_counter = max(s_course_counter - 1, 0)
            
            if s_course_counter >= S_COURSE_DETECT_FRAMES and not s_course_mode:
                s_course_mode = True
                rospy.logwarn("[S-COURSE] S 코스 감지됨")
            elif s_course_counter == 0 and s_course_mode:
                s_course_mode = False
                rospy.loginfo("[S-COURSE] S 코스 종료")


            # ───── 3) 전방 장애물 감지 ─────
            obst = False
            min_d = float('inf')
            f_idx = list(range(0, 60)) + list(range(300, 360))
            f = ranges[f_idx]
            v = f[(f > 0.1) & (f < 25.0)]
            if v.size:
                min_d = float(np.min(v))
                obst = min_d < OBSTACLE_THR

            # ───── 4) 지능형 백업 모드 관리 ─────
            if not backup_mode and backup_system.backup_needed:
                backup_mode = True
                backup_cnt = 0
                rospy.logwarn("[INTELLIGENT BACKUP] 막다른 상황 감지 - 지능형 후진 시작")
                
            if backup_mode:
                backup_cnt += 1
                # 후진 완료 조건: 충분한 공간 확보 또는 최대 프레임 도달
                if not backup_system.backup_needed or backup_cnt >= BACKUP_FRAMES:
                    backup_mode = False
                    rospy.loginfo("[INTELLIGENT BACKUP] 후진 완료")

            # ───── 5) 모드 전환 (ACO/LANE) ─────
            if not backup_mode:
                if obst and not aco_mode:
                    aco_mode = True
                    trans_cnt = 0
                    rospy.logwarn(f"[MODE] ACO 모드 진입 (거리={min_d:.2f}m)")
                elif not obst and aco_mode:
                    trans_cnt += 1
                    transition_frames = TRANSITION_FRAMES // (2 if s_course_mode else 1)
                    if trans_cnt >= transition_frames:
                        aco_mode = False
                        trans_cnt = 0
                        rospy.loginfo(f"[MODE] 차선 모드 복귀 (거리={min_d:.2f}m)")

            # ===== 6) 주행 제어 =====
            if backup_mode:
                # ───── 지능형 백업 모드 ─────
                final_angle = backup_system.backup_direction
                speed = BACKUP_SPEED
                
                if disp is None or disp.shape != image.shape:
                    disp = np.empty_like(image)
                np.copyto(disp, image)
                
                # 상태 표시
                status_text = f"INTELLIGENT BACKUP (Angle: {final_angle:.1f}°)"
                if s_course_mode:
                    status_text += " [S-COURSE]"
                cv2.putText(disp, status_text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                cv2.putText(disp, f"Confidence: {backup_system.backup_confidence:.2f}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            elif aco_mode:
                # ───── ACO 모드 ─────
                lidar_aco.update_lidar_data(ranges, s_course_mode=s_course_mode)
                aco_ang, ok = lidar_aco.aco_path_planning(heading, min_d, s_course_mode)
                
                if ok:
                    final_angle = aco_ang
                    speed = 18 if s_course_mode else 15
                else:
                    left = ranges[45:135]
                    right = ranges[225:315]
                    l = safe_mean(left[(left > 0.1) & (left < 25.0)])
                    r = safe_mean(right[(right > 0.1) & (right < 25.0)])
                    
                    if np.isnan(l) and np.isnan(r):
                        final_angle = 0.0
                    elif np.isnan(l):
                        final_angle = -S_COURSE_WIDE_TURN_ANGLE if s_course_mode else -25
                    elif np.isnan(r):
                        final_angle = S_COURSE_WIDE_TURN_ANGLE if s_course_mode else 25
                    else:
                        angle = S_COURSE_WIDE_TURN_ANGLE if s_course_mode else 25
                        final_angle = -angle if l > r else angle
                    speed = 12 if s_course_mode else 10

                # GAP 탐색 결과 반영
                gap_res = find_widest_or_center_gap(ranges, prefer_wide=s_course_mode)
                if gap_res is not None:
                    g_ang, g_w, g_mid = gap_res
                    pp_ang = pure_pursuit_angle(g_mid)
                    
                    gap_weight = 0.8 if s_course_mode else 0.7
                    aco_weight = 1.0 - gap_weight
                    final_angle = gap_weight * pp_ang + aco_weight * final_angle

                if disp is None or disp.shape != image.shape:
                    disp = np.empty_like(image)
                np.copyto(disp, image)
                status_text = "ACO MODE"
                if s_course_mode:
                    status_text += " [S-COURSE]"
                cv2.putText(disp, status_text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(disp, f"Min Dist: {min_d:.2f}m", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            else:
                # ───── 차선 추종 모드 ─────
                if disp is None or disp.shape != image.shape:
                    disp = np.empty_like(image)
                np.copyto(disp, image)
                cv2.rectangle(disp, (0, roi_row), (w-1, h-1), (0, 255, 0), 2)

                roi = image[roi_row:, :]
                rh, rw = roi.shape[:2]
                if gray is None or gray.shape != (rh, rw):
                    gray = np.empty((rh, rw), np.uint8)
                    blur = np.empty_like(gray)
                    white = np.empty_like(gray)
                    ymask = np.empty_like(gray)
                    cmask = np.empty_like(gray)
                    hsv = np.empty((rh, rw, 3), np.uint8)

                cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY, dst=gray)
                cv2.GaussianBlur(gray, (5, 5), 0, dst=blur)
                cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY, dst=white)
                cv2.cvtColor(roi, cv2.COLOR_BGR2HSV, dst=hsv)
                cv2.inRange(hsv, lower_y, upper_y, dst=ymask)
                cv2.bitwise_or(white, ymask, dst=cmask)
                edges = cv2.Canny(cmask, 75, 225)

                lines = cv2.HoughLinesP(edges, 1, np.pi/180, 30,
                                        minLineLength=20, maxLineGap=10)
                left_l, right_l = [], []
                if lines is not None:
                    for ln in lines:
                        x1, y1, x2, y2 = ln[0]
                        y1 += roi_row; y2 += roi_row
                        m, _ = get_line_params(x1, y1, x2, y2)
                        if abs(m) < 0.1 or math.isinf(m):
                            continue
                        (left_l if m < 0 else right_l).append(ln[0])

                left_x = right_x = -1
                if left_l:
                    xs = [x for ln in left_l for x in (ln[0], ln[2])]
                    left_x = int(np.mean(xs))
                if right_l:
                    xs = [x for ln in right_l for x in (ln[0], ln[2])]
                    right_x = int(np.mean(xs))

                target = w // 2
                if left_x != -1 and right_x != -1:
                    target = (left_x + right_x) // 2
                elif left_x != -1:
                    target = left_x + w // 4
                elif right_x != -1:
                    target = right_x - w // 4

                cv2.circle(disp, (target, h-10), 10, (0, 255, 255), -1)
                error = target - w//2
                final_angle = float(np.clip(error * P_GAIN, -35, 35))

                abs_a = abs(final_angle)
                speed = 40 if abs_a < 5 else 30 if abs_a < 10 else \
                        20 if abs_a < 20 else 15

                cv2.putText(disp, "LANE FOLLOWING", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # ───── 7) 조향 변화 제한 및 최종 처리 ─────
            dyn_lim = get_dynamic_steer_limit(min_d, s_course_mode)
            if abs(final_angle - prev_angle) > MAX_DELTA * (0.033 / 0.02) : # Time-step based smoothing
                 final_angle = prev_angle + np.sign(final_angle - prev_angle) * MAX_DELTA * (0.033/0.02)
            prev_angle = final_angle
            heading += math.radians(final_angle * 0.1)

            # 긴급 상황 속도 제한
            if obst and min_d < 1.5 and not backup_mode:
                speed = min(speed, 8 if s_course_mode else 5)

            # ───── 8) 디스플레이 ─────
            cv2.imshow("Camera", disp)

            # ───── 9) 모터 제어 ─────
            final_angle = float(np.clip(final_angle, -dyn_lim, dyn_lim))
            speed = float(np.clip(speed, -20, 50))
            drive(final_angle, speed)

            # ───── 10) 시스템 관리 ─────
            frame += 1
            if frame % gc_intv == 0:
                gc.collect()
            if frame % 200 == 0:
                print_mem()

            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(f"[MAIN] 오류: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            drive(0, 0)
            time.sleep(0.1)
            continue

    rospy.loginfo("▶ 프로그램 종료")
    drive(0, 0)
    cv2.destroyAllWindows()

# ──────────────────────────────────────────
# 진입점
# ──────────────────────────────────────────
if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()