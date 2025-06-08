#!/usr/bin/env python
# -*- coding: utf-8 -*-
# =============================================
# 본 프로그램은 2025 제8회 국민대 자율주행 경진대회에서
# 예선과제를 수행하기 위한 파일입니다.
# 예선과제 수행 용도로만 사용가능하며 외부유출은 금지됩니다.
# =============================================
import numpy as np
import cv2
import rospy
import time
import os
import math
import json
from sensor_msgs.msg import Image, LaserScan
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
import matplotlib.pyplot as plt # 시각화에 필요시 사용

# --- autonomous_planner 모듈 임포트 ---
# example.py와 같은 디렉토리에 autonomous_planner 폴더가 있다고 가정
try:
    from autonomous_planner.planner import DynamicPathPlanner
    from autonomous_planner.utils.data_structures import Waypoint, VehicleState, Obstacle, MovingObstacle, PathCandidate
    from autonomous_planner.config import PLANNING_DT # 경로 계획 주기
except ImportError as e:
    rospy.logerr(f"Failed to import autonomous_planner: {e}")
    rospy.logerr("Ensure 'autonomous_planner' directory is in the same directory as example.py or in PYTHONPATH.")
    sys.exit(1)

# =============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
# =============================================
image_raw = np.empty(shape=[0])  # 카메라 원본 이미지를 담을 변수
lidar_ranges = None  # 라이다 데이터를 담을 변수
motor_publisher = None  # 모터 퍼블리셔
current_motor_msg = XycarMotor()  # 현재 모터 제어 메시지 (차량 상태 업데이트용)
cv_bridge = CvBridge()

# Planner 관련
path_planner = None
current_vehicle_sim_state = VehicleState(position=Waypoint(0.0, 0.0), heading_angle=0.0, speed=0.0)
global_map_waypoints = [] # (Waypoint_left, Waypoint_right) 쌍의 리스트
global_road_edges = []    # 도로 경계선 [[Waypoint,...], [Waypoint,...]]
global_lane_lines = []    # 차선 [[Waypoint,...], ...]

# 차량 제어 및 시뮬레이션 관련 상수
XYCAR_SPEED_TO_MPS_SCALE = 0.1 # 예: Xycar 속도 10 -> 1.0 m/s (튜닝 필요)
XYCAR_ANGLE_TO_RAD_SCALE = math.radians(1.0) # 예: Xycar 각도 1 -> 1도 (라디안으로 변환)
XYCAR_MAX_SPEED = 50 # Xycar 모터 최대 속도값
XYCAR_MIN_SPEED = 0  # Xycar 모터 최소 속도값 (후진은 음수일 수 있음)
XYCAR_MAX_ANGLE_DEG = 40 # Xycar 최대 조향각 (도)

SIM_WHEELBASE = 0.26 # 시뮬레이터 차량 휠베이스 (m) - 가정값, 정확한 값 필요

# 시각화 플래그 (Rviz나 시뮬레이터 자체 뷰어 사용 권장)
ENABLE_MATPLOTLIB_LIDAR_VIZ = False
ENABLE_MATPLOTLIB_PATH_VIZ = False

# Matplotlib 시각화 객체 (선언은 하되, 사용은 플래그에 따라 결정)
if ENABLE_MATPLOTLIB_LIDAR_VIZ:
    fig_lidar_viz, ax_lidar_viz = plt.subplots(figsize=(6, 6))
    ax_lidar_viz.set_xlim(-5, 5) # m 단위
    ax_lidar_viz.set_ylim(-5, 5) # m 단위
    ax_lidar_viz.set_aspect('equal')
    lidar_points_plot, = ax_lidar_viz.plot([], [], 'bo', markersize=2)

if ENABLE_MATPLOTLIB_PATH_VIZ:
    fig_path_viz, ax_path_viz_plot = plt.subplots(figsize=(10, 10))
    line_optimal_path_plot, = ax_path_viz_plot.plot([], [], 'g-', linewidth=2, label="Optimal Path")
    line_center_line_plot, = ax_path_viz_plot.plot([], [], 'k--', alpha=0.7, label="Center Line")
    scatter_vehicle_plot = ax_path_viz_plot.plot([], [], 'ro', markersize=8, label="Vehicle")[0]
    scatter_obstacles_plot = ax_path_viz_plot.plot([], [], 'ms', markersize=6, label="Obstacles")[0]
    ax_path_viz_plot.legend()
    ax_path_viz_plot.set_xlabel("X (m, Global)")
    ax_path_viz_plot.set_ylabel("Y (m, Global)")
    ax_path_viz_plot.grid(True)
    # ax_path_viz_plot.set_aspect('equal') # 필요시 주석 해제

# =============================================
# 콜백함수
# =============================================
def image_callback(data):
    global image_raw
    image_raw = cv_bridge.imgmsg_to_cv2(data, "bgr8")

def lidar_callback(data):
    global lidar_ranges
    # 시뮬레이터 라이다 스펙에 따라 data.ranges 전체 또는 일부 사용
    # example.py에서는 [0:360] 사용. 360개 포인트로 가정.
    lidar_ranges = np.array(data.ranges[0:360])

# =============================================
# 유틸리티 함수
# =============================================
def drive_vehicle(angle_deg, speed_xycar_unit):
    global motor_publisher, current_motor_msg
    
    # 제어값 제한
    angle_deg_clipped = np.clip(angle_deg, -XYCAR_MAX_ANGLE_DEG, XYCAR_MAX_ANGLE_DEG)
    speed_xycar_unit_clipped = np.clip(speed_xycar_unit, XYCAR_MIN_SPEED, XYCAR_MAX_SPEED)

    current_motor_msg.angle = float(angle_deg_clipped)
    current_motor_msg.speed = float(speed_xycar_unit_clipped)
    
    if motor_publisher is not None:
        motor_publisher.publish(current_motor_msg)

def load_map_from_file(filepath="data/waypoints_example.json"):
    global global_map_waypoints, global_road_edges, global_lane_lines
    base_dir = os.path.dirname(os.path.abspath(__file__))
    full_filepath = os.path.join(base_dir, filepath)

    # data 폴더 및 예제 파일 생성 (경로 계획 모듈의 main.py에서 가져옴)
    data_dir_path = os.path.join(base_dir, "data")
    if not os.path.exists(data_dir_path):
        os.makedirs(data_dir_path)
    if not os.path.exists(full_filepath):
        default_wps_data_json = []
        # 50m 직선 도로, 2.5m 간격 웨이포인트, 도로 폭 3m
        for x_coord in np.arange(0, 50.5, 2.5): # x_coord from 0 to 50.0
            # [[lx,ly],[rx,ry]] 형식으로 저장
            default_wps_data_json.append([[float(x_coord), 1.5], [float(x_coord), -1.5]])
        with open(full_filepath, "w") as f:
            json.dump(default_wps_data_json, f)
        rospy.loginfo(f"Created default waypoint file at {full_filepath}")

    try:
        with open(full_filepath, 'r') as f:
            # JSON 파일이 [[ [lx,ly], [rx,ry] ], ... ] 형태라고 가정
            loaded_json_data = json.load(f)
            global_map_waypoints = [
                (Waypoint(pair[0][0], pair[0][1]), Waypoint(pair[1][0], pair[1][1]))
                for pair in loaded_json_data
            ]
        rospy.loginfo(f"Loaded {len(global_map_waypoints)} waypoint pairs from {full_filepath}")
    except Exception as e:
        rospy.logerr(f"Failed to load waypoints from {full_filepath}: {e}. Using empty map.")
        global_map_waypoints = []

    # TODO: global_road_edges, global_lane_lines도 유사하게 로드 또는 생성
    if global_map_waypoints:
        # 예시: 도로 가장자리를 웨이포인트 바깥쪽 0.5m로 설정
        left_edge = [Waypoint(wp_pair[0].x, wp_pair[0].y + 0.5) for wp_pair in global_map_waypoints]
        right_edge = [Waypoint(wp_pair[1].x, wp_pair[1].y - 0.5) for wp_pair in global_map_waypoints]
        global_road_edges = [left_edge, right_edge]
    else:
        global_road_edges = []

def update_sim_vehicle_state(dt_sec):
    """ 시뮬레이터 차량 상태를 간단히 업데이트 (제어 입력 기반) """
    global current_vehicle_sim_state, current_motor_msg

    current_speed_mps = current_vehicle_sim_state.speed # 현재 속도(m/s) 사용
    steer_angle_rad = math.radians(current_motor_msg.angle) # 현재 조향각(rad)

    # 차량 모델 (자전거 모델)
    # 실제 시뮬레이터는 이 계산을 내부적으로 수행하고 상태를 알려줄 것임.
    # 여기서는 시뮬레이터가 상태를 알려주지 않는다고 가정하고 직접 계산.
    current_heading_rad = current_vehicle_sim_state.heading_angle

    # 헤딩 변화율: omega = v * tan(steer_angle) / L
    if abs(SIM_WHEELBASE) > 1e-3:
        turning_rate_rad_per_sec = (current_speed_mps / SIM_WHEELBASE) * math.tan(steer_angle_rad)
    else:
        turning_rate_rad_per_sec = 0.0
    
    delta_heading = turning_rate_rad_per_sec * dt_sec
    
    # 이동 거리 계산 (중간 각 사용 근사)
    avg_heading = current_heading_rad + delta_heading / 2.0
    dx = current_speed_mps * math.cos(avg_heading) * dt_sec
    dy = current_speed_mps * math.sin(avg_heading) * dt_sec

    current_vehicle_sim_state.position.x += dx
    current_vehicle_sim_state.position.y += dy
    current_vehicle_sim_state.heading_angle = (current_heading_rad + delta_heading) % (2 * math.pi)
    
    # 속도 업데이트: 플래너의 목표 속도를 따라간다고 가정 (실제론 가속도 제어)
    # 여기서는 current_motor_msg.speed (Xycar 단위)를 m/s로 변환하여 현재 속도로 설정
    current_vehicle_sim_state.speed = current_motor_msg.speed * XYCAR_SPEED_TO_MPS_SCALE
    current_vehicle_sim_state.speed = np.clip(current_vehicle_sim_state.speed, 0, 15.0) # 최대 15m/s 가정


def process_lidar_to_obstacles(raw_lidar_ranges, vehicle_current_state: VehicleState) -> Tuple[List[Obstacle], List[MovingObstacle]]:
    static_obs_list = []
    moving_obs_list = [] # 동적 장애물은 이번 구현에서 제외

    if raw_lidar_ranges is None or len(raw_lidar_ranges) == 0:
        return static_obs_list, moving_obs_list

    num_pts = len(raw_lidar_ranges)
    
    # example.py의 라이다 각도 처리: np.linspace(0,2*np.pi, len(ranges)) + np.pi/2
    # 이는 0번째 인덱스가 +y축(차량 우측), 반시계방향으로 각도 증가, 90번째 인덱스가 -x축(차량 후방)을 의미.
    # 즉, 차량 전방은 270번째 인덱스 (3*pi/2 또는 -pi/2) 근처가 됨. -> 이 해석이 맞는지 시뮬레이터 스펙 확인 필요!
    # 여기서는 example.py의 각도 계산을 그대로 사용.
    angles_rad = np.linspace(0, 2 * np.pi, num_pts, endpoint=False) + np.pi/2.0

    veh_x, veh_y, veh_h_rad = vehicle_current_state.position.x, vehicle_current_state.position.y, vehicle_current_state.heading_angle

    for i, r_m in enumerate(raw_lidar_ranges):
        if 0.1 < r_m < 4.0:  # 유효 거리 범위 (m)
            # 라이다 포인트의 차량 로컬 좌표 (example.py 방식)
            # example.py의 x,y는 matplotlib plot 기준이었으므로, 센서 좌표계로 변환 필요.
            # angles_rad[i]는 라이다 센서 좌표계에서의 각도.
            # 센서 좌표계: x축이 라이다 0도 방향, y축이 라이다 90도 방향.
            # 차량 좌표계: x축이 차량 전방, y축이 차량 좌측.
            # 라이다가 차량 전방을 향해 설치되었다고 가정하고, 라이다 0도가 차량 전방과 일치한다고 가정하면,
            # local_x = r_m * math.cos(angles_rad[i])
            # local_y = r_m * math.sin(angles_rad[i])
            # 하지만 example.py의 lidar_points.set_data(x, y) 는
            # x = ranges * np.cos(angles) , y = ranges * np.sin(angles) 였음.
            # 이 (x,y)는 matplotlib의 x,y축. 이를 차량 로컬 (전방 x, 좌측 y)로 변환해야 함.
            # example.py의 angles는 matplotlib plot 기준 각도임.
            # 우리는 차량 로컬 (전방 x, 좌측 y)이 필요.
            # 만약 라이다 0도가 전방이라면:
            # local_x_veh = r_m * math.cos(angles_rad[i]) # 전방
            # local_y_veh = r_m * math.sin(angles_rad[i]) # 좌측
            # 시뮬레이터의 라이다 좌표계를 명확히 알아야 함.
            # 가정: lidar_ranges의 0번째 인덱스가 차량 전방, 각도는 반시계 방향으로 증가.
            angle_from_front_rad = np.linspace(0, 2 * np.pi, num_pts, endpoint=False)[i] # 0~359도
            
            local_x_veh = r_m * math.cos(angle_from_front_rad) # 차량 전방이 x+
            local_y_veh = r_m * math.sin(angle_from_front_rad) # 차량 좌측이 y+

            # 전역 좌표로 변환
            global_x_obs = veh_x + local_x_veh * math.cos(veh_h_rad) - local_y_veh * math.sin(veh_h_rad)
            global_y_obs = veh_y + local_x_veh * math.sin(veh_h_rad) + local_y_veh * math.cos(veh_h_rad)
            
            # 전방 일정 각도 내의 장애물만 고려 (예: +/- 45도)
            if abs(angle_from_front_rad) < math.radians(45) or \
               abs(angle_from_front_rad - 2*math.pi) < math.radians(45): # -45 to +45 deg
                # 장애물로 추가 (간단히 모든 유효 포인트를 장애물로 처리, 실제로는 클러스터링 필요)
                if i % 10 == 0: # 너무 많은 장애물을 피하기 위해 샘플링
                    static_obs_list.append(Obstacle(position=Waypoint(global_x_obs, global_y_obs), radius=0.1)) # 반경 0.1m

    return static_obs_list, moving_obs_list


def calculate_xycar_controls(opt_path: PathCandidate, current_state: VehicleState, target_spd_mps: float, target_acc_mps2: float) -> Tuple[float, float]:
    final_steer_deg = 0.0
    
    if opt_path and len(opt_path.points_cartesian) > 1:
        # 경로 추종: Pure Pursuit 간략화 (첫번째 경로 세그먼트의 방향 또는 lookahead 점 사용)
        # 경로의 두번째 점을 lookahead 점으로 사용 (첫번째 점은 현재 위치와 거의 같을 수 있음)
        lookahead_idx = min(1, len(opt_path.points_cartesian) - 1)
        target_wp = opt_path.points_cartesian[lookahead_idx]

        dx_global = target_wp.x - current_state.position.x
        dy_global = target_wp.y - current_state.position.y

        # 목표점까지의 전역 각도
        angle_to_target_global_rad = math.atan2(dy_global, dx_global)
        
        # 차량 헤딩과의 각도차 (로컬 좌표계에서의 목표점 각도)
        alpha_rad = (angle_to_target_global_rad - current_state.heading_angle) % (2 * math.pi)
        if alpha_rad > math.pi: alpha_rad -= 2 * math.pi # -pi ~ pi 범위로 정규화

        # Pure Pursuit 유사: steer_angle = atan(2*L*sin(alpha)/ld)
        # ld (lookahead distance)
        ld_mps = math.sqrt(dx_global**2 + dy_global**2)
        
        if ld_mps > 0.1: # lookahead 거리가 충분할 때만 조향각 계산
            # target_steer_rad = math.atan2(2.0 * SIM_WHEELBASE * math.sin(alpha_rad), ld_mps)
            # 더 간단히: alpha_rad를 직접 조향각으로 사용 (P제어기 형태)
            target_steer_rad = alpha_rad * 1.5 # P_gain, 튜닝 필요
        else:
            target_steer_rad = 0.0

        final_steer_deg = math.degrees(target_steer_rad)
        final_steer_deg = np.clip(final_steer_deg, -XYCAR_MAX_ANGLE_DEG, XYCAR_MAX_ANGLE_DEG)

    # 속도 제어: 목표 속도(m/s)를 Xycar 단위로 변환
    # 실제로는 현재 속도와 목표 가속도를 고려하여 다음 스텝의 목표 속도를 결정해야 함.
    # planner가 반환하는 target_spd_mps는 경로 전체에 대한 권장 속도일 수 있음.
    # 여기서는 planner가 반환한 target_spd_mps를 직접 사용.
    final_speed_xycar = target_spd_mps / XYCAR_SPEED_TO_MPS_SCALE
    final_speed_xycar = np.clip(final_speed_xycar, XYCAR_MIN_SPEED, XYCAR_MAX_SPEED)
    
    return final_steer_deg, final_speed_xycar

# =============================================
# 시각화 함수 (Matplotlib)
# =============================================
def viz_lidar_matplotlib(raw_ranges):
    global lidar_points_plot, fig_lidar_viz, ax_lidar_viz
    if raw_ranges is not None and plt.fignum_exists(fig_lidar_viz.number):
        num_pts = len(raw_ranges)
        # example.py의 라이다 각도 변환 사용
        angles_plot = np.linspace(0, 2 * np.pi, num_pts, endpoint=False) + np.pi/2.0
        
        valid_idx = np.isfinite(raw_ranges) & (raw_ranges > 0.01) # 유효한 데이터만
        plot_ranges = raw_ranges[valid_idx]
        plot_angles = angles_plot[valid_idx]

        x_coords = plot_ranges * np.cos(plot_angles)
        y_coords = plot_ranges * np.sin(plot_angles)

        lidar_points_plot.set_data(x_coords, y_coords)
        # ax_lidar_viz.set_xlim(min(x_coords)-0.5, max(x_coords)+0.5) # 동적 스케일링
        # ax_lidar_viz.set_ylim(min(y_coords)-0.5, max(y_coords)+0.5)
        try:
            fig_lidar_viz.canvas.draw_idle()
            plt.pause(0.001)
        except Exception: # GUI 관련 오류 무시
            pass

def viz_path_matplotlib(opt_path_obj, cl_obj, veh_state_obj, stat_obs_list):
    global line_optimal_path_plot, line_center_line_plot, scatter_vehicle_plot, scatter_obstacles_plot, fig_path_viz, ax_path_viz_plot
    
    if not plt.fignum_exists(fig_path_viz.number): return

    # Optimal Path
    if opt_path_obj and opt_path_obj.points_cartesian:
        path_x = [p.x for p in opt_path_obj.points_cartesian]
        path_y = [p.y for p in opt_path_obj.points_cartesian]
        line_optimal_path_plot.set_data(path_x, path_y)
    else:
        line_optimal_path_plot.set_data([], [])

    # Center Line
    if cl_obj and cl_obj.segments:
        s_vals = np.linspace(0, cl_obj.segments[-1].start_s_global + cl_obj.segments[-1].segment_arc_length, 100)
        cl_x = [cl_obj.get_cartesian_coords(s).x for s in s_vals]
        cl_y = [cl_obj.get_cartesian_coords(s).y for s in s_vals]
        line_center_line_plot.set_data(cl_x, cl_y)
    else:
        line_center_line_plot.set_data([], [])
        
    # Vehicle
    scatter_vehicle_plot.set_data([veh_state_obj.position.x], [veh_state_obj.position.y])
    
    # Obstacles
    if stat_obs_list:
        obs_x = [obs.position.x for obs in stat_obs_list]
        obs_y = [obs.position.y for obs in stat_obs_list]
        scatter_obstacles_plot.set_data(obs_x, obs_y)
    else:
        scatter_obstacles_plot.set_data([], [])
    
    # Dynamic plot limits
    all_x_coords = []
    all_y_coords = []
    for artist in [line_optimal_path_plot, line_center_line_plot, scatter_vehicle_plot, scatter_obstacles_plot]:
        all_x_coords.extend(artist.get_xdata())
        all_y_coords.extend(artist.get_ydata())

    if all_x_coords and all_y_coords:
        min_x, max_x = min(all_x_coords), max(all_x_coords)
        min_y, max_y = min(all_y_coords), max(all_y_coords)
        margin_x = (max_x - min_x) * 0.15 + 2.0 # 15% + 2m margin
        margin_y = (max_y - min_y) * 0.15 + 2.0
        ax_path_viz_plot.set_xlim(min_x - margin_x, max_x + margin_x)
        ax_path_viz_plot.set_ylim(min_y - margin_y, max_y + margin_y)
    else: # Fallback if no data
        ax_path_viz_plot.set_xlim(veh_state_obj.position.x - 15, veh_state_obj.position.x + 15)
        ax_path_viz_plot.set_ylim(veh_state_obj.position.y - 15, veh_state_obj.position.y + 15)

    try:
        fig_path_viz.canvas.draw_idle()
        plt.pause(0.001)
    except Exception: # GUI 관련 오류 무시
        pass

# =============================================
# 메인 실행 함수
# =============================================
def main_loop():
    global motor_publisher, image_raw, lidar_ranges, path_planner
    global current_vehicle_sim_state, global_map_waypoints, global_road_edges, global_lane_lines

    rospy.loginfo("Initializing ROS Node and Path Planner...")
    rospy.init_node('DynamicPlannerNode_Sim')
    
    # ROS Subscribers & Publisher
    rospy.Subscriber("/usb_cam/image_raw/", Image, image_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    motor_publisher = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)

    # Planner 초기화 및 맵 로드
    path_planner = DynamicPathPlanner()
    load_map_from_file("data/waypoints_example.json") # data 폴더 내 파일명

    # 초기 차량 상태 설정 (시뮬레이터 시작 위치에 맞게 조정 필요)
    if global_map_waypoints:
        first_wp_l = global_map_waypoints[0][0]
        first_wp_r = global_map_waypoints[0][1]
        start_x = (first_wp_l.x + first_wp_r.x) / 2.0
        start_y = (first_wp_l.y + first_wp_r.y) / 2.0
        # 도로 방향에 따른 초기 헤딩 (두번째 웨이포인트 중심점 방향)
        if len(global_map_waypoints) > 1:
            second_wp_l = global_map_waypoints[1][0]
            second_wp_r = global_map_waypoints[1][1]
            next_x = (second_wp_l.x + second_wp_r.x) / 2.0
            next_y = (second_wp_l.y + second_wp_r.y) / 2.0
            start_heading_rad = math.atan2(next_y - start_y, next_x - start_x)
        else:
            start_heading_rad = 0.0 # 기본값: x축 방향
        current_vehicle_sim_state = VehicleState(position=Waypoint(start_x, start_y), heading_angle=start_heading_rad, speed=0.0)
    else:
        rospy.logwarn("No global waypoints loaded. Using default initial state.")
        current_vehicle_sim_state = VehicleState(position=Waypoint(0.0, 0.0), heading_angle=0.0, speed=0.0)

    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    rospy.loginfo("Camera feed active.")
    rospy.wait_for_message("/scan", LaserScan)
    rospy.loginfo("Lidar feed active.")

    if ENABLE_MATPLOTLIB_LIDAR_VIZ or ENABLE_MATPLOTLIB_PATH_VIZ:
        plt.ion()
        plt.show()
        rospy.loginfo("Matplotlib visualization enabled and figure shown.")

    rospy.loginfo("==========================================")
    rospy.loginfo("    S T A R T I N G   D R I V I N G      ")
    rospy.loginfo("==========================================")

    loop_rate = rospy.Rate(1.0 / PLANNING_DT) # 예: 10Hz
    last_plan_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_loop_time = rospy.Time.now()
        elapsed_dt_sec = (current_loop_time - last_plan_time).to_sec()
        if elapsed_dt_sec < PLANNING_DT * 0.5 and elapsed_dt_sec > 0: # 너무 짧은 간격의 루프 방지
            time.sleep(PLANNING_DT * 0.5 - elapsed_dt_sec) # 최소 간격 유지
            current_loop_time = rospy.Time.now() # 시간 재측정
            elapsed_dt_sec = (current_loop_time - last_plan_time).to_sec()

        if elapsed_dt_sec == 0: elapsed_dt_sec = PLANNING_DT # 첫 루프 또는 시간 오류 방지
        last_plan_time = current_loop_time

        # 1. 차량 상태 업데이트 (시뮬레이터로부터 받거나, 제어 입력 기반으로 추정)
        # 여기서는 이전 제어 입력 기반으로 차량 상태 추정
        update_sim_vehicle_state(elapsed_dt_sec)
        rospy.logdebug(f"Sim Vehicle State: x={current_vehicle_sim_state.position.x:.2f}, y={current_vehicle_sim_state.position.y:.2f}, "
                      f"h={math.degrees(current_vehicle_sim_state.heading_angle):.1f}, v={current_vehicle_sim_state.speed:.2f}")

        # 2. 장애물 인식
        static_obstacles_detected, moving_obstacles_detected = process_lidar_to_obstacles(lidar_ranges, current_vehicle_sim_state)
        rospy.logdebug(f"Detected {len(static_obstacles_detected)} static obstacles.")

        # 3. 경로 계획
        rospy.logdebug("Planning path...")
        planned_path, target_acceleration, target_speed_mps = path_planner.plan_path(
            global_map_waypoints,
            current_vehicle_sim_state,
            static_obstacles_detected,
            moving_obstacles_detected, # 현재는 항상 비어있음
            global_road_edges,
            global_lane_lines
        )

        # 4. 제어 입력 계산 및 차량 제어
        if planned_path:
            rospy.logdebug(f"Optimal path found. Cost: {planned_path.total_cost:.3f}")
            # 플래너가 계산한 target_speed_mps와 target_acceleration을 이용하여 차량 제어
            # current_vehicle_sim_state.speed는 다음 스텝의 예측 속도로 업데이트 될 것임.
            # target_speed_mps는 경로 전체에 대한 권장 최고 속도일 수 있음.
            # 실제 차량에 적용할 속도는 target_acceleration을 현재 속도에 적용한 결과여야 함.
            next_step_speed_mps = current_vehicle_sim_state.speed + target_acceleration * elapsed_dt_sec
            next_step_speed_mps = np.clip(next_step_speed_mps, 0, target_speed_mps) # 목표 최고 속도 이내로 제한

            control_steer_deg, control_speed_xycar = calculate_xycar_controls(
                planned_path, current_vehicle_sim_state, next_step_speed_mps, target_acceleration
            )
            rospy.loginfo(f"To Motor -> Angle: {control_steer_deg:.1f} deg, Speed: {control_speed_xycar:.1f} (target_v_mps={next_step_speed_mps:.2f}, target_a_mps2={target_acceleration:.2f})")
        else:
            rospy.logwarn("Path planning failed. Applying emergency stop.")
            control_steer_deg = 0.0
            control_speed_xycar = 0.0 # 정지
            # 현재 속도도 0으로 강제
            current_vehicle_sim_state.speed = 0.0


        drive_vehicle(control_steer_deg, control_speed_xycar)

        # (Optional) 카메라 이미지 처리 및 표시
        if image_raw.shape[0] > 0:
             # gray = cv2.cvtColor(image_raw, cv2.COLOR_BGR2GRAY)
             # cv2.imshow("Raw Camera Image", image_raw)
             # cv2.imshow("Grayscale Image", gray)
             pass # 현재는 사용 안함
        # cv2.waitKey(1) # ROS 루프에서는 cv2.waitKey 사용시 문제 발생 가능성

        # (Optional) Matplotlib 시각화
        if ENABLE_MATPLOTLIB_LIDAR_VIZ:
            viz_lidar_matplotlib(lidar_ranges)
        if ENABLE_MATPLOTLIB_PATH_VIZ and path_planner.center_line_cache:
            viz_path_matplotlib(planned_path, path_planner.center_line_cache, current_vehicle_sim_state, static_obstacles_detected)
            
        loop_rate.sleep()

    # 루프 종료 시 처리
    if ENABLE_MATPLOTLIB_LIDAR_VIZ or ENABLE_MATPLOTLIB_PATH_VIZ:
        plt.ioff()
        plt.show() # 마지막 프레임 유지
    # cv2.destroyAllWindows() # cv2.imshow 사용 시 필요

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted.")
    except Exception as e:
        rospy.logerr(f"Unhandled exception in main_loop: {e}", exc_info=True)
    finally:
        # 프로그램 종료 시 모터 정지 명령
        drive_vehicle(0.0, 0.0)
        rospy.loginfo("Program terminated. Motors stopped.")