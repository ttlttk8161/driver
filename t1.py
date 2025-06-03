#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor # 사용자 환경에 맞게 확인
from cv_bridge import CvBridge

# --- 최적화 및 디버깅 플래그 ---
ENABLE_DEBUG_VISUALIZATION = True # imshow 자동으로 켜지도록 True로 설정
PROFILE_PERFORMANCE = False      # True로 설정 시 간단한 성능 측정

# --- Global Variables for ROS ---
bridge = CvBridge()
current_image_cv = None
motor_control_publisher = None
roi_y_start_abs = 0 # main_image_processing_pipeline에서 설정됨

# --- Global State Variables for Lane Detection ---
previous_left_polynomial_fit = None
previous_right_polynomial_fit = None
previous_vehicle_offset_meters = 0.0
last_time_for_derivative_calc = 0.0
accumulated_integral_offset_meters = 0.0

# --- 차선 복귀 모드를 위한 추가 변수들 ---
lane_loss_counter = {"left": 0, "right": 0}
lane_recovery_mode = {"active": False, "type": "none", "confidence": 0.0}
previous_steering_angle = 0.0
lane_width_pixels = 200  # 추정 차선 폭 (픽셀 단위)
confidence_history = {"left": [], "right": []}
max_history_length = 5

# 양쪽 차선 소실 시 직진 유지를 위한 변수들
last_valid_lane_center = None
last_valid_heading = 0.0
emergency_straight_counter = 0

# 차선 변경 방지를 위한 안정성 검증 변수들
stable_detection_counter = {"left": 0, "right": 0, "both": 0}
recovery_exit_threshold = 5  # 복귀 모드 해제를 위한 연속 감지 프레임 수
recovery_enter_threshold = 3  # 복귀 모드 진입을 위한 연속 소실 프레임 수
current_mode_lock_counter = 0  # 모드 변경 방지 잠금 카운터

# --- Camera Calibration Parameters ---
CAMERA_CALIBRATION_DATA_LOADED = True
CAMERA_MATRIX = np.array([[350.0,   0.0, 320.0], [0.0, 350.0, 240.0], [0.0,   0.0,   1.0]], dtype=np.float32)
DIST_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

# --- Configuration Constants ---
# ROI 최소화 시도 (튜닝 필요)
ROI_Y_START_PERCENTAGE = 0.5 # 이전 0.45에서 약간 높여 처리량 감소 시도
ROI_Y_END_PERCENTAGE = 0.8    # 끝 부분을 이미지 하단까지로 설정
IMAGE_RESIZED_WIDTH = 256     # 이 값을 줄이면 더 빨라짐 (예: 256)
IMAGE_RESIZED_HEIGHT = 192    # 이 값을 줄이면 더 빨라짐 (예: 192)

H_THRESHOLD_YELLOW_LANE = (18, 30)
L_THRESHOLD_YELLOW_LANE = (70, 200)
S_THRESHOLD_YELLOW_LANE = (80, 255)

L_THRESHOLD_WHITE_LANE = (170, 255)
S_THRESHOLD_WHITE_LANE = (0, 50)

BGR_EXCLUDE_COLOR_LOWER = np.clip(np.array([158-40, 166-40, 175-40]), 0, 255)
BGR_EXCLUDE_COLOR_UPPER = np.clip(np.array([158+40, 166+40, 175+40]), 0, 255)

SLIDING_WINDOW_COUNT = 7 # 윈도우 수 감소 시도
SLIDING_WINDOW_MARGIN_PERCENTAGE = 0.18 # 마진 약간 감소 시도
MIN_PIXELS_FOR_RECENTERING = 25 # 최소 픽셀 수 감소 시도
POLYNOMIAL_SEARCH_MARGIN_PERCENTAGE = 0.12

KP_LATERAL_OFFSET = 50.0; KI_LATERAL_OFFSET = 1.0; KD_LATERAL_OFFSET = 15.0
KP_HEADING_ERROR = 30.0; K_FF_LANE_CURVATURE = 0.5; MAX_INTEGRAL_TERM_VALUE = 0.3
MAX_STEERING_ANGLE_DEGREES = 35.0

# 속도를 20% 증가 (1.2배)
MAX_SPEED = 100.0; MIN_SPEED_CURVE = 50.0; MIN_SPEED_ANGLE = 40.0; WEAK_LANE_DETECTION_SPEED = 50.0
MIN_PIXEL_SUM_FOR_VALID_MASK = (IMAGE_RESIZED_WIDTH * (ROI_Y_END_PERCENTAGE - ROI_Y_START_PERCENTAGE) * IMAGE_RESIZED_HEIGHT) * 0.0003 # ROI 크기 기반으로 변경

def undistort_image(bgr_image_input):
    # CPU에서만 수행
    if CAMERA_CALIBRATION_DATA_LOADED:
        try: return cv2.undistort(bgr_image_input, CAMERA_MATRIX, DIST_COEFFS, None, CAMERA_MATRIX)
        except: return bgr_image_input
    return bgr_image_input

def preprocess_and_get_roi(bgr_image_input_cpu):
    global roi_y_start_abs
    # Undistortion (CPU)
    undistorted_bgr_cpu = undistort_image(bgr_image_input_cpu)

    # Resize (CPU만 사용)
    resized_bgr_cpu = cv2.resize(undistorted_bgr_cpu, (IMAGE_RESIZED_WIDTH, IMAGE_RESIZED_HEIGHT), interpolation=cv2.INTER_LINEAR)

    # 특정 색상 제거 (CPU에서 수행)
    exclude_mask = cv2.inRange(resized_bgr_cpu, BGR_EXCLUDE_COLOR_LOWER, BGR_EXCLUDE_COLOR_UPPER)
    resized_bgr_cpu[exclude_mask > 0] = [0, 0, 0]

    roi_y_start_abs = int(IMAGE_RESIZED_HEIGHT * ROI_Y_START_PERCENTAGE)
    roi_y_end_abs = int(IMAGE_RESIZED_HEIGHT * ROI_Y_END_PERCENTAGE)
    roi_bgr_cpu = resized_bgr_cpu[roi_y_start_abs:roi_y_end_abs, :]
    return roi_bgr_cpu, resized_bgr_cpu, roi_y_start_abs

def generate_lane_masks_separated(roi_bgr_cpu):
    # CPU에서만 처리
    hls_image_cpu = cv2.cvtColor(roi_bgr_cpu, cv2.COLOR_BGR2HLS)
    h_channel, l_channel, s_channel = hls_image_cpu[:,:,0], hls_image_cpu[:,:,1], hls_image_cpu[:,:,2]

    yellow_mask_cpu = np.zeros_like(h_channel, dtype=np.uint8)
    yellow_cond = ((h_channel >= H_THRESHOLD_YELLOW_LANE[0]) & (h_channel <= H_THRESHOLD_YELLOW_LANE[1]) &
                   (l_channel >= L_THRESHOLD_YELLOW_LANE[0]) & (l_channel <= L_THRESHOLD_YELLOW_LANE[1]) &
                   (s_channel >= S_THRESHOLD_YELLOW_LANE[0]) & (s_channel <= S_THRESHOLD_YELLOW_LANE[1]))
    yellow_mask_cpu[yellow_cond] = 255

    white_mask_cpu = np.zeros_like(l_channel, dtype=np.uint8)
    white_cond = ((l_channel >= L_THRESHOLD_WHITE_LANE[0]) & (l_channel <= L_THRESHOLD_WHITE_LANE[1]) &
                  (s_channel >= S_THRESHOLD_WHITE_LANE[0]) & (s_channel <= S_THRESHOLD_WHITE_LANE[1]))
    white_mask_cpu[white_cond] = 255
    
    combined_mask_for_viz_cpu = cv2.bitwise_or(yellow_mask_cpu, white_mask_cpu)
    return yellow_mask_cpu, white_mask_cpu, combined_mask_for_viz_cpu

# 차선 픽셀 검출, 다항식 피팅 등은 CPU에서 수행
def get_lane_start_point_single_lane(roi_binary_mask, side='left'):
    histogram = np.sum(roi_binary_mask[roi_binary_mask.shape[0]//2:, :], axis=0)
    midpoint = histogram.shape[0]//2
    if side == 'left': return np.argmax(histogram[:midpoint]) if midpoint > 0 and np.any(histogram[:midpoint]) else 0
    else: return (np.argmax(histogram[midpoint:]) + midpoint) if midpoint < histogram.shape[0] and np.any(histogram[midpoint:]) else histogram.shape[0]-1

def find_single_lane_pixels_sliding_window(roi_binary_mask, initial_x_base, side_to_find='left'):
    output_viz_img = None
    if ENABLE_DEBUG_VISUALIZATION:
        output_viz_img = np.dstack((roi_binary_mask, roi_binary_mask, roi_binary_mask))
        if np.max(roi_binary_mask) <= 1 and roi_binary_mask.ndim == 2: output_viz_img *= 255
        output_viz_img = output_viz_img.astype(np.uint8)

    window_height = roi_binary_mask.shape[0] // SLIDING_WINDOW_COUNT; margin = int(roi_binary_mask.shape[1] * SLIDING_WINDOW_MARGIN_PERCENTAGE)
    nonzero = roi_binary_mask.nonzero(); nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])
    current_x = initial_x_base; lane_pixel_indices = []
    for window in range(SLIDING_WINDOW_COUNT):
        win_y_low = roi_binary_mask.shape[0] - (window + 1) * window_height; win_y_high = roi_binary_mask.shape[0] - window * window_height
        win_x_low, win_x_high = current_x - margin, current_x + margin
        if ENABLE_DEBUG_VISUALIZATION and output_viz_img is not None: cv2.rectangle(output_viz_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0,255,0), 2)
        good_indices = ((nonzeroy>=win_y_low)&(nonzeroy<win_y_high)&(nonzerox>=win_x_low)&(nonzerox<win_x_high)).nonzero()[0]
        lane_pixel_indices.append(good_indices)
        if len(good_indices) > MIN_PIXELS_FOR_RECENTERING: current_x = np.int32(np.mean(nonzerox[good_indices]))
    lane_pixel_indices = np.concatenate(lane_pixel_indices) if lane_pixel_indices and any(i.size>0 for i in lane_pixel_indices) else np.array([],dtype=np.int32)
    final_x = nonzerox[lane_pixel_indices] if lane_pixel_indices.size > 0 else np.array([]); final_y = nonzeroy[lane_pixel_indices] if lane_pixel_indices.size > 0 else np.array([])
    if ENABLE_DEBUG_VISUALIZATION and output_viz_img is not None and final_x.size > 0:
        color = [255,0,0] if side_to_find == 'left' else [0,0,255]; output_viz_img[final_y, final_x] = color
    return final_x, final_y, output_viz_img

def find_single_lane_pixels_around_poly(roi_binary_mask, poly_fit_coeffs, side_to_find='left'):
    output_viz_img = None
    if ENABLE_DEBUG_VISUALIZATION:
        output_viz_img = np.dstack((roi_binary_mask, roi_binary_mask, roi_binary_mask))
        if np.max(roi_binary_mask) <=1 and roi_binary_mask.ndim == 2: output_viz_img *= 255
        output_viz_img = output_viz_img.astype(np.uint8)
    
    margin = int(roi_binary_mask.shape[1] * POLYNOMIAL_SEARCH_MARGIN_PERCENTAGE)
    nonzero = roi_binary_mask.nonzero(); nonzeroy, nonzerox = np.array(nonzero[0]), np.array(nonzero[1])
    if poly_fit_coeffs is None or len(nonzeroy) == 0: return np.array([]), np.array([]), output_viz_img
    try:
        poly_line_x = poly_fit_coeffs[0]*(nonzeroy**2) + poly_fit_coeffs[1]*nonzeroy + poly_fit_coeffs[2]
        lane_indices_bool = (nonzerox > (poly_line_x - margin)) & (nonzerox < (poly_line_x + margin))
        final_x, final_y = nonzerox[lane_indices_bool], nonzeroy[lane_indices_bool]
        if ENABLE_DEBUG_VISUALIZATION and output_viz_img is not None:
            color = [255,0,0] if side_to_find == 'left' else [0,0,255]
            if final_x.size > 0: output_viz_img[final_y, final_x] = color
            ploty = np.linspace(0, roi_binary_mask.shape[0]-1, roi_binary_mask.shape[0])
            fitx = poly_fit_coeffs[0]*ploty**2 + poly_fit_coeffs[1]*ploty + poly_fit_coeffs[2]
            pts = np.array([np.transpose(np.vstack([fitx.astype(np.int32), ploty.astype(np.int32)]))])
            cv2.polylines(output_viz_img, pts, isClosed=False, color=(0,255,255), thickness=1)
    except (TypeError, ValueError, IndexError) as e: rospy.logwarn_throttle(1.0, f"Poly search error: {e}"); return np.array([]), np.array([]), output_viz_img
    return final_x, final_y, output_viz_img

def apply_polynomial_fit(detected_leftx_px, detected_lefty_px, detected_rightx_px, detected_righty_px, roi_image_h_px):
    global previous_left_polynomial_fit, previous_right_polynomial_fit
    current_left_fit, current_right_fit = None, None; left_valid, right_valid = False, False
    if detected_leftx_px.size > 2 and detected_lefty_px.size > 2:
        try: current_left_fit = np.polyfit(detected_lefty_px, detected_leftx_px, 2); left_valid=True
        except: pass
    if not left_valid: current_left_fit = previous_left_polynomial_fit
    if detected_rightx_px.size > 2 and detected_righty_px.size > 2:
        try: current_right_fit = np.polyfit(detected_righty_px, detected_rightx_px, 2); right_valid=True
        except: pass
    if not right_valid: current_right_fit = previous_right_polynomial_fit
    else: previous_right_polynomial_fit = current_right_fit
    y_points_for_plotting = np.linspace(0, roi_image_h_px - 1, roi_image_h_px, dtype=np.float32)
    fitted_leftx_px, fitted_rightx_px = np.array([]), np.array([])
    if current_left_fit is not None:
        try: fitted_leftx_px = current_left_fit[0]*y_points_for_plotting**2 + current_left_fit[1]*y_points_for_plotting + current_left_fit[2]; previous_left_polynomial_fit = current_left_fit
        except: previous_left_polynomial_fit = None; current_left_fit = None
    else: previous_left_polynomial_fit = None
    if current_right_fit is not None:
        try: fitted_rightx_px = current_right_fit[0]*y_points_for_plotting**2 + current_right_fit[1]*y_points_for_plotting + current_right_fit[2]; previous_right_polynomial_fit = current_right_fit
        except: previous_right_polynomial_fit = None; current_right_fit = None
    else: previous_right_polynomial_fit = None
    return current_left_fit, current_right_fit, fitted_leftx_px, fitted_rightx_px, y_points_for_plotting

def get_simplified_offset_and_curvature_params(fitted_leftx_px, fitted_rightx_px, y_points_roi, roi_image_dims):
    roi_h_px, roi_w_px = roi_image_dims; pixel_offset = 0.0
    if y_points_roi.size > 0:
        idx_bottom_roi = -1
        valid_left_fit = fitted_leftx_px.size == y_points_roi.size; valid_right_fit = fitted_rightx_px.size == y_points_roi.size
        if valid_left_fit and valid_right_fit : lane_center_bottom_px = (fitted_leftx_px[idx_bottom_roi] + fitted_rightx_px[idx_bottom_roi]) / 2; pixel_offset = (roi_w_px / 2) - lane_center_bottom_px
        elif valid_left_fit: pixel_offset = (roi_w_px * 0.25) - fitted_leftx_px[idx_bottom_roi]
        elif valid_right_fit: pixel_offset = (roi_w_px * 0.75) - fitted_rightx_px[idx_bottom_roi]
    left_curvature_param = previous_left_polynomial_fit[0] if previous_left_polynomial_fit is not None and len(previous_left_polynomial_fit)==3 else 0.0
    right_curvature_param = previous_right_polynomial_fit[0] if previous_right_polynomial_fit is not None and len(previous_right_polynomial_fit)==3 else 0.0
    avg_curvature_param = (left_curvature_param + right_curvature_param) / 2.0 if left_curvature_param!=0 and right_curvature_param!=0 else (left_curvature_param or right_curvature_param)
    heading_param = 0.0
    if previous_left_polynomial_fit is not None and previous_right_polynomial_fit is not None and y_points_roi.size > 1:
        y_eval_bottom = y_points_roi[-1]; y_eval_mid = y_points_roi[len(y_points_roi)//2]
        if (y_eval_bottom - y_eval_mid) != 0:
            try:
                xl_b = previous_left_polynomial_fit[0]*y_eval_bottom**2 + previous_left_polynomial_fit[1]*y_eval_bottom + previous_left_polynomial_fit[2]
                xr_b = previous_right_polynomial_fit[0]*y_eval_bottom**2 + previous_right_polynomial_fit[1]*y_eval_bottom + previous_right_polynomial_fit[2]
                xl_m = previous_left_polynomial_fit[0]*y_eval_mid**2 + previous_left_polynomial_fit[1]*y_eval_mid + previous_left_polynomial_fit[2]
                xr_m = previous_right_polynomial_fit[0]*y_eval_mid**2 + previous_right_polynomial_fit[1]*y_eval_mid + previous_right_polynomial_fit[2]
                heading_param = ((xl_b+xr_b)/2 - (xl_m+xr_m)/2) / (y_eval_bottom - y_eval_mid)
            except: pass
    return pixel_offset, avg_curvature_param, heading_param

def draw_lane_visualization_on_roi(resized_bgr_img, current_roi_y_start_abs, fitted_leftx_roi, fitted_rightx_roi, y_points_roi, pixel_offset, curvature_param, steer_angle):
    output_image = resized_bgr_img.copy(); font, font_scale, font_color, line_type = cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2; text_y, text_dy = 30, 25
    if y_points_roi.size == 0: cv2.putText(output_image, "No Y-pts", (10,text_y),font,font_scale,(0,0,255),line_type); return output_image
    if fitted_leftx_roi.size == y_points_roi.size: cv2.polylines(output_image, np.int32([np.transpose(np.vstack([fitted_leftx_roi, y_points_roi + current_roi_y_start_abs]))]), False, (255,0,0),3)
    if fitted_rightx_roi.size == y_points_roi.size: cv2.polylines(output_image, np.int32([np.transpose(np.vstack([fitted_rightx_roi, y_points_roi + current_roi_y_start_abs]))]), False, (0,0,255),3)
    cv2.putText(output_image,f"PxlOff:{pixel_offset:.1f}",(10,text_y),font,font_scale,font_color,line_type)
    cv2.putText(output_image,f"CurvP:{curvature_param:.4f}",(10,text_y+text_dy),font,font_scale,font_color,line_type)
    cv2.putText(output_image,f"Steer:{steer_angle:.1f}deg",(10,text_y+2*text_dy),font,font_scale,font_color,line_type)
    return output_image

def main_image_processing_pipeline(cv_image):
    global previous_left_polynomial_fit, previous_right_polynomial_fit, previous_vehicle_offset_meters, last_time_for_derivative_calc, accumulated_integral_offset_meters, roi_y_start_abs
    
    start_time_frame = time.time() if PROFILE_PERFORMANCE else 0

    roi_image_bgr, resized_original_bgr, current_roi_y_start = preprocess_and_get_roi(cv_image)
    roi_y_start_abs = current_roi_y_start; roi_image_dims = roi_image_bgr.shape[:2]
    
    start_time_mask = time.time() if PROFILE_PERFORMANCE else 0
    yellow_mask_roi, white_mask_roi, combined_mask_roi_viz = generate_lane_masks_separated(roi_image_bgr)
    if PROFILE_PERFORMANCE: rospy.loginfo(f"Time - Mask Gen: {(time.time() - start_time_mask)*1000:.2f} ms")

    detected_leftx, detected_lefty, detected_rightx, detected_righty = np.array([]),np.array([]),np.array([]),np.array([])
    
    viz_left_base = None; viz_right_base = None; lane_search_viz_img = None
    if ENABLE_DEBUG_VISUALIZATION:
        viz_left_base = np.dstack((yellow_mask_roi,)*3); viz_right_base = np.dstack((white_mask_roi,)*3)
        if np.max(yellow_mask_roi) <=1 and yellow_mask_roi.ndim == 2: viz_left_base *=255
        if np.max(white_mask_roi) <=1 and white_mask_roi.ndim == 2: viz_right_base *=255
        viz_left_base = viz_left_base.astype(np.uint8); viz_right_base = viz_right_base.astype(np.uint8)

    start_time_search = time.time() if PROFILE_PERFORMANCE else 0
    is_yellow_valid = np.sum(yellow_mask_roi) > MIN_PIXEL_SUM_FOR_VALID_MASK; temp_lx, temp_ly = np.array([]),np.array([])
    active_l_mask = yellow_mask_roi; final_viz_l = viz_left_base.copy() if viz_left_base is not None else None; l_color="Yellow"
    if previous_left_polynomial_fit is not None and is_yellow_valid:
        temp_lx,temp_ly,v_l_p = find_single_lane_pixels_around_poly(active_l_mask,previous_left_polynomial_fit,'left')
        if ENABLE_DEBUG_VISUALIZATION and v_l_p is not None: final_viz_l = v_l_p if temp_lx.size > 0 else final_viz_l
    if temp_lx.size < MIN_PIXELS_FOR_RECENTERING and is_yellow_valid:
        initial_lx=get_lane_start_point_single_lane(active_l_mask,'left')
        temp_lx,temp_ly,v_l_s=find_single_lane_pixels_sliding_window(active_l_mask,initial_lx,'left')
        if ENABLE_DEBUG_VISUALIZATION and v_l_s is not None: final_viz_l = v_l_s if temp_lx.size > 0 else final_viz_l
    if temp_lx.size < MIN_PIXELS_FOR_RECENTERING:
        active_l_mask=white_mask_roi; l_color="White(FB)"
        if ENABLE_DEBUG_VISUALIZATION and viz_right_base is not None: final_viz_l=viz_right_base.copy()
        if previous_left_polynomial_fit is not None:
             temp_lx,temp_ly,v_l_p_fb=find_single_lane_pixels_around_poly(active_l_mask,previous_left_polynomial_fit,'left')
             if ENABLE_DEBUG_VISUALIZATION and v_l_p_fb is not None: final_viz_l = v_l_p_fb if temp_lx.size > 0 else final_viz_l
        if temp_lx.size < MIN_PIXELS_FOR_RECENTERING and np.sum(active_l_mask)>MIN_PIXEL_SUM_FOR_VALID_MASK:
            initial_lx=get_lane_start_point_single_lane(active_l_mask,'left')
            temp_lx,temp_ly,v_l_s_fb=find_single_lane_pixels_sliding_window(active_l_mask,initial_lx,'left')
            if ENABLE_DEBUG_VISUALIZATION and v_l_s_fb is not None: final_viz_l = v_l_s_fb if temp_lx.size > 0 else final_viz_l
    detected_leftx,detected_lefty = temp_lx,temp_ly
    
    active_r_mask = white_mask_roi; final_viz_r = viz_right_base.copy() if viz_right_base is not None else None; temp_rx,temp_ry = np.array([]),np.array([])
    if previous_right_polynomial_fit is not None:
        temp_rx,temp_ry,v_r_p=find_single_lane_pixels_around_poly(active_r_mask,previous_right_polynomial_fit,'right')
        if ENABLE_DEBUG_VISUALIZATION and v_r_p is not None: final_viz_r = v_r_p if temp_rx.size > 0 else final_viz_r
    if temp_rx.size < MIN_PIXELS_FOR_RECENTERING and np.sum(active_r_mask)>MIN_PIXEL_SUM_FOR_VALID_MASK:
        initial_rx=get_lane_start_point_single_lane(active_r_mask,'right')
        temp_rx,temp_ry,v_r_s=find_single_lane_pixels_sliding_window(active_r_mask,initial_rx,'right')
        if ENABLE_DEBUG_VISUALIZATION and v_r_s is not None: final_viz_r = v_r_s if temp_rx.size > 0 else final_viz_r
    detected_rightx,detected_righty = temp_rx,temp_ry
    if PROFILE_PERFORMANCE: rospy.loginfo(f"Time - Lane Search: {(time.time() - start_time_search)*1000:.2f} ms")
    
    if ENABLE_DEBUG_VISUALIZATION:
        lane_search_viz_img = cv2.cvtColor(combined_mask_roi_viz,cv2.COLOR_GRAY2BGR)
        if detected_leftx.size>0: lane_search_viz_img[detected_lefty,detected_leftx]=[255,100,100]
        if detected_rightx.size>0: lane_search_viz_img[detected_righty,detected_rightx]=[100,100,255]
    
    curr_l_fit,curr_r_fit,fit_lx_roi,fit_rx_roi,y_pts_roi = apply_polynomial_fit(detected_leftx,detected_lefty,detected_rightx,detected_righty,roi_image_dims[0])
    
    # === 차선 복귀 모드 통합 ===
    global lane_recovery_mode, previous_steering_angle
    
    # 차선 신뢰도 계산 (검출된 픽셀 수 기반)
    left_confidence = min(1.0, detected_leftx.size / 100.0) if detected_leftx.size > 0 else 0.0
    right_confidence = min(1.0, detected_rightx.size / 100.0) if detected_rightx.size > 0 else 0.0
    
    # 차선 상태 감지
    lane_status, detection_confidence = detect_lane_loss(
        curr_l_fit is not None and detected_leftx.size > MIN_PIXELS_FOR_RECENTERING,
        curr_r_fit is not None and detected_rightx.size > MIN_PIXELS_FOR_RECENTERING,
        left_confidence, right_confidence
    )
    
    # 기본 오프셋 및 곡률 계산
    px_offset,curv_param,head_param = get_simplified_offset_and_curvature_params(fit_lx_roi,fit_rx_roi,y_pts_roi,roi_image_dims)
    
    # 복귀 모드 조향각 계산
    recovery_steering = 0.0
    vehicle_center_x = roi_image_dims[1] / 2  # ROI 중앙
    
    if lane_status == "left_lost" and curr_r_fit is not None:
        # 왼쪽 차선 소실 - 오른쪽 차선 기준 복귀
        recovery_steering = calculate_recovery_steering(lane_status, curr_r_fit, vehicle_center_x, roi_image_dims[0], detection_confidence)
        lane_recovery_mode = {"active": True, "type": "left_lost", "confidence": detection_confidence}
        
    elif lane_status == "right_lost" and curr_l_fit is not None:
        # 오른쪽 차선 소실 - 왼쪽 차선 기준 복귀
        recovery_steering = calculate_recovery_steering(lane_status, curr_l_fit, vehicle_center_x, roi_image_dims[0], detection_confidence)
        lane_recovery_mode = {"active": True, "type": "right_lost", "confidence": detection_confidence}
        
    elif lane_status == "both_detected":
        # 정상 모드
        lane_recovery_mode = {"active": False, "type": "normal", "confidence": detection_confidence}
        
    else:
        # 양쪽 차선 모두 소실 - 직진 유지 모드
        recovery_steering = calculate_recovery_steering(lane_status, None, vehicle_center_x, roi_image_dims[0], 0.0)
        lane_recovery_mode = {"active": True, "type": "both_lost", "confidence": 0.0}
    
    # PID 제어 계산
    curr_offset_pid = px_offset; curr_time = rospy.get_time(); dt = curr_time - last_time_for_derivative_calc if last_time_for_derivative_calc > 0 else 0.0
    deriv_offset = (curr_offset_pid - previous_vehicle_offset_meters)/dt if dt > 0.001 else 0.0
    if dt > 0: accumulated_integral_offset_meters += curr_offset_pid * dt
    accumulated_integral_offset_meters = np.clip(accumulated_integral_offset_meters,-MAX_INTEGRAL_TERM_VALUE,MAX_INTEGRAL_TERM_VALUE)
    previous_vehicle_offset_meters=curr_offset_pid; last_time_for_derivative_calc=curr_time
    p_off=KP_LATERAL_OFFSET*curr_offset_pid; i_off=KI_LATERAL_OFFSET*accumulated_integral_offset_meters; d_off=KD_LATERAL_OFFSET*deriv_offset
    p_head=KP_HEADING_ERROR*head_param; ff_curv=K_FF_LANE_CURVATURE*curv_param
    
    # 조향각 통합 계산
    if lane_recovery_mode["active"] and lane_recovery_mode["type"] != "both_lost":
        # 복귀 모드: 복귀 조향과 일반 조향을 가중 평균
        normal_steer_rad = -(p_off+i_off+d_off) - p_head + ff_curv
        normal_steer_deg = np.degrees(normal_steer_rad)
        
        # 신뢰도 기반 가중치 적용
        recovery_steering_weighted = apply_confidence_weighting(recovery_steering, detection_confidence)
        
        # 복귀 모드와 일반 모드 블렌딩
        blend_factor = detection_confidence
        blended_steering = (1 - blend_factor) * normal_steer_deg + blend_factor * recovery_steering_weighted
        
        # 점진적 복귀 제어
        final_steer_deg = gradual_recovery_control(previous_steering_angle, blended_steering, max_change_rate=2.0)
    else:
        # 일반 모드
        steer_rad = -(p_off+i_off+d_off) - p_head + ff_curv
        steer_deg = np.degrees(steer_rad)
        final_steer_deg = steer_deg
    
    # 조향각 제한
    final_steer_deg = np.clip(final_steer_deg,-MAX_STEERING_ANGLE_DEGREES,MAX_STEERING_ANGLE_DEGREES)
    previous_steering_angle = final_steer_deg
    
    final_speed=MAX_SPEED; valid_l=curr_l_fit is not None and fit_lx_roi.size>0; valid_r=curr_r_fit is not None and fit_rx_roi.size>0
    
    # 복귀 모드에 따른 속도 조절
    if lane_recovery_mode["active"]:
        if lane_recovery_mode["type"] == "both_lost":
            # 양쪽 차선 모두 소실 - 안전을 위해 대폭 감속
            final_speed = WEAK_LANE_DETECTION_SPEED * 0.4  # 40%로 감속
            rospy.logwarn_throttle(1.0, "EMERGENCY SPEED: Both lanes lost")
            
        elif lane_recovery_mode["type"] in ["left_lost", "right_lost"]:
            # 한쪽 차선 소실 - 신뢰도와 조향각에 따라 속도 조절
            confidence_factor = max(0.4, lane_recovery_mode["confidence"])  # 최소 40%
            steering_factor = max(0.6, 1.0 - abs(recovery_steering) / 20.0)  # 조향각이 클수록 감속
            
            # 복합 안전 계수 적용
            safety_speed_factor = confidence_factor * steering_factor * 0.7  # 최대 70% 속도
            recovery_speed = MAX_SPEED * safety_speed_factor
            
            # 최소/최대 속도 제한
            final_speed = np.clip(recovery_speed, WEAK_LANE_DETECTION_SPEED * 0.6, MAX_SPEED * 0.8)
            
            rospy.loginfo_throttle(2.0, f"RECOVERY SPEED: {final_speed:.0f} (conf:{confidence_factor:.2f}, steer:{steering_factor:.2f})")
    else:
        # 일반 모드 속도 제어
        if not valid_l and not valid_r: 
            final_speed=WEAK_LANE_DETECTION_SPEED
        elif not valid_l or not valid_r: 
            final_speed=min(MAX_SPEED*0.7,WEAK_LANE_DETECTION_SPEED*1.5)
        else:
            spd_curv=MAX_SPEED; spd_ang=MAX_SPEED
            if abs(curv_param)>0.0005: spd_curv=MIN_SPEED_CURVE
            elif abs(curv_param)>0.0001: spd_curv=MAX_SPEED*0.7
            if abs(final_steer_deg)>25: spd_ang=MIN_SPEED_ANGLE
            elif abs(final_steer_deg)>15: spd_ang=MAX_SPEED*0.6
            final_speed=min(spd_curv,spd_ang,MAX_SPEED)
        
    final_viz_img = None
    if ENABLE_DEBUG_VISUALIZATION or PROFILE_PERFORMANCE:
         final_viz_img = draw_lane_visualization_on_roi(resized_original_bgr,roi_y_start_abs,fit_lx_roi,fit_rx_roi,y_pts_roi,px_offset,curv_param,final_steer_deg)

    # 디버그 정보에 복귀 모드 정보 추가
    dbg_data={"vehicle_offset_m":px_offset,"heading_error_rad":head_param,"target_steering_angle_rad":0,"final_steering_angle_deg":final_steer_deg,
              "final_vehicle_speed":final_speed,"pid_p_offset":p_off,"pid_i_offset":i_off,"pid_d_offset":d_off,"pid_p_heading":p_head,"ff_curvature":ff_curv,
              "left_curvature_m":curv_param,"right_curvature_m":curv_param,"accumulated_integral_offset_m":accumulated_integral_offset_meters,
              "left_lane_detected":valid_l,"right_lane_detected":valid_r,"left_lane_color":l_color,
              "lane_recovery_mode":lane_recovery_mode["type"],"recovery_confidence":lane_recovery_mode["confidence"],
              "lane_status":lane_status,"recovery_steering":recovery_steering,
              "emergency_counter":emergency_straight_counter,"last_valid_center":last_valid_lane_center,
              "left_confidence":left_confidence,"right_confidence":right_confidence,
              "stable_left":stable_detection_counter["left"],"stable_right":stable_detection_counter["right"],"stable_both":stable_detection_counter["both"],
              "mode_lock_counter":current_mode_lock_counter}
    
    if PROFILE_PERFORMANCE: rospy.loginfo(f"Time - Frame Total: {(time.time() - start_time_frame)*1000:.2f} ms")
    
    return final_steer_deg,final_speed,final_viz_img,combined_mask_roi_viz,combined_mask_roi_viz,lane_search_viz_img,dbg_data

def ros_image_callback(ros_img_msg):
    global current_image_cv
    try: current_image_cv = bridge.imgmsg_to_cv2(ros_img_msg,"bgr8")
    except Exception as e: rospy.logerr(f"CvBridge Error: {e}"); current_image_cv = None

def publish_motor_commands(steering_angle,vehicle_speed):
    if motor_control_publisher is not None:
        motor_msg = XycarMotor(); motor_msg.angle=float(steering_angle); motor_msg.speed=float(vehicle_speed)
        motor_control_publisher.publish(motor_msg)

def ros_main_loop():
    global current_image_cv,motor_control_publisher,previous_left_polynomial_fit,previous_right_polynomial_fit, \
           last_time_for_derivative_calc, previous_vehicle_offset_meters, accumulated_integral_offset_meters, roi_y_start_abs
    rospy.init_node('optimized_lane_controller_v5')
    motor_control_publisher = rospy.Publisher('/xycar_motor',XycarMotor,queue_size=1)
    rospy.Subscriber('/usb_cam/image_raw',Image,ros_image_callback,queue_size=1, buff_size=2**24)
    rospy.loginfo("Waiting for initial image...")
    while current_image_cv is None and not rospy.is_shutdown(): rospy.loginfo_throttle(1.0,"Still no image..."); time.sleep(0.01)
    if rospy.is_shutdown(): return
    rospy.loginfo("Initial image received. Starting Optimized Controller.")
    rospy.loginfo("Using CPU for processing.")
    
    last_time_for_derivative_calc=rospy.get_time(); previous_vehicle_offset_meters=0.0; accumulated_integral_offset_meters=0.0
    previous_left_polynomial_fit=None; previous_right_polynomial_fit=None; roi_y_start_abs=0
    
    loop_rate = rospy.Rate(24) # 목표 FPS에 맞춰 Rate 설정 (24FPS)
    
    try:
        while not rospy.is_shutdown():
            start_ros_loop_iter = time.time() if PROFILE_PERFORMANCE else 0
            if current_image_cv is None: loop_rate.sleep(); continue
            
            local_image_copy = current_image_cv
            
            final_angle,final_speed,viz_final,viz_mask_roi,viz_roi_mask_for_warp_pos,viz_search_roi,dbg_info = main_image_processing_pipeline(local_image_copy)
            publish_motor_commands(final_angle,final_speed)

            if ENABLE_DEBUG_VISUALIZATION or PROFILE_PERFORMANCE:
                # 복귀 모드별 상세 정보 표시
                if dbg_info['lane_recovery_mode'] == 'both_lost':
                    recovery_info = f"EMERGENCY({dbg_info['emergency_counter']})"
                elif dbg_info['lane_recovery_mode'] in ['left_lost', 'right_lost']:
                    recovery_info = f"Recovery:{dbg_info['lane_recovery_mode']}({dbg_info['recovery_confidence']:.2f}) RS:{dbg_info['recovery_steering']:.1f}"
                else:
                    recovery_info = "Normal"
                
                # 안정성 정보 추가
                stability_info = f"StableL:{dbg_info['stable_left']}/R:{dbg_info['stable_right']}/Both:{dbg_info['stable_both']}"
                if dbg_info['mode_lock_counter'] > 0:
                    stability_info += f" Lock:{dbg_info['mode_lock_counter']}"
                    
                log_str = (f"PxlOff:{dbg_info['vehicle_offset_m']:.1f} HeadP:{dbg_info.get('heading_error_rad',0):.2f} "
                           f"Steer:{final_angle:.1f} Speed:{final_speed:.0f} "
                           f"L:{dbg_info['left_lane_detected']}({dbg_info['left_confidence']:.2f}) R:{dbg_info['right_lane_detected']}({dbg_info['right_confidence']:.2f}) "
                           f"{recovery_info} {stability_info}")
                rospy.loginfo(log_str)

            if ENABLE_DEBUG_VISUALIZATION:
                if viz_final is not None: cv2.imshow("1. Final Visualization (ROI Lanes)",viz_final)
                # if viz_mask_roi is not None: cv2.imshow("2. ROI Combined Mask (Refined)", viz_mask_roi)
                # if viz_roi_mask_for_warp_pos is not None: cv2.imshow("3. ROI Mask (No Warp, Refined)", viz_roi_mask_for_warp_pos)
                if viz_search_roi is not None: cv2.imshow("4. Lane Search on ROI (Refined)", viz_search_roi)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'): rospy.signal_shutdown("Q pressed"); break
                elif key == ord('p'): cv2.waitKey(0)
            
            if PROFILE_PERFORMANCE: rospy.loginfo(f"Time - ROS Loop Total: {(time.time() - start_ros_loop_iter)*1000:.2f} ms")
            loop_rate.sleep()
            
    except rospy.ROSInterruptException: rospy.loginfo("ROSInterruptException caught.")
    except Exception as e: rospy.logerr(f"Exception in main loop: {e}"); import traceback; traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        if motor_control_publisher is not None: publish_motor_commands(0.0, 0.0)
        rospy.loginfo("Controller node shut down.")

# --- 차선 복귀 모드 함수들 ---
def detect_lane_loss(left_lane_detected, right_lane_detected, left_confidence=0.0, right_confidence=0.0, confidence_threshold=0.5):
    """차선 소실 상황 감지 - 차선 변경 방지 로직 포함"""
    global lane_loss_counter, confidence_history, stable_detection_counter, current_mode_lock_counter, lane_recovery_mode
    
    # 신뢰도 히스토리 업데이트
    confidence_history["left"].append(left_confidence)
    confidence_history["right"].append(right_confidence)
    
    if len(confidence_history["left"]) > max_history_length:
        confidence_history["left"].pop(0)
    if len(confidence_history["right"]) > max_history_length:
        confidence_history["right"].pop(0)
    
    # 평균 신뢰도 계산
    avg_left_confidence = sum(confidence_history["left"]) / len(confidence_history["left"]) if confidence_history["left"] else 0
    avg_right_confidence = sum(confidence_history["right"]) / len(confidence_history["right"]) if confidence_history["right"] else 0
    
    # 높은 임계값으로 안정적인 차선 감지 판단
    stable_threshold = confidence_threshold + 0.2  # 더 높은 임계값으로 안정성 확보
    left_stable = left_lane_detected and avg_left_confidence > stable_threshold
    right_stable = right_lane_detected and avg_right_confidence > stable_threshold
    
    # 기본 유효성 판단 (기존 로직)
    left_valid = left_lane_detected and avg_left_confidence > confidence_threshold
    right_valid = right_lane_detected and avg_right_confidence > confidence_threshold
    
    # 안정적인 감지 카운터 업데이트
    if left_stable:
        stable_detection_counter["left"] += 1
    else:
        stable_detection_counter["left"] = 0
        
    if right_stable:
        stable_detection_counter["right"] += 1  
    else:
        stable_detection_counter["right"] = 0
        
    if left_stable and right_stable:
        stable_detection_counter["both"] += 1
    else:
        stable_detection_counter["both"] = 0
    
    # 차선 소실 카운터 업데이트 (기존 로직)
    if not left_valid:
        lane_loss_counter["left"] += 1
    else:
        lane_loss_counter["left"] = 0
        
    if not right_valid:
        lane_loss_counter["right"] += 1
    else:
        lane_loss_counter["right"] = 0
    
    # 현재 모드 상태 확인
    current_recovery_active = lane_recovery_mode.get("active", False)
    current_recovery_type = lane_recovery_mode.get("type", "normal")
    
    # === 차선 변경 방지 로직 === 
    
    # 1. 양쪽 차선이 안정적으로 감지되면 반드시 정상 모드로 복귀
    if stable_detection_counter["both"] >= recovery_exit_threshold:
        rospy.loginfo_throttle(1.0, f"STABLE LANES DETECTED: Exiting recovery mode (both stable for {stable_detection_counter['both']} frames)")
        current_mode_lock_counter = 0
        return "both_detected", min(avg_left_confidence, avg_right_confidence)
    
    # 2. 복귀 모드 중일 때 해당 차선이 안정적으로 복구되면 정상 모드로 복귀
    if current_recovery_active:
        if (current_recovery_type == "left_lost" and stable_detection_counter["left"] >= recovery_exit_threshold):
            rospy.loginfo_throttle(1.0, f"LEFT LANE RECOVERED: Exiting recovery mode (stable for {stable_detection_counter['left']} frames)")
            current_mode_lock_counter = 0
            return "both_detected", min(avg_left_confidence, avg_right_confidence)
            
        elif (current_recovery_type == "right_lost" and stable_detection_counter["right"] >= recovery_exit_threshold):
            rospy.loginfo_throttle(1.0, f"RIGHT LANE RECOVERED: Exiting recovery mode (stable for {stable_detection_counter['right']} frames)")
            current_mode_lock_counter = 0 
            return "both_detected", min(avg_left_confidence, avg_right_confidence)
        
        # 3. 복귀 모드 중에는 더 엄격한 기준으로 판단 (모드 변경 최소화)
        current_mode_lock_counter += 1
        if current_mode_lock_counter < 10:  # 10프레임 동안은 모드 유지
            # 현재 복귀 모드 상태 유지
            if current_recovery_type == "left_lost":
                return "left_lost", avg_right_confidence
            elif current_recovery_type == "right_lost":
                return "right_lost", avg_left_confidence
            elif current_recovery_type == "both_lost":
                return "both_lost", 0.0
    
    # 4. 일반 모드에서 복귀 모드 진입 (연속 소실 확인)
    current_mode_lock_counter = 0  # 새로운 상황이므로 락 해제
    
    # 연속 소실 기준으로 복귀 모드 진입 판단
    if (not left_valid and right_valid and 
        lane_loss_counter["left"] >= recovery_enter_threshold and
        stable_detection_counter["right"] >= 2):  # 오른쪽은 최소 2프레임 안정
        return "left_lost", avg_right_confidence
        
    elif (left_valid and not right_valid and 
          lane_loss_counter["right"] >= recovery_enter_threshold and
          stable_detection_counter["left"] >= 2):  # 왼쪽은 최소 2프레임 안정
        return "right_lost", avg_left_confidence
        
    elif (left_valid and right_valid):
        return "both_detected", min(avg_left_confidence, avg_right_confidence)
    else:
        # 양쪽 모두 소실
        return "both_lost", 0.0

def estimate_lane_center_single_line(detected_line_x, lane_type="left"):
    """단일 차선으로부터 차선 중앙 추정"""
    global lane_width_pixels
    
    if lane_type == "left":
        # 왼쪽 차선만 검출된 경우, 오른쪽으로 차선 폭의 절반만큼 이동
        estimated_right_x = detected_line_x + lane_width_pixels
        estimated_center = (detected_line_x + estimated_right_x) / 2
        return estimated_center, estimated_right_x
    else:
        # 오른쪽 차선만 검출된 경우, 왼쪽으로 차선 폭의 절반만큼 이동
        estimated_left_x = detected_line_x - lane_width_pixels
        estimated_center = (estimated_left_x + detected_line_x) / 2
        return estimated_center, estimated_left_x

def calculate_recovery_steering(lane_status, detected_line_fit, vehicle_center_x, roi_height, confidence):
    """복귀 모드 조향각 계산 - 상황별 제어 전략"""
    global lane_width_pixels, last_valid_lane_center, last_valid_heading, emergency_straight_counter
    
    if lane_status == "both_lost":
        # 양쪽 차선 모두 소실 - 마지막 감지 위치에서 직진 유지
        emergency_straight_counter += 1
        
        if last_valid_lane_center is not None and emergency_straight_counter < 30:  # 30프레임(약 1.25초) 동안 유지
            # 마지막 유효한 차선 중앙을 기준으로 조향
            target_offset = vehicle_center_x - last_valid_lane_center
            emergency_steering = np.degrees(np.arctan(target_offset / (lane_width_pixels * 2))) * 0.3  # 매우 약한 조향
            
            rospy.logwarn_throttle(1.0, f"EMERGENCY STRAIGHT: Using last valid center, offset: {target_offset:.1f}")
            return emergency_steering
        else:
            # 마지막 헤딩 방향으로 직진 (매우 약한 보정)
            straight_steering = last_valid_heading * 0.1  # 10%만 적용
            rospy.logwarn_throttle(1.0, f"EMERGENCY STRAIGHT: Maintaining heading {straight_steering:.1f}")
            return straight_steering
    
    # 정상적인 복귀 모드 처리
    emergency_straight_counter = 0  # 정상 모드로 복귀 시 카운터 리셋
    
    if detected_line_fit is None:
        return 0.0
    
    # 이미지 하단에서의 차선 위치 계산
    y_eval = roi_height - 1
    detected_x = detected_line_fit[0] * y_eval**2 + detected_line_fit[1] * y_eval + detected_line_fit[2]
    
    if lane_status == "left_lost":
        # 왼쪽 차선 소실: 오른쪽 차선 기준으로 안전 거리 유지
        # 차선 폭의 75% 지점을 목표 (더 안전하게)
        target_position = detected_x - (lane_width_pixels * 0.75)
        strategy_info = "RIGHT_REF_LEFT_RECOVERY"
        
    elif lane_status == "right_lost":
        # 오른쪽 차선 소실: 왼쪽 차선에서 적정 거리 확보  
        # 차선 폭의 75% 지점을 목표
        target_position = detected_x + (lane_width_pixels * 0.75)
        strategy_info = "LEFT_REF_RIGHT_RECOVERY"
        
    else:
        return 0.0
    
    # 유효한 차선 정보 업데이트 (양쪽 소실이 아닐 때)
    if lane_status in ["left_lost", "right_lost"]:
        # 현재 감지된 차선을 기준으로 차선 중앙 추정
        if lane_status == "left_lost":
            estimated_center = detected_x - (lane_width_pixels * 0.5)
        else:
            estimated_center = detected_x + (lane_width_pixels * 0.5)
        
        last_valid_lane_center = estimated_center
        
        # 헤딩 추정 (간단한 1차 미분)
        if len(detected_line_fit) >= 2:
            last_valid_heading = np.degrees(np.arctan(detected_line_fit[1]))  # 1차 계수에서 헤딩 추정
    
    # 오프셋 계산
    offset = vehicle_center_x - target_position
    
    # 조향각 계산 (신뢰도와 안전 계수 적용)
    safety_factor = 0.8  # 80% 안전 계수
    steering_angle_rad = np.arctan(offset / lane_width_pixels) * confidence * safety_factor
    steering_angle_deg = np.degrees(steering_angle_rad)
    
    # 조향각 제한 (차량 동역학 고려)
    max_recovery_angle = 15.0  # 복귀 모드에서 최대 15도로 제한
    steering_angle_deg = np.clip(steering_angle_deg, -max_recovery_angle, max_recovery_angle)
    
    rospy.loginfo_throttle(2.0, f"Recovery Strategy: {strategy_info}, Target: {target_position:.1f}, Offset: {offset:.1f}, Steer: {steering_angle_deg:.1f}")
    
    return steering_angle_deg

def gradual_recovery_control(current_steering, target_steering, max_change_rate=3.0):
    """점진적 복귀 제어 - 차량 동역학 고려"""
    global lane_recovery_mode, previous_steering_angle
    
    steering_diff = target_steering - current_steering
    
    # 복귀 모드별 제한 속도 조정
    if lane_recovery_mode.get("active", False):
        recovery_type = lane_recovery_mode.get("type", "normal")
        confidence = lane_recovery_mode.get("confidence", 0.0)
        
        # 신뢰도 기반 적응적 제한
        adaptive_rate = max_change_rate * (0.5 + 0.5 * confidence)  # 50%~100% 범위
        
        if recovery_type == "both_lost":
            # 양쪽 차선 소실 시 매우 보수적 제어
            adaptive_rate = min(adaptive_rate, 1.0)  # 최대 1도/프레임
        elif recovery_type in ["left_lost", "right_lost"]:
            # 한쪽 차선 소실 시 중간 수준 제어  
            adaptive_rate = min(adaptive_rate, 2.5)  # 최대 2.5도/프레임
    else:
        adaptive_rate = max_change_rate
    
    # 조향각 변화율 제한 (차량 동역학 고려)
    if abs(steering_diff) > adaptive_rate:
        if steering_diff > 0:
            limited_steering = current_steering + adaptive_rate
        else:
            limited_steering = current_steering - adaptive_rate
            
        # 추가 안전 검사: 급격한 방향 전환 방지
        if abs(limited_steering - previous_steering_angle) > adaptive_rate * 1.5:
            # 이전 조향각과의 차이도 고려
            smoothing_factor = 0.7  # 70% 현재, 30% 이전
            limited_steering = smoothing_factor * limited_steering + (1 - smoothing_factor) * previous_steering_angle
            
        return limited_steering
    
    return target_steering

def apply_confidence_weighting(steering_angle, confidence, min_confidence=0.3):
    """신뢰도 기반 조향 가중치"""
    if confidence < min_confidence:
        return steering_angle * 0.1  # 신뢰도가 너무 낮으면 조향 강도를 크게 줄임
    
    weight = (confidence - min_confidence) / (1.0 - min_confidence)
    return steering_angle * weight

if __name__ == '__main__':
    try: ros_main_loop()
    except Exception as e: print(f"CRITICAL UNHANDLED EXCEPTION IN __main__: {e}"); import traceback; traceback.print_exc()
