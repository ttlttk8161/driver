#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from scipy.interpolate import CubicSpline

class Perception:
    def __init__(self):
        self._last_left_lines = []
        self._last_right_lines = []
        self._last_roi_y_start = None
        self._last_waypoints = []
        self._last_valid_waypoints = []
        self.processed_image = None
        # 1. 클래스 변수 추가
        self._last_yellow_left_line = None
        self._last_yellow_right_line = None

    def process_image(self, image):
        if image is None or image.size == 0:
            return None, image
        self.processed_image = self._preprocess_image(image)
        self._extract_lane_lines(self.processed_image, image)
        waypoints = self._extract_lane_waypoints(image)
        # 3개 이상이면 정상 저장
        if len(waypoints) >= 3:
            self._last_valid_waypoints = waypoints
        self._last_waypoints = waypoints
        vis_img = self._draw_waypoints_and_spline(image, waypoints)
        return waypoints, vis_img

    def _preprocess_image(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        return blur

    def _extract_lane_lines(self, processed_image, orig_image):
        height, width = processed_image.shape
        roi_y_start = int(height * 0.5)
        self._last_roi_y_start = roi_y_start
        hsv = cv2.cvtColor(orig_image[roi_y_start:, :], cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 40, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([40, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        edges_white = cv2.Canny(mask_white, 50, 150)
        edges_yellow = cv2.Canny(mask_yellow, 50, 150)
        lines_white = cv2.HoughLinesP(edges_white, 1, np.pi/180, threshold=60, minLineLength=60, maxLineGap=30)
        lines_yellow = cv2.HoughLinesP(edges_yellow, 1, np.pi/180, threshold=60, minLineLength=60, maxLineGap=30)
        left_lines = []
        right_lines = []
        min_slope = 0.5
        min_length = 40
        if lines_white is not None:
            for line in lines_white:
                x1, y1, x2, y2 = line[0]
                dx = x2 - x1
                dy = y2 - y1
                if dx == 0:
                    continue
                slope = dy / dx
                length = np.hypot(dx, dy)
                if abs(slope) < min_slope or length < min_length:
                    continue
                y1_full = y1 + roi_y_start
                y2_full = y2 + roi_y_start
                if slope < 0:
                    left_lines.append([x1, y1_full, x2, y2_full, 'white'])
                else:
                    right_lines.append([x1, y1_full, x2, y2_full, 'white'])
        yellow_detected = False
        if lines_yellow is not None:
            for line in lines_yellow:
                x1, y1, x2, y2 = line[0]
                dx = x2 - x1
                dy = y2 - y1
                if dx == 0:
                    continue
                slope = dy / dx
                length = np.hypot(dx, dy)
                if abs(slope) < min_slope or length < min_length:
                    continue
                y1_full = y1 + roi_y_start
                y2_full = y2 + roi_y_start
                if slope < 0:
                    left_lines.append([x1, y1_full, x2, y2_full, 'yellow'])
                    self._last_yellow_left_line = [x1, y1_full, x2, y2_full, 'yellow']
                    yellow_detected = True
                else:
                    right_lines.append([x1, y1_full, x2, y2_full, 'yellow'])
                    self._last_yellow_right_line = [x1, y1_full, x2, y2_full, 'yellow']
                    yellow_detected = True
        # 노란선이 한 번이라도 감지된 이후에는, 감지가 안 될 때까지 가상 실선 유지
        if not yellow_detected:
            if self._last_yellow_left_line is not None:
                left_lines.append(self._last_yellow_left_line)
            if self._last_yellow_right_line is not None:
                right_lines.append(self._last_yellow_right_line)
        self._last_left_lines = left_lines
        self._last_right_lines = right_lines

    def _extract_lane_waypoints(self, orig_image, n_points=10):
        height, width = orig_image.shape[:2]
        roi_y_start = self._last_roi_y_start if self._last_roi_y_start is not None else int(height * 0.5)
        y_samples = np.linspace(height-1, roi_y_start, n_points, dtype=int)
        left_xs = self._get_lane_xs(self._last_left_lines, y_samples)
        right_xs = self._get_lane_xs(self._last_right_lines, y_samples)
        waypoints = []
        lane_widths = []
        prev_cx = None
        prev_lane_width = None
        max_lane_width_change = 0.3  # 차로폭 변화 최대 비율(30%)
        max_cx_jump = int(width * 0.4)  # 웨이포인트 x좌표 급격한 변화 제한 (20% 프레임 폭)

        lane_number = self.get_lane_number()
        for i, y in enumerate(y_samples):
            # 각 y에서 좌/우 차선 후보 추출
            left_candidates = []
            right_candidates = []
            for x1, y1, x2, y2, color in self._last_left_lines:
                if abs(y1 - y) < 5 or abs(y2 - y) < 5:
                    left_candidates.append((x1, color))
                    left_candidates.append((x2, color))
            for x1, y1, x2, y2, color in self._last_right_lines:
                if abs(y1 - y) < 5 or abs(y2 - y) < 5:
                    right_candidates.append((x1, color))
                    right_candidates.append((x2, color))
            # 차로별로 좌/우 차선 선택
            if lane_number == 1:
                # 왼쪽: 흰선, 오른쪽: 노란선
                lx = np.mean([x for x, c in left_candidates if c == 'white']) if any(c == 'white' for _, c in left_candidates) else None
                rx = np.mean([x for x, c in right_candidates if c == 'yellow']) if any(c == 'yellow' for _, c in right_candidates) else None
            elif lane_number == 2:
                # 왼쪽: 노란선, 오른쪽: 흰선
                lx = np.mean([x for x, c in left_candidates if c == 'yellow']) if any(c == 'yellow' for _, c in left_candidates) else None
                rx = np.mean([x for x, c in right_candidates if c == 'white']) if any(c == 'white' for _, c in right_candidates) else None
            else:
                # fallback: 기존 방식
                lx = np.mean([x for x, _ in left_candidates]) if left_candidates else None
                rx = np.mean([x for x, _ in right_candidates]) if right_candidates else None
            # 이후 중앙값 계산 및 보간 로직 동일
            cx = None
            lane_width = None

            if lx is not None and rx is not None:
                lane_width = abs(rx - lx)
                # 차로폭 변화 제한
                if prev_lane_width is not None:
                    min_width = prev_lane_width * (1 - max_lane_width_change)
                    max_width = prev_lane_width * (1 + max_lane_width_change)
                    lane_width = np.clip(lane_width, min_width, max_width)
                cx = int((lx + rx) / 2)
                lane_widths.append(lane_width)
                prev_lane_width = lane_width
            elif lx is not None:
                lane_width = int(np.mean(lane_widths)) if lane_widths else int(width * 0.5)
                cx = int(lx + lane_width // 2)
            elif rx is not None:
                lane_width = int(np.mean(lane_widths)) if lane_widths else int(width * 0.5)
                cx = int(rx - lane_width // 2)
            else:
                # 차선 미검출 구간: 이전 웨이포인트 보간
                if prev_cx is not None:
                    cx = prev_cx  # 이전 값을 그대로 사용 (혹은 보간)
                else:
                    cx = width // 2  # 완전 미검출 시 중앙값
            # 웨이포인트 x좌표 급격한 변화 제한
            if prev_cx is not None and abs(cx - prev_cx) > max_cx_jump:
                cx = prev_cx + np.sign(cx - prev_cx) * max_cx_jump
            waypoints.append((cx, y))
            prev_cx = cx
        return waypoints

    def _get_lane_xs(self, lines, y_samples):
        xs = []
        for y in y_samples:
            x_candidates = []
            for x1, y1, x2, y2, _ in lines:
                if y1 == y2:
                    continue
                if (y1 <= y <= y2) or (y2 <= y <= y1):
                    x = int(x1 + (x2 - x1) * (y - y1) / (y2 - y1))
                    x_candidates.append(x)
            if x_candidates:
                xs.append(int(np.mean(x_candidates)))
            else:
                xs.append(None)
        return xs

    def _draw_waypoints_and_spline(self, orig_image, waypoints):
        height, width = orig_image.shape[:2]
        vis_img = np.zeros((height, width, 3), dtype=np.uint8)
        # waypoint 시각화 (노랑)
        for (x, y) in waypoints:
            cv2.circle(vis_img, (x, y), 5, (0,255,255), -1)
        # Cubic Spline 곡선 시각화 (빨강)
        if len(waypoints) >= 4:
            # y값 기준 오름차순 정렬
            pts = np.array(sorted(waypoints, key=lambda p: p[1]))
            cs = CubicSpline(pts[:,1], pts[:,0])
            y_curve = np.linspace(pts[:,1].min(), pts[:,1].max(), 100)
            x_curve = cs(y_curve)
            for i in range(len(y_curve)-1):
                pt1 = (int(x_curve[i]), int(y_curve[i]))
                pt2 = (int(x_curve[i+1]), int(y_curve[i+1]))
                cv2.line(vis_img, pt1, pt2, (0,0,255), 2)
        return vis_img

    def _draw_lane_lines(self, orig_image):
        """
        검출된 차선(흰색, 노란색)을 원본 이미지 위에 시각화하여 반환
        Returns:
            vis_img: 차선이 그려진 이미지(BGR)
        """
        vis_img = orig_image.copy()
        # 왼쪽 차선(노란/흰) 그리기
        for x1, y1, x2, y2, color in self._last_left_lines:
            if color == 'yellow':
                cv2.line(vis_img, (x1, y1), (x2, y2), (0, 0, 0), 4)  # 검은색
            else:
                cv2.line(vis_img, (x1, y1), (x2, y2), (0, 0, 0), 4)  # 검은색
        # 오른쪽 차선(노란/흰) 그리기
        for x1, y1, x2, y2, color in self._last_right_lines:
            if color == 'yellow':
                cv2.line(vis_img, (x1, y1), (x2, y2), (0, 0, 0), 4)
            else:
                cv2.line(vis_img, (x1, y1), (x2, y2), (0, 0, 0), 4) # 검은색
        return vis_img

    def show_lane_info(self, orig_image):
        """
        원본 이미지 위에 검출된 waypoint(노랑)와 Cubic Spline(빨강), 그리고 추종된 차선(노랑/흰)을 모두 시각화 (통합)
        Args:
            orig_image: 원본 BGR 이미지
        """
        vis_img = orig_image.copy()
        # 1. 추종된 차선(노랑/흰) 시각화
        for x1, y1, x2, y2, color in self._last_left_lines:
            if color == 'yellow':
                cv2.line(vis_img, (x1, y1), (x2, y2), (0, 255, 255), 4)
            else:
                cv2.line(vis_img, (x1, y1), (x2, y2), (255, 255, 255), 4)
        for x1, y1, x2, y2, color in self._last_right_lines:
            if color == 'yellow':
                cv2.line(vis_img, (x1, y1), (x2, y2), (0, 255, 255), 4)
            else:
                cv2.line(vis_img, (x1, y1), (x2, y2), (255, 255, 255), 4)
        # 2. waypoint(노랑) 시각화
        waypoints = self._last_waypoints if self._last_waypoints else self._extract_lane_waypoints(orig_image)
        for (x, y) in waypoints:
            cv2.circle(vis_img, (x, y), 5, (0,255,255), -1)
        # 3. Cubic Spline(빨강) 시각화
        if len(waypoints) >= 4:
            pts = np.array(sorted(waypoints, key=lambda p: p[1]))
            cs = CubicSpline(pts[:,1], pts[:,0])
            y_curve = np.linspace(pts[:,1].min(), pts[:,1].max(), 100)
            x_curve = cs(y_curve)
            for i in range(len(y_curve)-1):
                pt1 = (int(x_curve[i]), int(y_curve[i]))
                pt2 = (int(x_curve[i+1]), int(y_curve[i+1]))
                cv2.line(vis_img, pt1, pt2, (0,0,255), 2)
        cv2.imshow('lane_info_all', vis_img)
        cv2.waitKey(1)

    def show_lane_lines(self, orig_image):
        """
        검출된 차선(노란/흰)만 원본 이미지 위에 시각화 (별도 창)
        """
        vis_img = self._draw_lane_lines(orig_image)
        cv2.imshow('detected_lanes', vis_img)
        cv2.waitKey(1)

    def get_lane_waypoints(self):
        """waypoint 리스트 반환"""
        return self._last_waypoints

    def get_processed_image(self):
        return self.processed_image

    def get_valid_waypoints(self):
        """
        3개 이상이면 현재값, 아니면 마지막 정상값 반환
        """
        if len(self._last_waypoints) >= 3:
            return self._last_waypoints
        else:
            return self._last_valid_waypoints

    def process_lidar(self, ranges):
        """
        라이다 데이터 처리용 인터페이스 (임시)
        Args:
            ranges: 라이다 센서 데이터
        Returns:
            obstacle_data: 장애물 정보 등 (현재 None 반환)
        """
        return None

    def _calculate_steering_control(self, lane_data):
        if lane_data is None or not isinstance(lane_data, list) or len(lane_data) < 3:
            return 0.0
        sorted_wps = sorted(lane_data, key=lambda p: p[1], reverse=True)
        target_wp = sorted_wps[2]
        # ...

    def get_lane_number(self):
        """
        트랙 환경(흰-노-흰)에서 ego 차량의 차로 번호 추정
        Returns:
            int or str: 1(왼쪽 차로), 2(오른쪽 차로), 'unknown'
        """
        # 이미지 중심 x좌표
        width = self.processed_image.shape[1] if self.processed_image is not None else 640
        center_x = width // 2

        # 노란 점선(중앙선) 후보들만 추출
        yellow_lines = [line for line in self._last_left_lines + self._last_right_lines if line[-1] == 'yellow']
        if not yellow_lines:
            return 'unknown'

        # 노란 점선의 평균 x좌표 계산 (여러 개 검출될 수 있음)
        yellow_xs = []
        for x1, y1, x2, y2, _ in yellow_lines:
            yellow_xs.append(x1)
            yellow_xs.append(x2)
        yellow_center = int(np.mean(yellow_xs))

        if center_x < yellow_center:
            return 1  # 1차로 (왼쪽)
        else:
            return 2  # 2차로 (오른쪽)

perception = Perception()
# ... (이미지 처리 및 차선 인식 코드)

lane_number = perception.get_lane_number()
print(f"현재 차로 번호: {lane_number}")
