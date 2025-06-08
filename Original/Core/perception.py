#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from scipy.interpolate import CubicSpline
import sys
sys.path.append('/home/xytron/xycar_ws/src/kookmin/driver/Original/cloudsonnet/test')
from lane_detector import LaneDetector
from obstacle_detector import ObstacleDetector

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
        self.lane_detector = LaneDetector()
        self.obstacle_detector = ObstacleDetector()

    def process_image(self, image):
        if image is None or image.size == 0:
            return None, image
        # LaneDetector 모듈 사용
        left_lane_points, right_lane_points, vis_img = self.lane_detector.detect_lanes(image)
        lane_data = {
            'left_lane_points': left_lane_points,
            'right_lane_points': right_lane_points
        }
        return lane_data, vis_img

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
        # 기존 CubicSpline 기반 중심선 시각화 코드 제거
        return vis_img

    def generate_centerline_spline(self, waypoints):
        """
        주어진 웨이포인트로부터 중심선(스플라인) 생성
        Returns:
            cs_x: CubicSpline (y -> x)
            cs_y: CubicSpline (s -> y)
            s_samples: 아크길이 누적값 배열
        """
        if len(waypoints) < 4:
            return None, None, None
        pts = np.array(sorted(waypoints, key=lambda p: p[1]))
        y = pts[:,1]
        x = pts[:,0]
        # y 기준 CubicSpline (y->x)
        cs_x = CubicSpline(y, x)
        # 아크길이 s 계산
        s_samples = np.zeros_like(y, dtype=np.float32)
        for i in range(1, len(y)):
            dx = x[i] - x[i-1]
            dy = y[i] - y[i-1]
            ds = np.hypot(dx, dy)
            s_samples[i] = s_samples[i-1] + ds
        # s 기준 y CubicSpline (s->y)
        cs_y = CubicSpline(s_samples, y)
        return cs_x, cs_y, s_samples

    def project_point_to_centerline(self, cs_x, cs_y, s_samples, px, py):
        """
        차량 위치(px, py)를 중심선 스플라인에 투영하여 s(아크길이), q(횡방향 오프셋) 반환
        Returns:
            s_proj: 투영점의 아크길이
            q: 횡방향 오프셋(좌우 거리)
        """
        if cs_x is None or cs_y is None or s_samples is None:
            return None, None
        # s 범위 샘플링
        s_dense = np.linspace(s_samples[0], s_samples[-1], 200)
        y_dense = cs_y(s_dense)
        x_dense = cs_x(y_dense)
        # 각 샘플에 대해 거리 계산
        dists = np.hypot(x_dense - px, y_dense - py)
        min_idx = np.argmin(dists)
        s_proj = s_dense[min_idx]
        x_proj = x_dense[min_idx]
        y_proj = y_dense[min_idx]
        # 중심선의 접선 벡터 계산
        if min_idx < len(s_dense) - 1:
            dx = x_dense[min_idx+1] - x_dense[min_idx]
            dy = y_dense[min_idx+1] - y_dense[min_idx]
        else:
            dx = x_dense[min_idx] - x_dense[min_idx-1]
            dy = y_dense[min_idx] - y_dense[min_idx-1]
        tangent = np.array([dx, dy])
        tangent = tangent / (np.linalg.norm(tangent) + 1e-6)
        # 투영점에서 차량 위치로의 벡터
        vec = np.array([px - x_proj, py - y_proj])
        # 횡방향 오프셋(q): 접선에 수직인 방향으로의 거리
        normal = np.array([-tangent[1], tangent[0]])
        q = np.dot(vec, normal)
        return s_proj, q

    # 예시: 이미지 처리 후 중심선 및 s-q 좌표 계산
    def process_image_with_centerline(self, image, vehicle_pos=None):
        """
        이미지 처리 + 중심선 생성 + (선택) 차량 위치의 s-q 좌표 반환
        vehicle_pos: (x, y) 픽셀좌표 (옵션)
        Returns:
            waypoints, vis_img, (s, q) or None
        """
        if image is None or image.size == 0:
            return None, image, None
        self.processed_image = self._preprocess_image(image)
        self._extract_lane_lines(self.processed_image, image)
        waypoints = self._extract_lane_waypoints(image)
        if len(waypoints) >= 3:
            self._last_valid_waypoints = waypoints
        self._last_waypoints = waypoints
        cs_x, cs_y, s_samples = self.generate_centerline_spline(waypoints)
        sq = None
        if vehicle_pos is not None and cs_x is not None:
            s, q = self.project_point_to_centerline(cs_x, cs_y, s_samples, vehicle_pos[0], vehicle_pos[1])
            sq = (s, q)
        vis_img = self._draw_waypoints_and_spline(image, waypoints)
        return waypoints, vis_img, sq

    def process_lidar(self, scan_msg):
        if scan_msg is None:
            return None
        # scan_msg가 tuple/array라면, LaserScan 메시지처럼 변환
        if isinstance(scan_msg, (tuple, list, np.ndarray)):
            class FakeScan:
                pass
            fake = FakeScan()
            fake.ranges = np.array(scan_msg)
            fake.angle_min = 0.0
            fake.angle_max = 2 * np.pi
            fake.range_min = 0.1
            fake.range_max = 20.0
            scan_msg = fake
        obstacles = self.obstacle_detector.process_lidar_scan(scan_msg)
        # 예시: 전방 장애물까지의 최소 거리 추출
        front_distance = float('inf')
        for obs in obstacles:
            # 차량 전방(0도 부근) 기준, x축이 전방이라고 가정
            if hasattr(obs, 'position') and obs.position[0] > 0 and abs(obs.position[1]) < 1.0:
                dist = np.linalg.norm(obs.position)
                if dist < front_distance:
                    front_distance = dist
        return {'obstacles': obstacles, 'front_distance': front_distance}

    def show_lane_info(self, image):
        # 차선 포인트 및 웨이포인트를 시각화하여 별도 창에 표시
        lane_data, vis_img = self.process_image(image)
        if vis_img is not None:
            import cv2
            cv2.imshow("Lane Info", vis_img)
            cv2.waitKey(1)
