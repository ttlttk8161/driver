import queue
import threading
import time
from typing import Dict, List, Tuple, Any, Optional
from .data_structures import (
    SensorData, PerceptionOutput, DetectedObject, LaneMarking, TrafficSignInfo
)
import cv2
import numpy as np

class DetectionComponent:
    def __init__(self, config: dict):
        self.config = config
        print("DetectionComponent: Initialized.")

    def process(self, sensor_data: SensorData, depth_map: Optional[Any]) -> Tuple[List[DetectedObject], List[LaneMarking], Any, List[TrafficSignInfo]]:
        # print(f"DetectionComponent: Processing sensor data at {sensor_data.timestamp}")
        
        lane_markings = []
        if sensor_data.vision_data is not None:
            image = sensor_data.vision_data
            height, width = image.shape[:2]

            # 1. 그레이스케일 변환
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # 2. 가우시안 블러
            blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

            # 3. 캐니 엣지 검출
            # TODO: config에서 임계값 가져오기
            canny_low_threshold = self.config.get("canny_low_threshold", 50)
            canny_high_threshold = self.config.get("canny_high_threshold", 150)
            edges_image = cv2.Canny(blurred_image, canny_low_threshold, canny_high_threshold)

            # 4. ROI (Region of Interest) 설정
            # ROI를 사다리꼴 모양으로 정의 (이미지 하단 중앙 영역)
            # TODO: config에서 ROI 좌표 가져오기
            roi_vertices = np.array([
                [width * 0.1, height],          # 왼쪽 아래
                [width * 0.4, height * 0.6],    # 왼쪽 위
                [width * 0.6, height * 0.6],    # 오른쪽 위
                [width * 0.9, height]           # 오른쪽 아래
            ], dtype=np.int32)
            
            mask = np.zeros_like(edges_image)
            cv2.fillPoly(mask, [roi_vertices], 255)
            roi_edges_image = cv2.bitwise_and(edges_image, mask)

            # 5. 허프 변환으로 직선 검출
            # TODO: config에서 허프 변환 파라미터 가져오기
            lines = cv2.HoughLinesP(roi_edges_image, rho=1, theta=np.pi/180, threshold=25, minLineLength=20, maxLineGap=50)

            left_lines = []  # (slope, intercept, x1, y1, x2, y2)
            right_lines = [] # (slope, intercept, x1, y1, x2, y2)

            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    if x2 - x1 == 0: # 수직선 방지 (기울기 무한대)
                        slope = np.inf if y2 > y1 else -np.inf
                    else:
                        slope = (y2 - y1) / (x2 - x1)
                    
                    # 기울기 필터링 (너무 수평이거나 수직인 선 제외)
                    if abs(slope) < 0.3 or abs(slope) > 2.0: # TODO: config 값 사용
                        continue

                    intercept = y1 - slope * x1
                    
                    # 차선 위치 (이미지 중앙 기준) 및 기울기 부호로 좌우 구분
                    if slope < 0 and x1 < width / 2 and x2 < width / 2: # 왼쪽 차선 후보 (기울기 음수)
                        left_lines.append((slope, intercept, x1, y1, x2, y2))
                    elif slope > 0 and x1 > width / 2 and x2 > width / 2: # 오른쪽 차선 후보 (기울기 양수)
                        right_lines.append((slope, intercept, x1, y1, x2, y2))

            # 대표 차선 계산 (평균)
            def average_lines(lines_data, existing_height):
                if not lines_data:
                    return None
                
                avg_slope = np.mean([ld[0] for ld in lines_data])
                avg_intercept = np.mean([ld[1] for ld in lines_data])
                
                # y 좌표를 기준으로 x 좌표 계산 (ROI 하단과 ROI 상단 중간쯤)
                y1_new = existing_height # 이미지 하단
                y2_new = existing_height * 0.7 # 이미지 하단에서 70% 지점
                
                if avg_slope == 0: return None # 수평선이면 무시

                x1_new = int((y1_new - avg_intercept) / avg_slope)
                x2_new = int((y2_new - avg_intercept) / avg_slope)
                
                return [(x1_new, int(y1_new)), (x2_new, int(y2_new))]

            left_lane_points = average_lines(left_lines, height)
            if left_lane_points:
                lane_markings.append(LaneMarking(points=left_lane_points, type="left_lane", confidence=0.8))
            
            right_lane_points = average_lines(right_lines, height)
            if right_lane_points:
                lane_markings.append(LaneMarking(points=right_lane_points, type="right_lane", confidence=0.8))

        # 다른 객체들은 더미 데이터 유지
        detected_objects = [
            # DetectedObject(id=1, type="car", position_3d=(10.0, 2.0, 0.0), bounding_box_2d=(100,100,150,150) ,velocity=(1.0,0.0,0.0), confidence=0.9, tracked_history=[], predicted_trajectory_short_term=[])
        ]
        drivable_area_mask = "Sample Drivable Area Mask" # Placeholder
        traffic_signs = [
            # TrafficSignInfo(type="stop_sign", position_3d=(20.0, 5.0, 1.5), confidence=0.8)
        ]
        return detected_objects, lane_markings, drivable_area_mask, traffic_signs

class SceneUnderstandingComponent:
    def __init__(self, config: dict):
        self.config = config
        print("SceneUnderstandingComponent: Initialized.")

    def process(self, sensor_data: SensorData, previous_frame_data: Optional[Any] = None) -> Tuple[Optional[Any], Optional[Any], Optional[Any], Optional[Any], Optional[Any]]:
        # Placeholder for Segmentation, Depth, Optical Flow, Scene Flow (Fig 3)
        print(f"SceneUnderstandingComponent: Processing sensor data at {sensor_data.timestamp}")
        semantic_map = "Sample Semantic Map"
        instance_map = "Sample Instance Map"
        depth_map_output = "Sample Depth Map from SceneUnderstanding"
        optical_flow = "Sample Optical Flow"
        scene_flow = "Sample Scene Flow"
        return semantic_map, instance_map, depth_map_output, optical_flow, scene_flow

class TrackingComponent:
    def __init__(self, config: dict):
        self.config = config
        self._tracked_objects_state: Dict[int, DetectedObject] = {} # Internal state for tracking
        print("TrackingComponent: Initialized.")

    def process(self, detected_objects: List[DetectedObject], timestamp: float) -> List[DetectedObject]:
        # Placeholder for object tracking logic (Fig 3: Traditional, Neural Network Tracking)
        print(f"TrackingComponent: Processing {len(detected_objects)} objects at {timestamp}")
        # Simple ID-based tracking for now or update existing tracks
        updated_tracked_objects = []
        for obj in detected_objects:
            # Add tracking logic here (e.g., Kalman filter, data association)
            # For now, just pass through with updated timestamp if needed
            updated_obj = obj._replace(tracked_history=(obj.tracked_history or []) + [obj.position_3d])
            updated_tracked_objects.append(updated_obj)
        return updated_tracked_objects

class PerceptionPredictionComponent: # Short-term trajectory for detected objects
    def __init__(self, config: dict):
        self.config = config
        print("PerceptionPredictionComponent: Initialized.")

    def process(self, tracked_objects: List[DetectedObject]) -> List[DetectedObject]:
        # Placeholder for short-term prediction (Fig 3: Model-based, Data-driven Prediction)
        print(f"PerceptionPredictionComponent: Predicting for {len(tracked_objects)} objects.")
        predicted_objects = []
        for obj in tracked_objects:
            # Simple prediction: current position + velocity * dt
            predicted_traj = []
            if obj.velocity:
                current_pos = obj.position_3d
                for dt_step in [0.1, 0.2, 0.3]: # Predict for next 0.3 seconds
                    predicted_pos = (current_pos[0] + obj.velocity[0]*dt_step,
                                     current_pos[1] + obj.velocity[1]*dt_step,
                                     current_pos[2] + obj.velocity[2]*dt_step)
                    predicted_traj.append(predicted_pos)
            predicted_obj = obj._replace(predicted_trajectory_short_term=predicted_traj)
            predicted_objects.append(predicted_obj)
        return predicted_objects

class PerceptionModule:
    def __init__(self, config: dict, input_queue: queue.Queue, output_queues: Dict[str, queue.Queue]):
        self.config = config
        self.input_queue = input_queue
        self.output_queues = output_queues  # e.g., {"localization": q1, "prediction": q2, "planning": q3}

        self.detection_comp = DetectionComponent(config.get("detection", {}))
        self.scene_understanding_comp = SceneUnderstandingComponent(config.get("scene_understanding", {}))
        self.tracking_comp = TrackingComponent(config.get("tracking", {}))
        self.perception_prediction_comp = PerceptionPredictionComponent(config.get("perception_prediction", {}))

        self._running = False
        self._thread = None
        self._previous_frame_data = None # For temporal tasks like optical flow
        print("PerceptionModule: Initialized.")

    def _process_frame(self, sensor_data: SensorData) -> PerceptionOutput:
        print(f"PerceptionModule: Processing frame {sensor_data.timestamp}")

        # 1. Scene Understanding (e.g., depth might be used by detection)
        semantic_map, instance_map, depth_map, optical_flow, scene_flow = \
            self.scene_understanding_comp.process(sensor_data, self._previous_frame_data)

        # 2. Detection
        detected_objects_raw, lane_markings, drivable_area_mask, traffic_signs = \
            self.detection_comp.process(sensor_data, depth_map) # Pass depth_map if needed

        # 3. Tracking
        tracked_objects = self.tracking_comp.process(detected_objects_raw, sensor_data.timestamp)

        # 4. Short-term Prediction (within Perception)
        final_objects_with_predictions = self.perception_prediction_comp.process(tracked_objects)

        # Store data for next frame if needed (e.g., for optical flow)
        self._previous_frame_data = {"vision": sensor_data.vision_data, "depth": depth_map}


        # 전체 인지 결과 준비
        perception_result = PerceptionOutput(
            timestamp=sensor_data.timestamp,
            detected_objects=final_objects_with_predictions,
            lane_markings=lane_markings,
            drivable_area_mask=drivable_area_mask,
            traffic_signs=traffic_signs,
            semantic_segmentation_map=semantic_map,
            instance_segmentation_map=instance_map,
            depth_map=depth_map, # This is the one from scene understanding
            optical_flow_map=optical_flow,
            scene_flow_map=scene_flow,
            raw_features_for_localization="Sample Features for SLAM" # Placeholder
        )
        return perception_result

    def run(self):
        print("PerceptionModule: Thread started.")
        while self._running:
            try:
                sensor_data: SensorData = self.input_queue.get(timeout=1.0)
                perception_output = self._process_frame(sensor_data)

                for key, q in self.output_queues.items():
                    try:
                        q.put(perception_output, timeout=0.1) # Non-blocking if possible or short timeout
                    except queue.Full:
                        print(f"PerceptionModule: Output queue '{key}' is full. Discarding data.")

                self.input_queue.task_done()
            except queue.Empty:
                if not self._running:
                    break # Exit if stopped and queue is empty
                continue # No data, loop again
            except Exception as e:
                print(f"PerceptionModule: Error processing frame: {e}")
                # Potentially add more robust error handling
        print("PerceptionModule: Thread stopped.")

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self.run, name="PerceptionThread")
            self._thread.start()
            print("PerceptionModule: Started.")

    def stop(self):
        if self._running:
            self._running = False # Signal thread to stop
            # self.input_queue.put(None) # Poison pill if blocking indefinitely on get
            if self._thread:
                self._thread.join(timeout=2.0) # Wait for thread to finish
            print("PerceptionModule: Stopped.")