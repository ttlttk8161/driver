# 인지 모듈
import logging
import queue
import _queue
import threading
import cv2
import numpy as np
import time
from .optimized_data_structures import SensorData, LaneMarking
from .optimized_data_structures import OptimizedSensorInput, OptimizedPerceptionOutput, PerformanceMetrics, monitor_performance, WhiteLineHsvMetrics, YellowLineHsvMetrics

logger = logging.getLogger(__name__)

class PerceptionModule:
    def __init__(self, config: dict, 
                 input_queue_sensor_data: queue.Queue, 
                 output_queues: dict):
        """인지 모듈 초기화"""
        self.config = config if config is not None else {}
        self.input_queue_sensor_data = input_queue_sensor_data
        self.output_queues = output_queues
        
        detection_config = self.config.get('detection', {})
        
        self.active_perception_algorithm = detection_config.get('active_perception_algorithm')
        
        # 각 알고리즘 파라미터 저장
        self.params = {}
        self.params['hsv_lane_detection'] = detection_config.get('hsv_lane_detection_params', {})
        self.params['canny_hough_lane_detection'] = detection_config.get('canny_hough_lane_detection_params', {})
        self.params['custom_block_example'] = detection_config.get('custom_block_example_params', {})

        self._running = False
        self._thread = None
        logger.info(f"PerceptionModule initialized. Active algorithm: {self.active_perception_algorithm}")

    @monitor_performance
    def _process_sensor_data(self, sensor_input) -> OptimizedPerceptionOutput:
        """
        active_perception_algorithm에 따라 센서 데이터를 처리하여 OptimizedPerceptionOutput을 생성합니다.
        Args:
            sensor_input: 처리할 센서 데이터 (SensorData 또는 OptimizedSensorInput).
        Returns:
            OptimizedPerceptionOutput: 최적화된 인식 결과 데이터 구조.
        """
        print(f"PerceptionModule: Processing sensor input - timestamp: {getattr(sensor_input, 'timestamp', 'unknown')}")
        
        # 입력 데이터 타입 확인 및 호환성 처리
        if isinstance(sensor_input, OptimizedSensorInput):
            image = sensor_input.camera_data
            timestamp = sensor_input.timestamp
            sequence_id = sensor_input.sequence_id
        else:  # 기존 SensorData 호환성
            image = sensor_input.vision_data
            timestamp = sensor_input.timestamp
            sequence_id = 0
        
        start_time = time.time()
        
        # 알고리즘 이름과 해당 실행 메소드를 매핑합니다.
        # 새로운 알고리즘을 추가할 때 이 사전에 추가하면 됩니다.
        algorithm_map = {
            "hsv_lane_detection": self._execute_hsv_lane_detection,
            "canny_hough_lane_detection": self._execute_canny_hough_lane_detection,
            "custom_block_example": self._execute_custom_block_example,
            # 예: "my_new_algorithm": self._execute_my_new_algorithm,
        }

        if self.active_perception_algorithm and self.active_perception_algorithm in algorithm_map:
            selected_method = algorithm_map[self.active_perception_algorithm]
            algorithm_params = self.params.get(self.active_perception_algorithm, {})
            # logger.debug(f"Executing {self.active_perception_algorithm} with params: {algorithm_params}") # 너무 빈번할 수 있음
            # 각 알고리즘 메소드는 (detected_objects, lane_markings, ...) 등을 포함하는 튜플이나 딕셔너리를 반환해야 함
            # 여기서는 플레이스홀더이므로, 기본 OptimizedPerceptionOutput을 반환하도록 수정
            if image is not None: # 이미지가 있을 때만 알고리즘 실행
                algo_output_dict = selected_method(image, algorithm_params)
            else:
                algo_output_dict = {} # 이미지가 없으면 빈 결과
            
            processing_time_ms = (time.time() - start_time) * 1000
            
            # 최적화된 출력 생성
            return OptimizedPerceptionOutput(
                timestamp=timestamp,
                sequence_id=sequence_id,
                processing_time_ms=processing_time_ms,
                white_line_metrics=algo_output_dict.get("white_line_hsv_metrics"),
                yellow_line_metrics=algo_output_dict.get("yellow_line_hsv_metrics"),
                detected_objects=algo_output_dict.get("detected_objects", []),
                detection_confidence=algo_output_dict.get("detection_confidence", 0.0),
                quality_score=algo_output_dict.get("quality_score", 0.0)
            )
        elif self.active_perception_algorithm is None:
            logger.warning("PerceptionModule: Active perception algorithm is None. 작업이 수행되지 않습니다.")
        else:
            logger.warning(f"알 수 없거나 지원되지 않는 인식 알고리즘이 선택되었습니다: {self.active_perception_algorithm}")
            logger.warning("PerceptionModule: 작업이 수행되지 않습니다.")
        
        # 알고리즘이 선택되지 않았거나, 알 수 없는 경우 기본 빈 OptimizedPerceptionOutput 반환
        processing_time_ms = (time.time() - start_time) * 1000
        
        return OptimizedPerceptionOutput(
            timestamp=timestamp,
            sequence_id=sequence_id,
            processing_time_ms=processing_time_ms
        )

    @monitor_performance
    def _execute_hsv_lane_detection(self, image, params) -> dict:
        """
        HSV 차선 감지 알고리즘 예시 플레이스홀더입니다.
        Args:
            image: 입력 이미지입니다.
            params (dict): 이 알고리즘을 위한 파라미터입니다 (예: hsv_lane_detection_params).
        Returns:
            dict: 인식 결과를 담은 딕셔너리 (예: {"white_line_hsv_metrics": WhiteLineHsvMetrics(...), ...})
        """
        if image is None: return {}
        # logger.debug(f"Running _execute_hsv_lane_detection with params: {params}")        
        # 실제 HSV 차선 감지 로직 (예시, steering_balancing.py의 일부 로직 참고)
        # ROI 설정
        roi_y_start = int(image.shape[0] * params.get("roi_y_start_ratio", 0.2))
        roi = image[roi_y_start:, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # 흰색 차선
        lower_white = np.array(params.get("lower_white_hsv", [0,0,180]))
        upper_white = np.array(params.get("upper_white_hsv", [180,30,255]))
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # 노란색 차선
        lower_yellow = np.array(params.get("lower_yellow_hsv", [20,100,100]))
        upper_yellow = np.array(params.get("upper_yellow_hsv", [30,255,255]))
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # WhiteLineHsvMetrics 계산 (간단화된 예시)
        total_white_pixels = np.sum(white_mask > 0)
        w_metrics = WhiteLineHsvMetrics(
            timestamp=time.time(), total_white_pixels=total_white_pixels,
            left_ratio=0.0, mid_ratio=0.0, right_ratio=0.0, # 실제 계산 필요
            is_detected=total_white_pixels > params.get("white_pixel_threshold", 300)
        )

        # YellowLineHsvMetrics 계산 (간단화된 예시)
        moments_yellow = cv2.moments(yellow_mask)
        y_area = moments_yellow['m00']
        y_center_x = int(moments_yellow['m10'] / y_area) if y_area > 0 else None
        y_metrics = YellowLineHsvMetrics(
            timestamp=time.time(), area=y_area, center_x=y_center_x,
            is_detected=y_area > params.get("yellow_area_threshold", 100)
        )

        if params.get("debug_cv_show", False):
            cv2.imshow("HSV White Mask", white_mask)
            cv2.imshow("HSV Yellow Mask", yellow_mask)
            cv2.waitKey(1)

        return {
            "white_line_hsv_metrics": w_metrics,
            "yellow_line_hsv_metrics": y_metrics
        }

    def _execute_canny_hough_lane_detection(self, image, params) -> dict:
        """
        Canny Hough 차선 감지 알고리즘 예시 플레이스홀더입니다.
        """
        if image is None: return {}
        
        height, width = image.shape[:2]
        
        # 1. Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 2. Apply Gaussian Blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 3. Apply Canny Edge Detection
        canny_low = params.get("canny_low_threshold", 50)
        canny_high = params.get("canny_high_threshold", 150)
        edges = cv2.Canny(blurred, canny_low, canny_high)
        
        # 4. Define and Apply ROI
        roi_y_start_ratio = params.get("roi_y_start_ratio", 0.5)
        roi_y_start = int(height * roi_y_start_ratio)
        
        mask = np.zeros_like(edges)
        roi_poly_vertices = np.array([[(0, roi_y_start), (width, roi_y_start), (width, height), (0, height)]], dtype=np.int32)
        cv2.fillPoly(mask, roi_poly_vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # 5. Apply Hough Transform
        hough_threshold = params.get("hough_threshold", 20)
        min_line_length = params.get("hough_min_line_length", 10)
        max_line_gap = params.get("hough_max_line_gap", 5)
        
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, hough_threshold,
                                minLineLength=min_line_length, maxLineGap=max_line_gap)
        
        lane_markings_list = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                lane_marking = LaneMarking(
                    points=[(float(x1), float(y1)), (float(x2), float(y2))],
                    type="detected_line_segment", # 차후 개선 가능
                    confidence=1.0 # 임시 신뢰도
                )
                lane_markings_list.append(lane_marking)

        if params.get("debug_cv_show", False):
            debug_image = image.copy()
            if lines is not None:
                for line_segment in lines: # 변수명 변경
                    x1, y1, x2, y2 = line_segment[0]
                    cv2.line(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.polylines(debug_image, [roi_poly_vertices], isClosed=True, color=(0,0,255), thickness=2)
            cv2.imshow("Canny-Hough Debug", debug_image)
            # cv2.imshow("Canny Edges (Full)", edges) # 디버깅 시 필요하면 활성화
            # cv2.imshow("Masked Edges (ROI)", masked_edges) # 디버깅 시 필요하면 활성화
            cv2.waitKey(1)
        
        logger.debug(f"CannyHough: Detected {len(lane_markings_list)} line segments.")
        return {"lane_markings": lane_markings_list}

    def _execute_custom_block_example(self, image, params) -> dict:
        """
        사용자 정의 알고리즘 블록 예시 플레이스홀더입니다.
        """
        # logger.debug(f"Running _execute_custom_block_example with params: {params}")
        if image is None: return {}
        # 여기에 실제 사용자 정의 로직을 구현합니다.
        return {} # 빈 결과 반환

    def run(self):
        print("[PERCEPTION] run() 진입", flush=True)
        logger.info(f"PerceptionModule: Thread started. Active algorithm: {self.active_perception_algorithm}")
        while self._running:
            try:
                sensor_data = self.input_queue_sensor_data.get(timeout=1.0)
                print(f"[PERCEPTION] sensor_data type: {type(sensor_data)}, value: {repr(sensor_data)}", flush=True)
                print(f"[PERCEPTION] input_queue_sensor_data type: {type(self.input_queue_sensor_data)}", flush=True)
                print(f"PerceptionModule: Got sensor data from queue (timestamp={getattr(sensor_data, 'timestamp', 'unknown')})", flush=True)
                logger.info(f"PerceptionModule: Got sensor data from queue (timestamp={getattr(sensor_data, 'timestamp', 'unknown')})")
                perception_output = self._process_sensor_data(sensor_data)
                for key, q in self.output_queues.items():
                    try:
                        q.put(perception_output, timeout=0.1)
                        print(f"PerceptionModule: Put perception output to '{key}' queue (timestamp={perception_output.timestamp})", flush=True)
                        logger.info(f"PerceptionModule: Put perception output to '{key}' queue (timestamp={perception_output.timestamp})")
                    except queue.Full:
                        logger.warning(f"PerceptionModule: Output queue '{key}' is full. Discarding data.")
                self.input_queue_sensor_data.task_done()
            except (queue.Empty, _queue.Empty):
                if not self._running:
                    break
                logger.info("PerceptionModule: Waiting for sensor data in queue...")
            except Exception as e:
                logger.error(f"PerceptionModule: Error processing sensor data: {e}", exc_info=True)
        logger.info("PerceptionModule: Thread stopped.")

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self.run, name="PerceptionThread")
            self._thread.start()
            logger.info("PerceptionModule: Started.")

    def stop(self):
        if self._running:
            self._running = False
            if self._thread:
                self._thread.join(timeout=2.0)
            logger.info("PerceptionModule: Stopped.")