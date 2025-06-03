import logging
import queue
import threading
import cv2 # For Canny/Hough example
import time
from .data_structures import SensorData, PerceptionOutput, WhiteLineHsvMetrics, YellowLineHsvMetrics # 필요한 데이터 구조 import

# 로깅 설정 (애플리케이션의 다른 부분에서 이미 설정되었을 수 있습니다)
# 예: logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PerceptionModule:
    def __init__(self, config: dict, 
                 input_queue_sensor_data: queue.Queue, 
                 output_queues: dict):
        """
        PerceptionModule을 초기화합니다.
        Args:
            config (dict): Perception 모듈 설정. 'detection' 설정을 포함합니다.
            input_queue_sensor_data (queue.Queue): SensorInputManager로부터 SensorData를 받는 큐.
            output_queues (dict): 처리된 PerceptionOutput을 전달할 출력 큐들의 딕셔너리.
                                  예: {"localization": queue_loc, "prediction": queue_pred, "planning": queue_plan}
        """
        self.config = config if config is not None else {}
        self.input_queue_sensor_data = input_queue_sensor_data
        self.output_queues = output_queues
        
        detection_config = self.config.get('detection', {})
        
        self.active_perception_algorithm = detection_config.get('active_perception_algorithm')
        # self.debug_cv_show = detection_config.get('debug_cv_show', False) # 각 알고리즘 파라미터로 이동됨
        
        
        # 각 알고리즘에 대한 파라미터를 저장합니다.
        self.params = {}
        # task.md에 명시된 파라미터 블록들을 로드합니다.
        # 실제 알고리즘 실행 시 해당 메소드에 전달됩니다.
        self.params['hsv_lane_detection'] = detection_config.get('hsv_lane_detection_params', {})
        self.params['canny_hough_lane_detection'] = detection_config.get('canny_hough_lane_detection_params', {})
        self.params['custom_block_example'] = detection_config.get('custom_block_example_params', {})
        # 새로운 알고리즘 "my_new_algorithm"의 경우 다음과 같이 추가할 수 있습니다:
        # self.params['my_new_algorithm'] = detection_config.get('my_new_algorithm_params', {})

        self._running = False
        self._thread = None
        logger.info(f"PerceptionModule initialized. Active algorithm: {self.active_perception_algorithm}")

    def _process_sensor_data(self, sensor_data: SensorData) -> PerceptionOutput:
        """
        active_perception_algorithm에 따라 센서 데이터를 처리하여 PerceptionOutput을 생성합니다.
        Args:
            sensor_data (SensorData): 처리할 센서 데이터 (이미지, 라이다 등 포함).
        Returns:
            PerceptionOutput: 인식 결과 데이터 구조.
        """
        image = sensor_data.vision_data # SensorData에서 이미지 추출
        timestamp = sensor_data.timestamp
        
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
            # 여기서는 플레이스홀더이므로, 기본 PerceptionOutput을 반환하도록 수정
            if image is not None: # 이미지가 있을 때만 알고리즘 실행
                algo_output_dict = selected_method(image, algorithm_params)
            else:
                algo_output_dict = {} # 이미지가 없으면 빈 결과
            
            # 예시: 알고리즘이 딕셔너리 형태로 white_line_hsv_metrics 등을 반환한다고 가정
            return PerceptionOutput(
                timestamp=timestamp,
                detected_objects=algo_output_dict.get("detected_objects", []),
                lane_markings=algo_output_dict.get("lane_markings", []),
                drivable_area_mask=algo_output_dict.get("drivable_area_mask"),
                traffic_signs=algo_output_dict.get("traffic_signs", []),
                semantic_segmentation_map=None, instance_segmentation_map=None, depth_map=None,
                optical_flow_map=None, scene_flow_map=None, raw_features_for_localization=None,
                white_line_hsv_metrics=algo_output_dict.get("white_line_hsv_metrics"),
                yellow_line_hsv_metrics=algo_output_dict.get("yellow_line_hsv_metrics")
            )
        elif self.active_perception_algorithm is None:
            logger.warning("PerceptionModule: Active perception algorithm is None. 작업이 수행되지 않습니다.")
        else:
            logger.warning(f"알 수 없거나 지원되지 않는 인식 알고리즘이 선택되었습니다: {self.active_perception_algorithm}")
            logger.warning("PerceptionModule: 작업이 수행되지 않습니다.")
        
        # 알고리즘이 선택되지 않았거나, 알 수 없는 경우 기본 빈 PerceptionOutput 반환
        return PerceptionOutput(
            timestamp=timestamp, detected_objects=[], lane_markings=[], drivable_area_mask=None,
            traffic_signs=[], semantic_segmentation_map=None, instance_segmentation_map=None,
            depth_map=None, optical_flow_map=None, scene_flow_map=None,
            raw_features_for_localization=None, white_line_hsv_metrics=None, yellow_line_hsv_metrics=None
        )

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
        # logger.debug(f"Running _execute_canny_hough_lane_detection with params: {params}")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, params.get("canny_low_threshold", 50), params.get("canny_high_threshold", 150))
        
        # ROI 설정 (Hough 변환 전에)
        # lines = cv2.HoughLinesP(edges, ...) # 실제 Hough 변환 로직

        if params.get("debug_cv_show", False):
            cv2.imshow("Canny Edges", edges)
            cv2.waitKey(1)
        return {} # 빈 결과 반환

    def _execute_custom_block_example(self, image, params) -> dict:
        """
        사용자 정의 알고리즘 블록 예시 플레이스홀더입니다.
        """
        # logger.debug(f"Running _execute_custom_block_example with params: {params}")
        if image is None: return {}
        # 여기에 실제 사용자 정의 로직을 구현합니다.
        return {} # 빈 결과 반환

    def run(self):
        logger.info(f"PerceptionModule: Thread started. Active algorithm: {self.active_perception_algorithm}")
        while self._running:
            try:
                sensor_data: SensorData = self.input_queue_sensor_data.get(timeout=1.0)
                perception_output = self._process_sensor_data(sensor_data)
                
                for key, q in self.output_queues.items():
                    try:
                        q.put(perception_output, timeout=0.1)
                    except queue.Full:
                        logger.warning(f"PerceptionModule: Output queue '{key}' is full. Discarding data.")
                self.input_queue_sensor_data.task_done()
            except queue.Empty:
                if not self._running:
                    break
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