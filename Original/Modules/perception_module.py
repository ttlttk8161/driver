import logging
import queue
import threading
import time
from .data_structures import SensorData, PerceptionOutput, WhiteLineHsvMetrics, YellowLineHsvMetrics # 필요한 데이터 구조 import
from .error_manager import error_manager, ErrorCode
from .Visualize import PerceptionVisualizer  # HSV 차선 감지 시각화를 위한 import

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
        self.debug_cv_show = detection_config.get('debug_cv_show', False) # cv2.imshow 사용 여부
        
        # HSV 차선 감지 시각화 초기화
        self.perception_visualizer = PerceptionVisualizer(debug_enabled=self.debug_cv_show)
        
        # 각 알고리즘에 대한 파라미터를 저장합니다.
        self.params = {}
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
            algo_output_dict = selected_method(image, algorithm_params) 
            
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
            logger.warning("작업을 수행하기 위한 모듈이 선택되지 않았습니다")
        else:
            logger.warning(f"알 수 없거나 지원되지 않는 인식 알고리즘이 선택되었습니다: {self.active_perception_algorithm}")
            logger.warning("작업을 수행하기 위한 모듈이 선택되지 않았습니다") # 또는 더 구체적인 메시지
        
        # 알고리즘이 선택되지 않았거나, 알 수 없는 경우 기본 빈 PerceptionOutput 반환
        return PerceptionOutput(
            timestamp=timestamp, detected_objects=[], lane_markings=[], drivable_area_mask=None,
            traffic_signs=[], semantic_segmentation_map=None, instance_segmentation_map=None,
            depth_map=None, optical_flow_map=None, scene_flow_map=None,
            raw_features_for_localization=None, white_line_hsv_metrics=None, yellow_line_hsv_metrics=None
        )

    def _execute_hsv_lane_detection(self, image, params) -> dict:
        """
        HSV 차선 감지 알고리즘 - steering_balancing.py 로직 기반
        Args:
            image: 입력 이미지입니다.
            params (dict): 이 알고리즘을 위한 파라미터입니다 (예: hsv_lane_detection_params).
        Returns:
            dict: 인식 결과를 담은 딕셔너리 (예: {"white_line_hsv_metrics": WhiteLineHsvMetrics(...), ...})
        """
        import cv2
        import numpy as np
        
        # 파라미터 읽기
        roi_y_start_ratio = params.get('roi_y_start_ratio', 0.6)
        lower_white_hsv = params.get('lower_white_hsv', [0, 0, 180])
        upper_white_hsv = params.get('upper_white_hsv', [180, 30, 255])
        lower_yellow_hsv = params.get('lower_yellow_hsv', [20, 100, 100])
        upper_yellow_hsv = params.get('upper_yellow_hsv', [30, 255, 255])
        white_pixel_threshold = params.get('white_pixel_threshold', 300)
        yellow_area_threshold = params.get('yellow_area_threshold', 100)
        
        # ROI 설정 (하단 40% 영역)
        height, width = image.shape[:2]
        roi_y_start = int(height * roi_y_start_ratio)
        roi_image = image[roi_y_start:, :]  # ROI 이미지 저장 (시각화용)
        
        # HSV 변환
        hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
        
        # 흰색 차선 감지
        lower_white = np.array(lower_white_hsv)
        upper_white = np.array(upper_white_hsv)
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # 노란색 차선 감지
        lower_yellow = np.array(lower_yellow_hsv)
        upper_yellow = np.array(upper_yellow_hsv)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # 흰색 차선 분석
        roi_height, roi_width = white_mask.shape
        left_mask = white_mask[:, :roi_width//3]
        mid_mask = white_mask[:, roi_width//3:2*roi_width//3]
        right_mask = white_mask[:, 2*roi_width//3:]
        
        left_ratio = cv2.countNonZero(left_mask) / left_mask.size
        mid_ratio = cv2.countNonZero(mid_mask) / mid_mask.size
        right_ratio = cv2.countNonZero(right_mask) / right_mask.size
        total_white = cv2.countNonZero(white_mask)
        
        # 흰색 차선 감지 여부
        white_detected = total_white > white_pixel_threshold
        
        # 노란색 차선 분석
        yellow_moments = cv2.moments(yellow_mask)
        yellow_area = yellow_moments['m00']
        yellow_center_x = None
        yellow_detected = False
        
        if yellow_area > yellow_area_threshold:
            yellow_center_x = int(yellow_moments['m10'] / yellow_moments['m00'])
            yellow_detected = True
        
        # 메트릭 생성
        white_metrics = WhiteLineHsvMetrics(
            timestamp=time.time(),
            total_white_pixels=total_white,
            left_ratio=left_ratio,
            mid_ratio=mid_ratio,
            right_ratio=right_ratio,
            is_detected=white_detected
        )
        
        yellow_metrics = YellowLineHsvMetrics(
            timestamp=time.time(),
            area=yellow_area,
            center_x=yellow_center_x,
            is_detected=yellow_detected
        )
        
        # ROI 하단 10줄에서 차선 x좌표 추출
        lane_boundaries_x = []
        roi_bottom = white_mask[-10:, :]  # 하단 10줄
        # 흰색 실선(좌/우) x좌표
        white_indices = np.where(np.sum(roi_bottom, axis=0) > 0)[0]
        if len(white_indices) > 0:
            left_white_x = int(white_indices[0])
            right_white_x = int(white_indices[-1])
            lane_boundaries_x.append(left_white_x)
        # 노란색 점선 x좌표(여러 개 가능)
        yellow_bottom = yellow_mask[-10:, :]
        yellow_indices = np.where(np.sum(yellow_bottom, axis=0) > 0)[0]
        # 노란선이 흰선과 겹칠 수 있으므로, 흰선 경계 바깥만 추출
        yellow_xs = []
        if len(yellow_indices) > 0:
            for x in yellow_indices:
                # 흰선 경계 바깥은 제외
                if (len(white_indices) == 0) or (x > left_white_x + 10 and x < right_white_x - 10):
                    yellow_xs.append(int(x))
        lane_boundaries_x.extend(yellow_xs)
        if len(white_indices) > 0:
            lane_boundaries_x.append(right_white_x)
        lane_boundaries_x = sorted(list(set(lane_boundaries_x)))
        
        # HSV 차선 감지 시각화 (새로운 Visualize.py 사용)
        if self.debug_cv_show:
            # 메트릭을 딕셔너리 형태로 변환 (PerceptionVisualizer 호환)
            white_metrics_dict = {
                'is_detected': white_metrics.is_detected,
                'total_white_pixels': white_metrics.total_white_pixels,
                'left_ratio': white_metrics.left_ratio,
                'mid_ratio': white_metrics.mid_ratio,
                'right_ratio': white_metrics.right_ratio
            }
            
            yellow_metrics_dict = {
                'is_detected': yellow_metrics.is_detected,
                'area': yellow_metrics.area,
                'center_x': yellow_metrics.center_x
            }
            
            # PerceptionVisualizer를 통한 HSV 차선 감지 시각화
            self.perception_visualizer.visualize_hsv_lane_detection(
                white_mask=white_mask,
                yellow_mask=yellow_mask,
                roi_image=roi_image,
                white_metrics=white_metrics_dict,
                yellow_metrics=yellow_metrics_dict
            )
        
        logger.debug(f"HSV Detection - White: detected={white_detected}, pixels={total_white}, ratios=L:{left_ratio:.3f} M:{mid_ratio:.3f} R:{right_ratio:.3f}")
        logger.debug(f"HSV Detection - Yellow: detected={yellow_detected}, area={yellow_area}, center_x={yellow_center_x}")
        
        return {
            "white_line_hsv_metrics": white_metrics,
            "yellow_line_hsv_metrics": yellow_metrics,
            "lane_boundaries_x": lane_boundaries_x
        }

    def _execute_canny_hough_lane_detection(self, image, params) -> dict:
        """
        Canny Hough 차선 감지 알고리즘 예시 플레이스홀더입니다.
        """
        # logger.debug(f"Running _execute_canny_hough_lane_detection with params: {params}")
        # 여기에 실제 Canny + Hough 차선 감지 로직을 구현합니다.
        return {} # 빈 결과 반환

    def _execute_custom_block_example(self, image, params) -> dict:
        """
        사용자 정의 알고리즘 블록 예시 플레이스홀더입니다.
        """
        # logger.debug(f"Running _execute_custom_block_example with params: {params}")
        # 여기에 실제 사용자 정의 로직을 구현합니다.
        return {} # 빈 결과 반환

    def run(self):
        logger.info("PerceptionModule: Thread started.")
        try:
            while self._running:
                try:
                    sensor_data = self.input_queue_sensor_data.get(block=False)
                except queue.Empty:
                    sensor_data = None
                if sensor_data:
                    try:
                        output = self._process_sensor_data(sensor_data)
                        for key, q in self.output_queues.items():
                            try:
                                q.put(output, timeout=0.1)
                            except queue.Full:
                                logger.warning(f"PerceptionModule: Output queue '{key}' is full.")
                    except Exception as e:
                        error_manager.handle(ErrorCode.MODULE_RUNTIME_EXCEPTION, str(e))
                time.sleep(0.01)
            logger.info("PerceptionModule: Thread stopped.")
        except Exception as e:
            error_manager.handle(ErrorCode.MODULE_RUNTIME_EXCEPTION, str(e))

    def start(self):
        if not self._running:
            self._running = True
            try:
                self._thread = threading.Thread(target=self.run, name="PerceptionThread")
                self._thread.start()
                logger.info("PerceptionModule: Started.")
            except Exception as e:
                error_manager.handle(ErrorCode.MODULE_START_FAIL, str(e))
                raise

    def stop(self):
        if self._running:
            self._running = False
            try:
                # 시각화 리소스 정리
                if hasattr(self, 'perception_visualizer'):
                    self.perception_visualizer.cleanup()
                
                if self._thread:
                    self._thread.join(timeout=2.0)
                logger.info("PerceptionModule: Stopped.")
            except Exception as e:
                error_manager.handle(ErrorCode.MODULE_RUNTIME_EXCEPTION, str(e))