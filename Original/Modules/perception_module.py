import cv2
import numpy as np
import logging
from .thread_queue_manager import ThreadQueueManager
from queue import Empty
import threading
import time
from .data_structures import SensorData, PerceptionOutput
from .error_manager import error_manager, ErrorCode
from .Visualize import PerceptionVisualizer  # HSV 차선 감지 시각화를 위한 import

# 로깅 설정 (애플리케이션의 다른 부분에서 이미 설정되었을 수 있습니다)
# 예: logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PerceptionModule:
    def __init__(self, config: dict, 
                 input_queue_sensor_data: ThreadQueueManager, 
                 output_queues: dict):
        
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
        
        logger.info(f"PerceptionModule: Processing frame {timestamp}")
        logger.debug(f"PerceptionModule: Image shape: {image.shape if image is not None else 'None'}")
        logger.debug(f"PerceptionModule: Active algorithm: {self.active_perception_algorithm}")
        
        # 알고리즘 이름과 해당 실행 메소드를 매핑합니다.
        # 새로운 알고리즘을 추가할 때 이 사전에 추가하면 됩니다.
        algorithm_map = {
            "hsv_lane_detection": self._execute_hsv_lane_detection
            # "canny_hough_lane_detection": self._execute_canny_hough_lane_detection
        }

        if self.active_perception_algorithm and self.active_perception_algorithm in algorithm_map:
            selected_method = algorithm_map[self.active_perception_algorithm]
            algorithm_params = self.params.get(self.active_perception_algorithm, {})

            algo_output_dict = selected_method(image, algorithm_params) 
            
            logger.debug(f"PerceptionModule: Algorithm output keys: {list(algo_output_dict.keys())}")
            logger.debug(f"PerceptionModule: white_mask shape: {algo_output_dict.get('white_mask').shape if algo_output_dict.get('white_mask') is not None else 'None'}")
            logger.debug(f"PerceptionModule: yellow_mask shape: {algo_output_dict.get('yellow_mask').shape if algo_output_dict.get('yellow_mask') is not None else 'None'}")
            
            perception_output = PerceptionOutput(
                timestamp=timestamp,
                detected_objects=algo_output_dict.get("detected_objects", []),
                lane_markings=algo_output_dict.get("lane_markings", []),
                drivable_area_mask=algo_output_dict.get("drivable_area_mask"),
                traffic_signs=algo_output_dict.get("traffic_signs", []),
                semantic_segmentation_map=None, instance_segmentation_map=None, depth_map=None,
                optical_flow_map=None, scene_flow_map=None, raw_features_for_localization=None,
                white_mask=algo_output_dict.get("white_mask"),
                yellow_mask=algo_output_dict.get("yellow_mask")
            )
            
            logger.info(f"PerceptionModule: Created PerceptionOutput with timestamp {timestamp}")
            logger.debug(f"PerceptionModule: PerceptionOutput has white_mask: {hasattr(perception_output, 'white_mask') and perception_output.white_mask is not None}")
            
            return perception_output
        elif self.active_perception_algorithm is None:
            error_manager.handle(ErrorCode.PERCEPTION_ALGO_NOT_SELECTED, "PerceptionModule: perception 작업을 위한 알고리즘이 선택되지 않았습니다.")
        else:
            error_manager.handle(ErrorCode.PERCEPTION_ALGO_NOT_SELECTED, f"PerceptionModule: 알 수 없거나 지원되지 않는 인식 알고리즘이 선택됨: {self.active_perception_algorithm}")
        # 알고리즘이 선택되지 않았거나, 알 수 없는 경우 기본 빈 PerceptionOutput 반환
        logger.warning(f"PerceptionModule: Returning empty PerceptionOutput due to algorithm issue")
        return PerceptionOutput(
            timestamp=timestamp, detected_objects=[], lane_markings=[], drivable_area_mask=None,
            traffic_signs=[], semantic_segmentation_map=None, instance_segmentation_map=None,
            depth_map=None, optical_flow_map=None, scene_flow_map=None,
            raw_features_for_localization=None, white_mask=None, yellow_mask=None
        )

    def _execute_hsv_lane_detection(self, image, params) -> dict:
        # 파라미터 읽기 - 더 엄격한 HSV 임계값으로 조정
        roi_y_start_ratio = params.get('roi_y_start_ratio', 0.6)
        # 흰색 차선: 더 높은 명도 최소값과 낮은 채도 최대값으로 조정
        lower_white_hsv = params.get('lower_white_hsv', [0, 0, 200])
        upper_white_hsv = params.get('upper_white_hsv', [180, 25, 255])
        # 노란색 차선: 색상 범위를 좁히고 채도/명도 최소값 상향 조정
        lower_yellow_hsv = params.get('lower_yellow_hsv', [15, 120, 120])
        upper_yellow_hsv = params.get('upper_yellow_hsv', [35, 255, 255])

        # ROI 설정
        height, width = image.shape[:2]
        roi_y_start = int(height * roi_y_start_ratio)
        roi_image = image[roi_y_start:, :]
        roi_height, roi_width = roi_image.shape[:2]

        # HSV 변환
        hsv = cv2.cvtColor(roi_image, cv2.COLOR_BGR2HSV)
        lower_white = np.array(lower_white_hsv)
        upper_white = np.array(upper_white_hsv)
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        lower_yellow = np.array(lower_yellow_hsv)
        upper_yellow = np.array(upper_yellow_hsv)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        white_pixels = np.sum(white_mask == 255)
        yellow_pixels = np.sum(yellow_mask == 255)
        
        logger.info(f"1. HSV Detection: white pixel density={white_pixels/(white_mask.shape[0]*white_mask.shape[1]):.3f}")
        logger.info(f"1. HSV Detection: yellow pixel density={yellow_pixels/(yellow_mask.shape[0]*yellow_mask.shape[1]):.3f}")

        # --- plan_path에서 사용할 데이터 dict로 반환 ---
        return {
            "white_mask": white_mask,
            "yellow_mask": yellow_mask
        }
    

    def _execute_canny_hough_lane_detection(self, image, params) -> dict:
        """
        Canny+Hough 차선 검출 알고리즘이 아직 구현되지 않은 경우를 위한 placeholder.
        실제 구현이 필요하다면 이 부분을 수정하세요.
        """
        logger.warning("_execute_canny_hough_lane_detection: 아직 구현되지 않은 알고리즘입니다.")
        return {
            "detected_objects": [],
            "lane_markings": [],
            "drivable_area_mask": None,
            "traffic_signs": [],
            "lane_boundaries_x": None
        }

    def run(self):
        logger.info("PerceptionModule: Thread started.")
        try:
            while self._running:
                try:
                    sensor_data = self.input_queue_sensor_data.get(block=False)
                except Empty:
                    sensor_data = None
                if sensor_data:
                    try:
                        logger.debug(f"PerceptionModule: Processing sensor data with timestamp {sensor_data.timestamp}")
                        output = self._process_sensor_data(sensor_data)
                        logger.debug(f"PerceptionModule: Sending output to {len(self.output_queues)} queues")
                        for key, q in self.output_queues.items():
                            try:
                                q.put(output, timeout=0.1)
                                logger.debug(f"PerceptionModule: Successfully sent to queue '{key}'")
                            except Exception as e:
                                logger.warning(f"PerceptionModule: Output queue '{key}' put error: {e}")
                    except Exception as e:
                        error_manager.handle(ErrorCode.MODULE_RUNTIME_EXCEPTION, str(e))
                time.sleep(0.001)
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