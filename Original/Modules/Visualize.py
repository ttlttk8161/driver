import cv2
import numpy as np
import logging
from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)

class HSVLaneVisualizer:
    """HSV 차선 감지 결과를 시각화하는 클래스"""
    
    def __init__(self, debug_enabled: bool = False):
        """
        HSVLaneVisualizer 초기화
        Args:
            debug_enabled (bool): 디버그 시각화 활성화 여부
        """
        self.debug_enabled = debug_enabled
        self.window_names = set()  # 생성된 윈도우 이름 추적
    
    def display_hsv_masks(self, white_mask: np.ndarray, yellow_mask: np.ndarray, 
                         detection_results: Optional[Dict[str, Any]] = None,
                         waypoints: Optional[list] = None) -> None:
        """
        HSV 마스크 이미지들을 화면에 표시
        Args:
            white_mask (np.ndarray): 흰색 차선 마스크
            yellow_mask (np.ndarray): 노란색 차선 마스크
            detection_results (dict, optional): 감지 결과 정보 (로깅용)
        """
        if not self.debug_enabled or not hasattr(cv2, 'imshow'):
            return
        
        try:
            # 흰색 마스크 표시
            cv2.imshow("HSV_White_Mask", white_mask)
            self.window_names.add("HSV_White_Mask")
            
            # 노란색 마스크 표시
            cv2.imshow("HSV_Yellow_Mask", yellow_mask)
            self.window_names.add("HSV_Yellow_Mask")
            
            # 결합된 마스크 표시 (선택사항)
            combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
            combined_mask_color = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)

            # waypoints 시각화 (좌표계 일치 여부 확인)
            if waypoints is not None and len(waypoints) > 0:
                for (x, y) in waypoints:
                    # ROI 좌표계 기준으로 (x, y)가 이미지 내에 있는지 체크
                    if 0 <= int(x) < combined_mask_color.shape[1] and 0 <= int(y) < combined_mask_color.shape[0]:
                        cv2.circle(combined_mask_color, (int(x), int(y)), 5, (0, 0, 255), -1)
                    else:
                        logger.debug(f"Waypoint ({x}, {y}) is out of image bounds and will not be drawn.")

            cv2.imshow("HSV_Combined_Mask", combined_mask_color)
            self.window_names.add("HSV_Combined_Mask")
            
            # 키 입력 대기 (1ms)
            cv2.waitKey(1)
            
            # 감지 결과 로깅
            if detection_results:
                self._log_detection_results(detection_results)
                
        except Exception as e:
            logger.warning(f"CV2 display error in HSVLaneVisualizer: {e}")
    
    def display_yellow_detection(self, yellow_mask: np.ndarray, yellow_metrics: Dict[str, Any]) -> None:
        """
        노란색 차선 감지 결과를 시각화
        Args:
            yellow_mask (np.ndarray): 노란색 마스크
            yellow_metrics (dict): 노란색 차선 메트릭
        """
        if not self.debug_enabled or not hasattr(cv2, 'imshow'):
            return
        
        try:
            # 노란색 마스크에 중심점 표시
            yellow_visual = cv2.cvtColor(yellow_mask, cv2.COLOR_GRAY2BGR)
            
            center_x = yellow_metrics.get('center_x')
            area = yellow_metrics.get('area', 0)
            is_detected = yellow_metrics.get('is_detected', False)
            
            # if center_x is not None and is_detected:
            #     # 중심점에 원 그리기
            #     mask_height = yellow_mask.shape[0]
            #     cv2.circle(yellow_visual, (int(center_x), mask_height // 2), 5, (0, 255, 0), -1)
            #     cv2.putText(yellow_visual, f"Center: {center_x}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.putText(yellow_visual, f"Area: {area:.0f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(yellow_visual, f"Detected: {is_detected}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if is_detected else (0, 0, 255), 2)
            
            cv2.imshow("HSV_Yellow_Detection", yellow_visual)
            self.window_names.add("HSV_Yellow_Detection")
            cv2.waitKey(1)
            
        except Exception as e:
            logger.warning(f"CV2 yellow detection display error: {e}")
    
    def _log_detection_results(self, results: Dict[str, Any]) -> None:
        """감지 결과를 로깅"""
        white_detected = results.get('white_detected', False)
        yellow_detected = results.get('yellow_detected', False)
        total_white = results.get('total_white', 0)
        yellow_area = results.get('yellow_area', 0)
        yellow_center_x = results.get('yellow_center_x')
        left_ratio = results.get('left_ratio', 0)
        right_ratio = results.get('right_ratio', 0)
        
        logger.debug(f"HSV Detection - White: detected={white_detected}, pixels={total_white}, ratios=L:{left_ratio:.3f}, R:{right_ratio:.3f}")
        logger.debug(f"HSV Detection - Yellow: detected={yellow_detected}, area={yellow_area}, center_x={yellow_center_x}")
    
    def close_all_windows(self) -> None:
        """생성된 모든 CV2 윈도우 닫기"""
        try:
            for window_name in self.window_names:
                cv2.destroyWindow(window_name)
            self.window_names.clear()
            cv2.waitKey(1)  # 윈도우 닫기 완료 대기
        except Exception as e:
            logger.warning(f"Error closing CV2 windows: {e}")
    
    def enable_debug(self) -> None:
        """디버그 시각화 활성화"""
        self.debug_enabled = True
    
    def disable_debug(self) -> None:
        """디버그 시각화 비활성화"""
        self.debug_enabled = False
        self.close_all_windows()


class PerceptionVisualizer:
    """전체 Perception 모듈 시각화를 관리하는 클래스"""
    
    def __init__(self, debug_enabled: bool = False):
        """
        PerceptionVisualizer 초기화
        Args:
            debug_enabled (bool): 디버그 시각화 활성화 여부
        """
        self.debug_enabled = debug_enabled
        self.hsv_visualizer = HSVLaneVisualizer(debug_enabled)
    
    def visualize_hsv_lane_detection(self, white_mask: np.ndarray, yellow_mask: np.ndarray,
                                   roi_image: Optional[np.ndarray] = None,
                                   white_metrics: Optional[Dict[str, Any]] = None,
                                   yellow_metrics: Optional[Dict[str, Any]] = None,
                                   waypoints: Optional[list] = None) -> None:  # waypoints 인자 추가
        """
        HSV 차선 감지 전체 결과를 시각화
        Args:
            white_mask (np.ndarray): 흰색 차선 마스크
            yellow_mask (np.ndarray): 노란색 차선 마스크
            roi_image (np.ndarray, optional): ROI 원본 이미지
            white_metrics (dict, optional): 흰색 차선 메트릭
            yellow_metrics (dict, optional): 노란색 차선 메트릭
            waypoints (list, optional): 웨이포인트 리스트
        """
        if not self.debug_enabled:
            return
        
        # 기본 마스크 표시
        detection_results = {}
        if white_metrics:
            detection_results.update({
                'white_detected': white_metrics.get('is_detected', False),
                'total_white': white_metrics.get('total_white_pixels', 0),
                'left_ratio': white_metrics.get('left_ratio', 0),
                'right_ratio': white_metrics.get('right_ratio', 0)
            })
        
        if yellow_metrics:
            detection_results.update({
                'yellow_detected': yellow_metrics.get('is_detected', False),
                'yellow_area': yellow_metrics.get('area', 0),
                'yellow_center_x': yellow_metrics.get('center_x')
            })
        
        # waypoints 인자 전달
        self.hsv_visualizer.display_hsv_masks(white_mask, yellow_mask, detection_results, waypoints)
        
        # # ROI 분석 시각화
        # if roi_image is not None and white_metrics:
        #     analysis_results = {
        #         'left_ratio': white_metrics.get('left_ratio', 0),
        #         'right_ratio': white_metrics.get('right_ratio', 0)
        #     }
        #     self.hsv_visualizer.display_roi_analysis(roi_image, white_mask, analysis_results)
        
        # 노란색 차선 상세 시각화
        if yellow_metrics:
            self.hsv_visualizer.display_yellow_detection(yellow_mask, yellow_metrics)
    
    def cleanup(self) -> None:
        """시각화 리소스 정리"""
        self.hsv_visualizer.close_all_windows()
    
    def enable_debug(self) -> None:
        """디버그 시각화 활성화"""
        self.debug_enabled = True
        self.hsv_visualizer.enable_debug()
    
    def disable_debug(self) -> None:
        """디버그 시각화 비활성화"""
        self.debug_enabled = False
        self.hsv_visualizer.disable_debug()
