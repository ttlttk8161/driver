# `_detect_hsv_lines` 함수 호출 흐름 및 반환 값의 여정

**날짜:** 2025년 05월 19일

## `_detect_hsv_lines` 함수 호출 흐름도

1.  **시작점: `/home/xytron/xycar_ws/src/kookmin/driver/Original/track_drive.py`**
    *   `start()` 함수가 실행되면 `MainSystem` 객체(`autonomous_system`)를 생성하고 `autonomous_system.start()`를 호출합니다.
    *   이 과정에서 `MainSystem`은 내부적으로 모든 모듈(SensorInputManager, PerceptionModule 등)을 초기화하고 각 모듈의 `start()` 메서드를 호출하여 개별 스레드에서 실행되도록 합니다.

2.  **센서 데이터 입력: `/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/sensor_input_module.py` (`SensorInputManager`)**
    *   `SensorInputManager`는 ROS 토픽 (`/usb_cam/image_raw`)으로부터 카메라 이미지를 지속적으로 수신합니다 (`_ros_image_callback` 메서드).
    *   수신된 이미지는 `_publish_sensor_data_loop` 메서드에 의해 `SensorData` 객체로 패키징되어 `sensor_to_perception_queue`라는 큐(Queue)에 넣어집니다.

3.  **인지 모듈 실행: `/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/perception_module.py` (`PerceptionModule`)**
    *   `PerceptionModule`의 `run()` 메서드는 별도의 스레드에서 실행되며, `sensor_to_perception_queue`에서 `SensorData` 객체를 가져옵니다.
    *   `SensorData`를 성공적으로 가져오면, `_process_frame(sensor_data)` 메서드를 호출하여 해당 센서 데이터를 처리합니다.

4.  **프레임 처리 내부: `/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/perception_module.py` (`PerceptionModule._process_frame`)**
    *   `_process_frame` 메서드 내부에서는 여러 인지 관련 컴포넌트들이 순차적으로 호출됩니다.
    *   그중 하나로 `self.detection_comp.process(sensor_data, depth_map)`가 호출됩니다. 여기서 `self.detection_comp`는 `DetectionComponent`의 인스턴스입니다.

5.  **탐지 컴포넌트 실행: `/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/perception_module.py` (`DetectionComponent.process`)**
    *   `DetectionComponent`의 `process` 메서드는 입력받은 `sensor_data` (여기에는 카메라 이미지가 포함됨)를 사용하여 다양한 탐지 작업을 수행합니다.
    *   이 메서드 내부에서 **`white_line_hsv_metrics, yellow_line_hsv_metrics, self.debug_white_mask, self.debug_yellow_mask = self._detect_hsv_lines(image, sensor_data.timestamp)`** 와 같이 `_detect_hsv_lines` 함수가 호출됩니다.
    *   이때 `image`는 `sensor_data.vision_data`에서 가져온 실제 카메라 이미지이고, `sensor_data.timestamp`는 해당 이미지의 타임스탬프입니다.

**호출 순서 요약:**

`track_drive.py` (MainSystem 시작)
`->` `MainSystem` (PerceptionModule 스레드 시작)
`->` `SensorInputManager` (카메라 이미지 수신 및 큐에 넣기)
`->` `PerceptionModule.run()` (큐에서 SensorData 가져오기)
`->` `PerceptionModule._process_frame()` (가져온 SensorData 처리)
`->` `DetectionComponent.process()` (SensorData 내 이미지로 탐지 작업 수행)
`->` **`DetectionComponent._detect_hsv_lines()`** (이미지를 사용하여 HSV 기반 차선 감지 수행)

---

## `_detect_hsv_lines` 반환 값의 여정 및 차량 조향에 미치는 영향

### 1. `_detect_hsv_lines`의 반환 값

`/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/perception_module.py`의 `_detect_hsv_lines` 함수는 다음을 반환합니다:
*   `white_metrics`: `WhiteLineHsvMetrics` 객체 (흰색 선 정보)
*   `yellow_metrics`: `YellowLineHsvMetrics` 객체 (노란색 선 정보)
*   `white_mask`, `yellow_mask`: 디버깅용 마스크 이미지

### 2. 반환 값의 여정

*   **`DetectionComponent.process`**:
    *   `white_line_hsv_metrics`와 `yellow_line_hsv_metrics`를 다른 탐지 결과와 함께 `PerceptionModule._process_frame`으로 반환합니다.

*   **`PerceptionModule._process_frame`**:
    *   이 메트릭들은 `PerceptionOutput` 객체에 포함됩니다.

*   **`PerceptionModule.run`**:
    *   `PerceptionOutput` 객체는 `perception_to_planning_queue`를 통해 `PlanningModule`로 전달됩니다.

*   **`PlanningModule.run`**:
    *   `PlanningModule`은 큐에서 `PerceptionOutput`을 가져와 `_latest_perception`에 저장합니다.

*   **`PlanningModule`의 컴포넌트 호출**:
    *   `ActionPlannerComponent.plan_action` 메서드가 호출될 때, `_latest_perception`이 `perception_info` 인자로 전달됩니다.

*   **`ActionPlannerComponent.plan_action`**:
    *   `perception_info` (흰색/노란색 선 메트릭 포함)를 사용하여 `_calculate_hsv_based_steering_and_speed` 메서드를 호출합니다.

*   **`ActionPlannerComponent._calculate_hsv_based_steering_and_speed`**:
    *   **핵심 지점:** `_detect_hsv_lines`의 결과인 `white_metrics`와 `yellow_metrics`가 직접적으로 조향각과 속도 계산에 사용됩니다.
        *   흰색 선의 비율 차이 또는 노란색 선의 중심점 오차를 기반으로 `error`를 계산합니다.
        *   `error`에 각 상황에 맞는 게인 값(`white_steering_gain`, `yellow_fallback_steering_gain`)을 곱하여 `angle_deg` (조향각)를 계산합니다.
        *   추가적인 오프셋, 스무딩, 속도 결정 로직이 적용됩니다.
    *   최종적으로 `angle_deg` (도 단위)와 `speed_xycar` (Xycar 속도 단위)를 반환합니다.

*   **다시 `ActionPlannerComponent.plan_action`**:
    *   반환된 `target_steering_deg`와 `target_speed_xycar_units`를 사용하여 `ActionCommand` 객체를 생성합니다 (조향각은 라디안, 속도는 m/s로 변환).

*   **`PlanningModule.run` (다시)**:
    *   생성된 `action_command`는 `planning_to_control_queue`를 통해 `ControlModule`로 전달됩니다.

*   **`ControlModule.run`**:
    *   `ControlModule`은 큐에서 `ActionCommand`를 가져옵니다.
    *   `_translate_action_to_commands`를 통해 `ControlActuatorCommands`로 변환될 수 있습니다 (현재는 `ActionCommand`의 값을 거의 그대로 사용).
    *   `vehicle_interface.send_commands` 메서드로 전달됩니다.

*   **`VehicleInterface.send_commands` (`control_module.py`)**:
    *   `ActionCommand`의 `target_steering_angle_rad` (라디안)가 Xycar의 각도 단위(-50 ~ 50도)로 변환됩니다.
    *   `target_velocity_mps`가 Xycar의 속도 단위(0 ~ 50)로 변환됩니다.
    *   변환된 값들은 `XycarMotor` 메시지에 담겨 `/xycar_motor` 토픽으로 발행됩니다.

### 3. 최후: 차량 조향에 작동

*   Xycar 차량의 실제 모터 제어 노드는 `/xycar_motor` 토픽을 구독합니다.
*   이 노드가 `VehicleInterface`에서 발행한 `XycarMotor` 메시지를 수신하면, 메시지 내의 조향각과 속도 값을 읽어 실제 차량의 조향 모터와 구동 모터를 제어합니다.

**결론:**

`_detect_hsv_lines`에서 계산된 차선 정보는 `PerceptionModule` -> `PlanningModule` (`ActionPlannerComponent`에서 조향/속도 계산) -> `ControlModule` (`VehicleInterface`에서 모터 명령 변환)을 거쳐 최종적으로 차량의 물리적인 움직임으로 이어집니다.