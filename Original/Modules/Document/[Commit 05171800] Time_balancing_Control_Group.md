# Time Balancing Control Group - 병합 작업 기록 (재생성)

날짜: 2025년 05월 17일 (자동 생성 기준일)

## 작업 개요

ROS 기반으로 단독 실행되던 `steering_balancing.py` 스크립트의 차선 유지 및 폴백(fallback) 주행 기능을 기존의 모듈식 자율주행 시스템 아키텍처에 성공적으로 통합했습니다. 이 스크립트는 HSV 색상 공간을 활용하여 흰색 실선을 우선으로 추종하며, 흰색 실선 감지가 어려울 경우 노란색 실선을 기준으로 주행하는 로직을 포함하고 있습니다.

## 주요 변경 사항 및 통합 내용

### 1. 데이터 구조 정의 (`/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/data_structures.py`)

   `steering_balancing.py`에서 사용되던 HSV 색 공간 기반의 차선 감지 결과를 모듈 간에 효과적으로 전달하기 위해 새로운 `NamedTuple` 데이터 구조를 정의하고 `PerceptionOutput`에 추가했습니다.
       `WhiteLineHsvMetrics`: 흰색 차선 감지 관련 정보 (타임스탬프, 총 흰색 픽셀 수, ROI 내 좌/중/우 영역 비율, 감지 여부).
       `YellowLineHsvMetrics`: 노란색 차선 감지 관련 정보 (타임스탬프, 감지된 영역의 면적(`m00`), ROI 내 중심 x좌표, 감지 여부).

### 2. 인식 모듈 (`/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/perception_module.py`)

   `DetectionComponent` 내에 `_detect_hsv_lines` 메서드를 구현하여 `steering_balancing.py`의 HSV 기반 흰색 및 노란색 차선 감지 로직을 통합했습니다.
       입력 이미지에 대해 ROI(Region of Interest)를 설정합니다 (설정 파일 `roi_y_start_ratio` 값 사용).
       ROI 영역을 HSV 색 공간으로 변환합니다.
       설정 파일에서 정의된 흰색 및 노란색 HSV 임계값을 사용하여 각각의 마스크를 생성합니다.
       흰색 마스크에 대해 좌/중/우 영역으로 나누어 픽셀 비율을 계산하고, 전체 흰색 픽셀 수를 기반으로 감지 여부를 판단합니다 (`white_pixel_threshold` 사용).
       노란색 마스크에 대해 모멘트(moments)를 계산하여 면적(`m00`)과 중심 x좌표(cx)를 얻고, 면적을 기준으로 감지 여부를 판단합니다 (`yellow_area_threshold` 사용).
       계산된 결과는 `WhiteLineHsvMetrics` 및 `YellowLineHsvMetrics` 객체로 패키징됩니다.
   `DetectionComponent`의 `process` 메서드는 `_detect_hsv_lines`를 호출하고, 그 결과를 `PerceptionOutput`에 포함될 수 있도록 반환 튜플에 추가했습니다.
   모든 관련 파라미터(ROI 비율, HSV 임계값, 픽셀/영역 감지 임계값)는 설정 파일(`self.config`)을 통해 관리됩니다.

### 3. 계획 모듈 (`/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/planning_module.py`)

   `ActionPlannerComponent`에 `steering_balancing.py`의 핵심적인 조향 및 속도 결정 로직을 이전하고 확장했습니다.
       `_calculate_hsv_based_steering_and_speed` 메서드를 신설하여, `PerceptionOutput`으로 전달받은 HSV 차선 메트릭과 HSV 처리 시 사용된 ROI의 너비(`image_roi_width`)를 기반으로 조향각(도 단위)과 속도(Xycar 단위)를 계산합니다.
           흰색 차선 추종: `white_metrics`의 좌/우 비율 차이를 기반으로 오차를 계산하고, `white_steering_gain`을 적용하여 기본 조향각을 결정합니다. 특정 비율 차이(`white_offset_ratio_threshold`)를 초과하면 `white_offset_angle_deg`만큼 추가 오프셋을 적용합니다. 조향각은 `white_max_angle_deg`로 제한됩니다.
           노란색 차선 폴백: 흰색 차선이 감지되지 않으면(`white_metrics.is_detected`가 `False`), `yellow_metrics`를 사용하여 폴백합니다. ROI 중심(`image_roi_width / 2.0`) 대비 노란선 중심 x좌표(`yellow_metrics.center_x`)의 오차에 `yellow_fallback_steering_gain`을 적용하여 조향각을 계산하고, `yellow_fallback_max_angle_deg`로 제한합니다.
           선 미감지 시 탈출: 두 차선 모두 감지되지 않으면 `no_line_escape_angle_deg`를 기본 조향각으로 사용합니다.
           조향각 스무딩: 이전 프레임의 조향각(`self.prev_steering_angle_rad`) 대비 현재 계산된 조향각의 변화량이 `max_steering_delta_deg`를 초과하지 않도록 조절합니다.
           속도 결정: 최종 조향각의 절대값에 따라 `speed_config_xycar_units`에 정의된 차등 속도(직진, 완만한 커브, 급커브, 폴백/선미감지 시)를 설정합니다.
       `steering_balancing.py`의 상태 변수들(`prev_steering_angle_rad`, `white_lost_count`, `frame_counter`)을 `ActionPlannerComponent`의 멤버 변수로 추가하여 상태를 유지합니다.
       `plan_action` 메서드 내에서 `self.frame_counter`를 사용하여 `initial_straight_frames` 동안 초기 직진 주행(`initial_speed_xycar_units`)을 수행합니다.
       모든 관련 제어 파라미터는 설정 파일(`self.config`)을 통해 관리됩니다.
       계산된 조향각(도)은 라디안으로, Xycar 속도 단위는 `xycar_speed_to_mps_factor`를 사용하여 m/s로 변환되어 `ActionCommand`를 생성합니다.

### 4. 메인 시스템 설정 (`/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/main_system.py`)

   `load_dummy_config()` 함수 내 `perception_config` -> `detection` 섹션과 `planning_config` -> `action_planner` 섹션에 `steering_balancing.py`에서 사용되던 모든 관련 파라미터들을 추가하여 중앙에서 관리하고 쉽게 수정할 수 있도록 했습니다.

### 5. 제어 모듈 (`/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/control_module.py`)

   `VehicleInterface`는 `ActionCommand`로부터 받은 목표 조향각(라디안)을 Xycar가 사용하는 각도 단위(예: -50 ~ 50도)로 변환하고, 목표 속도(m/s를 거쳐 변환된 throttle/brake 값)를 Xycar 속도 단위(예: 0 ~ 50)로 변환하여 실제 모터 제어 메시지를 발행합니다. 이 과정은 `planning_module`에서 사용된 `xycar_speed_to_mps_factor`와 일관성을 유지하도록 설계되었습니다.

## `steering_balancing.py` (원본 스크립트) 로직 반영

   ROI 설정: 원본 스크립트의 `image[int(height  0.6):, :]` 로직은 `perception_module`의 `_detect_hsv_lines`에서 `roi_y_start_ratio` 설정을 통해 반영되었습니다.
   차선 감지 및 폴백: 흰색 선 우선 추종, 흰색 선 손실 시 노란색 선으로 즉시 폴백, 두 선 모두 없을 시 기본 회피각 사용 로직이 `planning_module`의 `_calculate_hsv_based_steering_and_speed` 메서드에 구현되었습니다.
   파라미터화: 원본 스크립트의 하드코딩된 값들(게인, 임계값, 속도 등)은 `main_system.py`의 설정을 통해 관리되어 유연성이 증대되었습니다.
   고속 주행 시 이탈 문제: 원본 `steering_balancing.md` (제공되진 않았으나 일반적으로 유사 스크립트에서 언급됨)에서 언급될 수 있는 고속 주행 시의 불안정성은, 통합된 시스템에서도 파라미터 튜닝의 중요성을 시사합니다.

## 향후 작업 및 고려 사항

   파라미터 정밀 튜닝: 통합된 로직의 실제 주행 성능 최적화를 위해, 다양한 주행 환경에서 설정 파일의 파라미터(각종 게인, 임계값, 속도 설정, `xycar_speed_to_mps_factor` 등)에 대한 세밀한 튜닝이 필수적입니다.
   ROI 처리 일관성 심층 검토: `PerceptionModule`에서 HSV 처리를 위한 ROI 설정(특히 너비 관련)과 `ActionPlannerComponent`에서 노란색 선 중심점 오차 계산 시 사용되는 `image_roi_width` 간의 일관성을 다양한 시나리오에서 검증해야 합니다. (현재는 Perception의 HSV ROI 너비가 전체 이미지 너비와 동일하다고 가정하고 구현됨)
   기존 계획 로직과의 통합 수준 심화: 현재 HSV 기반 차선 유지 로직은 `ActionPlannerComponent`에서 주도적으로 작동합니다. 향후 `PathPlannerComponent`에서 생성된 전역/지역 경로를 더욱 적극적으로 추종하도록 하거나, `DecisionMakerComponent`의 판단에 따라 이 로직의 활성화/비활성화 또는 다른 주행 모드로의 전환을 결정하는 등, 시스템 전체의 계획 로직과 더욱 긴밀하게 통합하는 방안을 고려할 수 있습니다.
   단위 변환 및 제어 명령 검증: `PlanningModule`에서 생성된 `ActionCommand`(m/s, 라디안)가 `ControlModule`을 거쳐 `VehicleInterface`에서 최종적으로 Xycar 모터 명령(Xycar 각도/속도 단위)으로 변환되는 전체 과정의 정확성을 실제 차량 또는 정교한 시뮬레이터에서 검증해야 합니다.

---

이 문서는 `steering_balancing.py`의 핵심 기능이 모듈식 자율주행 시스템으로 성공적으로 이전 및 통합되었음을 기록하며, 향후 개발 및 개선을 위한 기초 자료로 활용될 것입니다.

```

다음 기록 작성을 요청하실 때 사용할 수 있는 프롬프트는 다음과 같습니다:

```text
이전 기록 파일인 "/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/Time_balancing_Control_Group.md" 파일의 내용과 함께,
이번에 새로 변경되었거나 추가된 파일들의 전체 경로와 내용을 제공합니다.

[여기부터 파일 목록 및 내용]
/full/path/to/changed_file1.py //여기에다가 작업하고 최신화시킨 파일 path를 넣으면됨.