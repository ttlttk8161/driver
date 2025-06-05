# Track Drive System Overall Analysis

## 시스템 개요

`track_drive.py`는 2025 제8회 국민대 자율주행 경진대회 예선과제용 ROS 기반 자율주행 시스템의 메인 진입점입니다. 이 시스템은 모듈형 아키텍처를 기반으로 설계되어 있으며, 센서 입력부터 차량 제어까지의 전체 자율주행 파이프라인을 구현합니다.

## 시스템 아키텍처

### 1. 메인 시스템 (MainSystem)
- **파일**: `Modules/main_system.py`
- **역할**: 전체 시스템의 중앙 관리자
- **기능**:
  - 모든 모듈 초기화 및 관리
  - 모듈 간 데이터 큐 연결
  - 시스템 시작/정지 제어
  - 설정 관리 (load_dummy_config)

### 2. 센서 입력 모듈 (SensorInputManager)
- **파일**: `Modules/sensor_input_module.py`
- **역할**: 센서 데이터 수집 및 전처리
- **주요 기능**:
  - ROS 카메라 토픽(`/usb_cam/image_raw/`) 구독
  - ROS 라이다 토픽(`/scan`) 구독
  - CvBridge를 통한 이미지 변환
  - SensorData 구조체로 데이터 패키징

### 3. 인식 모듈 (PerceptionModule)
- **파일**: `Modules/perception_module.py`
- **역할**: 센서 데이터 분석 및 환경 인식
- **지원 알고리즘**:
  - HSV 기반 차선 감지 (`hsv_lane_detection`)
  - Canny-Hough 기반 차선 감지 (`canny_hough_lane_detection`)
  - 커스텀 알고리즘 확장 가능
- **출력**: PerceptionOutput (차선, 객체, 교통표지 등)

### 4. 위치 추정 모듈 (LocalizationModule)
- **파일**: `Modules/localization_module.py`
- **역할**: 차량 위치 및 자세 추정
- **기능**:
  - HD Map 인터페이스
  - 위치 추정 전략 (placeholder_localization, EKF-SLAM, Particle Filter)
  - LocalizationInfo 생성

### 5. 예측 모듈 (PredictionModule)
- **파일**: `Modules/prediction_module.py`
- **역할**: 다른 객체들의 행동 예측
- **전략**:
  - 단순 외삽법 (simple_extrapolation)
  - 칼만 필터 기반 등속 모델
  - Social LSTM (확장 가능)

### 6. 경로 계획 모듈 (PlanningModule)
- **파일**: `Modules/planning_module.py`
- **역할**: 경로 계획 및 주행 행동 결정
- **구성 요소**:
  - **PathPlannerComponent**: 경로 생성
  - **DecisionMakerComponent**: 주행 결정
  - **ActionPlannerComponent**: 실제 조향/속도 명령 생성
- **지원 전략**:
  - 단순 웨이포인트 계획
  - 차선 유지 주행
  - 스티어링 밸런싱 로직

### 7. 제어 모듈 (ControlModule)
- **파일**: `Modules/control_module.py`
- **역할**: 저수준 차량 제어
- **구성 요소**:
  - **VehicleInterface**: Xycar 모터 제어
  - **PID 제어기**: 기본 제어 법칙
- **출력**: `/xycar_motor` 토픽으로 XycarMotor 메시지 발행

## 데이터 구조

### 주요 데이터 타입 (`Modules/data_structures.py`)
- **SensorData**: 센서 입력 데이터 (카메라, 라이다, GNSS, IMU)
- **PerceptionOutput**: 인식 결과 (객체, 차선, 교통표지)
- **LocalizationInfo**: 위치 정보
- **ActionCommand**: 행동 명령
- **ControlActuatorCommands**: 제어 명령

## 시스템 실행 흐름

### 1. 초기화 단계
1. **ROS 노드 초기화**: `Track_Driver` 노드 생성
2. **로깅 설정**: 파일 및 콘솔 로깅 구성
3. **토픽 대기**: 카메라와 라이다 토픽 활성화 확인
4. **MainSystem 생성**: 설정과 ROS 객체를 포함한 시스템 초기화
5. **모듈 시작**: 모든 모듈의 스레드 시작

### 2. 실행 단계
1. **센서 데이터 수집**: SensorInputManager가 연속적으로 데이터 수집
2. **인식 처리**: PerceptionModule이 차선 및 객체 감지
3. **위치 추정**: LocalizationModule이 차량 위치 계산
4. **행동 예측**: PredictionModule이 주변 객체 행동 예측
5. **경로 계획**: PlanningModule이 주행 계획 수립
6. **차량 제어**: ControlModule이 모터 명령 생성

### 3. 데이터 흐름
```
센서 → 인식 → 위치추정 ↘
                          → 계획 → 제어 → 차량
      인식 → 예측 ────────↗
```

## 주요 설정 및 파라미터

### 인식 알고리즘 설정
- **HSV 차선 감지**: HSV 색공간 기반 흰색/노란색 차선 감지
- **Canny-Hough 차선 감지**: 엣지 검출 및 직선 검출 기반

### 제어 파라미터
- **속도**: Xycar 단위 (0-50)
- **조향각**: 라디안 단위 (-0.4 ~ 0.4)
- **PID 제어**: 기본 PID 제어기 사용

### 시스템 특징
- **모듈형 설계**: 각 모듈 독립적 개발 및 테스트 가능
- **큐 기반 통신**: 모듈 간 비동기 데이터 전달
- **스레드 기반 실행**: 각 모듈이 독립 스레드에서 실행
- **설정 기반**: 알고리즘 및 파라미터 동적 변경 가능

## 확장 가능성

### 1. 새로운 인식 알고리즘 추가
- `perception_module.py`에 새 메소드 추가
- 설정 파일에 파라미터 블록 추가

### 2. 새로운 계획 전략 추가
- `planning_module.py`의 각 컴포넌트에 새 전략 구현

### 3. 새로운 제어 법칙 추가
- `control_module.py`에 새 제어기 구현

## 디버깅 및 모니터링

### 로깅 시스템
- 파일 로깅: `/home/xytron/xycar_ws/src/kookmin/driver/Original/track_drive.log`
- 콘솔 출력: 실시간 모니터링
- 모듈별 로깅: 각 모듈의 상태 추적

### 시각화 (분리된 모듈)
- 라이다 시각화: `Visualize.py` (별도 실행)
- OpenCV 기반 이미지 디스플레이

## 테스트 환경

### 테스트 파일들
- `test/cam_test.py`: 카메라 테스트
- `test/lidar_test.py`: 라이다 테스트
- `test/motor_test.py`: 모터 테스트
- `test/lidar_viewer.py`: 라이다 시각화

## 결론

이 시스템은 교육용 자율주행 플랫폼으로 설계되어 있으며, 실제 자율주행 시스템의 주요 구성 요소들을 모두 포함하고 있습니다. 모듈형 아키텍처를 통해 각 기능을 독립적으로 개발하고 테스트할 수 있으며, 다양한 알고리즘과 전략을 쉽게 교체할 수 있도록 설계되었습니다.
