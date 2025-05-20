# 자율 주행 시스템 설계 ([Doc.md](http://doc.md/))

이 문서는 제공된 다이어그램(그림 2 및 그림 3)을 기반으로 하는 자율 주행 시스템의 모듈식 설계를 간략하게 설명합니다. 이 시스템은 동시 작동을 위한 스레딩과 명확성 및 유지 관리성을 위한 모듈성으로 설계되었습니다.

## 목차

1. [코어 모듈 개요](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
2. [데이터 구조 (`data_structures.py`)](https://www.notion.so/1f4d05ac5cba8051b977feba0300ebfc?pvs=21)
3. 센서 입력 모듈 (`sensor_input_module.py`)
4. 인지 모듈 (`perception_module.py`)
    - 탐지 컴포넌트
    - 장면 이해 컴포넌트
    - 추적 컴포넌트
    - 인지 예측 컴포넌트
5. 측위 모듈 (`localization_module.py`)
6. 예측 모듈 (행동) (`prediction_module.py`)
7. 계획 모듈 (`planning_module.py`)
    - 경로 계획 컴포넌트
    - 의사 결정 컴포넌트
    - 행동 계획 컴포넌트
8. 제어 모듈 (`control_module.py`)
9. 주요 시스템 통합 (`main_system.py`)
10. 모듈 간 통신
11. 스레딩 모델

---

## 1. 코어 모듈 개요

시스템은 그림 2에 설명된 상위 수준 아키텍처를 따르며, 인지 모듈의 내부 구조는 그림 3에 자세히 설명되어 있습니다.

- **센서 입력**: 다양한 센서(LiDAR, 비전, GNSS, IMU)로부터 데이터를 수집합니다.
- **인지**: 센서 데이터를 처리하여 환경을 이해합니다. 여기에는 객체 상태의 탐지, 분할, 추적 및 단기 예측이 포함됩니다.
- **측위**: HD 맵과 센서 데이터를 사용하여 세계에서 차량의 정확한 위치와 방향을 추정합니다.
- **예측 (행동)**: 장면의 다른 동적 에이전트의 미래 행동과 궤적을 예측합니다.
- **계획**: 경로 계획, 기동 의사 결정, 모션/행동 계획을 포함하여 차량의 미래 행동 방침을 결정합니다.
- **제어**: 계획된 행동을 차량 액추에이터에 대한 명령으로 변환합니다.

---

## 2. 데이터 구조 (`data_structures.py`)

이 파일은 모듈 간 데이터 교환을 위한 공통 `NamedTuple` 클래스를 정의하여 명확하고 일관된 인터페이스를 보장합니다.

- `SensorData`: 주어진 타임스탬프에 대한 모든 센서의 원시/전처리된 데이터를 보유합니다.
- `DetectedObject`: 탐지된 객체에 대한 정보(ID, 유형, 위치, 속도 등).
- `LaneMarking`: 탐지된 차선을 설명합니다.
- `TrafficSignInfo`: 탐지된 교통 표지판에 대한 정보입니다.
- `PerceptionOutput`: 인지 모듈의 집계된 출력입니다.
- `LocalizationInfo`: 차량의 추정된 자세(위치, 방향) 및 속도입니다.
- `PredictedTrajectory`: 동적 에이전트의 잠재적인 미래 경로입니다.
- `BehavioralPredictionOutput`: 장면에 있는 에이전트에 대한 예측된 궤적 모음입니다.
- `PlannedPath`: 차량의 의도된 기하학적 경로를 정의하는 웨이포인트 시퀀스입니다.
- `ManeuverDecision`: 플래너가 선택한 상위 수준 주행 기동입니다.
- `ActionCommand`: 제어 모듈에 대한 하위 수준 모션 명령(목표 속도, 조향각)입니다.
- `ControlActuatorCommands`: 차량 액추에이터(조향, 스로틀, 브레이크)에 대한 특정 명령입니다.

---

## 3. 센서 입력 모듈 (`sensor_input_module.py`)

LiDAR, 카메라, GNSS 및 IMU와 같은 센서로부터 데이터 수집을 처리합니다.

### 클래스: `SensorInputManager`

- **목적**: 센서 초기화 및 지속적인 데이터 가져오기를 관리합니다.
- **인터페이스**:
    - `__init__(self, config: dict, output_queue: queue.Queue)`
    - `start_sensors(self)`: 별도의 스레드에서 데이터 수집을 시작합니다.
    - `stop_sensors(self)`: 데이터 수집을 중지합니다.
- **출력**: `SensorData` 객체를 `output_queue`로 푸시합니다.

---

## 4. 인지 모듈 (`perception_module.py`)

`SensorData`를 처리하여 차량 주변 환경에 대한 이해를 구축합니다. 내부 구조는 그림 3을 따릅니다.

### 클래스: `PerceptionModule`

- **목적**: 다양한 인지 하위 작업을 조정합니다.
- **인터페이스**:
    - `__init__(self, config: dict, input_queue: queue.Queue, output_queues: Dict[str, queue.Queue])`
    - `start(self)`: 인지 처리 스레드를 시작합니다.
    - `stop(self)`: 스레드를 중지합니다.
    - `run(self)`: `SensorData`를 소비하고 `PerceptionOutput`을 생성하는 메인 루프입니다.
- **입력**: `input_queue`의 `SensorData`.
- **출력**: `output_queues`로 분배되는 `PerceptionOutput` (측위, 예측, 계획 모듈용).

`PerceptionModule`이 관리하는 내부 컴포넌트:

### 탐지 컴포넌트

- **클래스**: `DetectionComponent`
- **목적**: 그림 3의 "탐지" 분기(차선 탐지, 주행 가능 영역, 교통 표지판 탐지, 시각적 3D 탐지, LiDAR 3D 탐지, 융합 3D 탐지)의 작업을 구현합니다.
- **인터페이스**: `process(self, sensor_data: SensorData, depth_map: Optional[Any]) -> Tuple[List[DetectedObject], List[LaneMarking], Any, List[TrafficSignInfo]]`

### 장면 이해 컴포넌트

- **클래스**: `SceneUnderstandingComponent`
- **목적**: 그림 3의 "장면 이해" 분기(의미론적/인스턴스/파노라마 분할, 깊이/광학 흐름/장면 흐름 추정)의 작업을 구현합니다.
- **인터페이스**: `process(self, sensor_data: SensorData, previous_frame_data: Optional[Any] = None) -> Tuple[Optional[Any], ...]`

### 추적 컴포넌트

- **클래스**: `TrackingComponent`
- **목적**: 그림 3의 "추적" 분기(전통적 추적, 신경망 추적)의 작업을 구현합니다. 시간에 따른 탐지를 연관시킵니다.
- **인터페이스**: `process(self, detected_objects: List[DetectedObject], timestamp: float) -> List[DetectedObject]`

### 인지 예측 컴포넌트 (단기)

- **클래스**: `PerceptionPredictionComponent`
- **목적**: 그림 3의 "예측" 분기(모델 기반 예측, 데이터 기반 예측)의 작업을 구현합니다. 이는 추적된 객체의 단기 상태 추정(예: 다음 몇 프레임의 위치/속도)을 위한 것입니다.
- **인터페이스**: `process(self, tracked_objects: List[DetectedObject]) -> List[DetectedObject]`

---

## 5. 측위 모듈 (`localization_module.py`)

차량의 전역 위치와 방향을 추정합니다. 그림 2의 "측위"에 해당하며 그림 3의 "측위" 분기(GNSS/IMU, 시각적 SLAM, LiDAR SLAM, 융합 SLAM)의 하위 작업을 사용합니다.

### 클래스: `HDMapInterface` (헬퍼)

- **목적**: 고정밀 지도 데이터에 액세스하기 위한 인터페이스를 제공합니다.
- **인터페이스**: `get_local_map_data(self, position, extent)`

### 클래스: `LocalizationModule`

- **목적**: 인지 모듈 및/또는 직접 센서의 센서 데이터와 HD 맵 정보를 융합하여 정확한 자세를 생성합니다.
- **인터페이스**:
    - `__init__(self, config: dict, hd_map_path: str, input_queue_perception: queue.Queue, input_queue_sensor: Optional[queue.Queue], output_queues: Dict[str, queue.Queue])`
    - `start(self)`: 측위 스레드를 시작합니다.
    - `stop(self)`: 스레드를 중지합니다.
    - `run(self)`: `PerceptionOutput`(및 선택적으로 직접 `SensorData`)을 소비하고 `LocalizationInfo`를 생성하는 메인 루프입니다.
- **입력**: `PerceptionOutput`(특징, 객체 목록 등 포함) 및/또는 직접 `SensorData`(GNSS/IMU).
- **출력**: `output_queues`로 `LocalizationInfo` 전송 (예측 및 계획 모듈용).

---

## 6. 예측 모듈 (행동) (`prediction_module.py`)

장면의 다른 동적 에이전트(차량, 보행자)의 미래 행동과 궤적을 예측합니다. 인지 및 측위로부터 입력을 받는 그림 2의 "예측" 블록에 해당합니다.

### 클래스: `PredictionModule`

- **목적**: 다른 도로 사용자의 장기적인 의도와 경로를 예측합니다.
- **인터페이스**:
    - `__init__(self, config: dict, input_queue_perception: queue.Queue, input_queue_localization: queue.Queue, output_queue: queue.Queue)`
    - `start(self)`: 예측 스레드를 시작합니다.
    - `stop(self)`: 스레드를 중지합니다.
    - `run(self)`: `PerceptionOutput` 및 `LocalizationInfo`를 소비하고 `BehavioralPredictionOutput`을 생성하는 메인 루프입니다.
- **입력**: `PerceptionOutput`(추적된 객체), `LocalizationInfo`(자차 상태).
- **출력**: `output_queue`로 `BehavioralPredictionOutput` 전송 (계획 모듈용).

---

## 7. 계획 모듈 (`planning_module.py`)

현재 상태, 환경 이해 및 다른 에이전트의 예측을 기반으로 자차의 미래 경로와 행동을 결정합니다. 그림 2의 "계획" 블록에 해당합니다.

### 클래스: `PlanningModule`

- **목적**: 경로 계획, 의사 결정 및 행동 계획을 조정합니다.
- **인터페이스**:
    - `__init__(self, config: dict, hd_map_path: str, input_queues: Dict[str, queue.Queue], output_queue_control: queue.Queue)`
    - `start(self)`: 계획 스레드를 시작합니다.
    - `stop(self)`: 스레드를 중지합니다.
    - `run(self)`: 입력을 소비하고 `ActionCommand`를 생성하는 메인 루프입니다.
- **입력**: `LocalizationInfo`, `BehavioralPredictionOutput` 및 `PerceptionOutput`(정적 장면 컨텍스트용).
- **출력**: `output_queue_control`로 `ActionCommand` 전송.

`PlanningModule`이 관리하는 내부 컴포넌트:

### 경로 계획 컴포넌트

- **클래스**: `PathPlannerComponent`
- **목적**: 자차를 위한 충돌 없는 기하학적 경로(루트)를 생성합니다.
- **인터페이스**: `plan_path(self, current_pose: LocalizationInfo, scene_info: PerceptionOutput, behavioral_predictions: BehavioralPredictionOutput) -> PlannedPath`

### 의사 결정 컴포넌트

- **클래스**: `DecisionMakerComponent`
- **목적**: 계획된 경로, 예측 및 규칙을 기반으로 상위 수준 주행 기동(예: 차선 유지, 차선 변경, 추월)을 선택합니다.
- **인터페이스**: `make_decision(self, current_pose: LocalizationInfo, planned_path: PlannedPath, behavioral_predictions: BehavioralPredictionOutput, scene_info: PerceptionOutput) -> ManeuverDecision`

### 행동 계획 컴포넌트

- **클래스**: `ActionPlannerComponent`
- **목적**: 결정된 기동과 경로를 상세하고 주행 가능한 궤적으로 변환하며, 종종 짧은 시간 범위에 대한 목표 속도 및 조향각 프로파일을 생성합니다.
- **인터페이스**: `plan_action(self, current_pose: LocalizationInfo, decision: ManeuverDecision, planned_path: PlannedPath) -> ActionCommand`

---

## 8. 제어 모듈 (`control_module.py`)

차량 액추에이터(조향, 스로틀, 브레이크)에 하위 수준 명령을 전송하여 계획된 행동을 실행합니다.

### 클래스: `VehicleInterface` (헬퍼)

- **목적**: 차량 하드웨어와의 통신을 추상화합니다.
- **인터페이스**: `send_commands(self, steering: float, throttle: float, brake: float)`

### 클래스: `ControlModule`

- **목적**: `ActionCommand`(예: 목표 속도, 조향각)를 `ControlActuatorCommands`로 변환합니다.
- **인터페이스**:
    - `__init__(self, config: dict, input_queue_planning: queue.Queue, vehicle_interface_config: dict)`
    - `start(self)`: 제어 스레드를 시작합니다.
    - `stop(self)`: 스레드를 중지합니다.
    - `run(self)`: `ActionCommand`를 소비하고 `VehicleInterface`를 통해 명령을 전송하는 메인 루프입니다.
- **입력**: `input_queue_planning`의 `ActionCommand`.
- **출력**: `VehicleInterface`를 통해 차량으로 전송되는 명령.

---

## 9. 주요 시스템 통합 (`main_system.py`)

모든 모듈을 초기화하고, 모듈 간 통신 큐를 설정하며, 각 스레드의 수명 주기(시작, 중지)를 관리합니다.

### 클래스: `MainSystem`

- **목적**: 자율 주행 파이프라인의 중앙 조정자입니다.
- **인터페이스**:
    - `__init__(self, config: dict)`
    - `start(self)`: 모든 모듈 스레드를 초기화하고 시작합니다.
    - `stop(self)`: 모든 모듈 스레드에 정상적으로 종료하도록 신호를 보내고 조인합니다.
- **기능**: 구성을 로드하고, 통신 큐를 생성하고, 모든 모듈을 인스턴스화하고, 실행을 관리합니다.

---

## 10. 모듈 간 통신

모듈 간 통신은 주로 파이썬의 `queue` 라이브러리에서 제공하는 스레드 안전 `queue.Queue` 객체를 사용하여 이루어집니다. 이는 모듈을 분리하고 각자의 처리 속도로 비동기적으로 작동할 수 있도록 합니다.

- `SensorInputManager` -> `PerceptionModule` (`sensor_to_perception_queue`를 통해)
- `PerceptionModule` -> `LocalizationModule` (`perception_to_localization_queue`를 통해)
- `PerceptionModule` -> `PredictionModule` (`perception_to_prediction_queue`를 통해)
- `PerceptionModule` -> `PlanningModule` (직접적인 장면 컨텍스트를 위해 `perception_to_planning_queue`를 통해)
- `LocalizationModule` -> `PredictionModule` (`localization_to_prediction_queue`를 통해)
- `LocalizationModule` -> `PlanningModule` (`localization_to_planning_queue`를 통해)
- `PredictionModule` -> `PlanningModule` (`prediction_to_planning_queue`를 통해)
- `PlanningModule` -> `ControlModule` (`planning_to_control_queue`를 통해)

각 큐는 일반적으로 `data_structures.py`에 정의된 데이터 구조의 객체를 보유합니다.

---

## 11. 스레딩 모델

각 코어 모듈(`PerceptionModule`, `LocalizationModule`, `PredictionModule`, `PlanningModule`, `ControlModule`)과 `SensorInputManager`는 자체 전용 스레드에서 실행되도록 설계되었습니다. 이를 통해 자율 주행 파이프라인의 여러 단계를 병렬로 처리할 수 있습니다.

- `MainSystem` 클래스는 이러한 스레드를 생성하고 관리하는 역할을 합니다.
- 각 모듈의 `start()` 메서드는 일반적으로 내부 처리 스레드를 초기화하고 시작합니다.
- `stop()` 메서드는 스레드에 종료 신호를 보내고 스레드를 `join()`하여 깔끔한 종료를 보장합니다.
- 큐는 무한정 차단을 방지하고 스레드가 종료 신호를 확인할 수 있도록 `get()` 및 `put()` 작업에 시간 초과를 사용합니다.

이러한 다중 스레드 설계는 시스템의 응답성과 처리량을 향상시키는 것을 목표로 하지만, 강력한 구현을 위해서는 동기화, 데이터 일관성 및 잠재적인 경쟁 조건(주로 큐 사용으로 완화됨)에 대한 신중한 고려가 필요합니다.