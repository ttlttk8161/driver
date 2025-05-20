# PlanningModule의 필수 입력 데이터 분석

## 1. 개요

자율주행 시스템의 `PlanningModule` (계획 모듈)은 차량의 다음 행동과 경로를 결정하는 핵심적인 역할을 수행합니다. `/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/Document.md`의 "## 10. Inter-Module Communication" 섹션에서 언급된 바와 같이, `PlanningModule`은 크게 세 가지 주요 모듈로부터 정보를 수신합니다:

*   `PerceptionModule` (인지 모듈)
*   `LocalizationModule` (위치 인식 모듈)
*   `PredictionModule` (예측 모듈)

이 문서에서는 `PlanningModule`이 성공적으로 주행 계획을 수립하기 위해 왜 이 세 가지 모듈로부터의 입력이 필수적인지 분석합니다.

## 2. 각 입력 모듈의 역할과 중요성

### 2.1. `LocalizationModule` (위치 인식 모듈)

*   **제공 정보**: 차량의 현재 정확한 위치(예: GPS 좌표, HD Map 상의 위치), 방향, 속도 등 (`LocalizationInfo` 데이터 구조 참고).
*   **`PlanningModule`에서의 중요성**:
    *   **"나는 지금 어디에 있는가?"**: 모든 계획 수립의 가장 기본적인 전제입니다. 차량의 현재 상태를 정확히 알아야 목표 지점까지의 경로를 생성하고, 지도 정보를 활용하며, 다른 객체와의 상대적인 관계를 파악할 수 있습니다.
    *   정확한 자기 위치 정보 없이는 경로 계획 자체가 불가능하거나 매우 부정확해질 수 있습니다.

### 2.2. `PerceptionModule` (인지 모듈)

*   **제공 정보**: 차량 주변 환경에 대한 실시간 정보. 예를 들어, 차선 정보, 신호등 상태, 교통 표지판, 정적/동적 장애물(다른 차량, 보행자, 물체 등)의 위치 및 유형 (`PerceptionOutput` 데이터 구조 참고).
*   **`PlanningModule`에서의 중요성**:
    *   **"내 주변에는 무엇이 있는가?"**: 안전하고 규칙을 준수하는 주행을 위해 필수적입니다.
    *   차선을 유지하고, 장애물을 회피하며, 교통 신호 및 표지판에 따라 적절한 행동(정지, 진행, 속도 조절 등)을 계획하려면 주변 환경에 대한 정확한 인지가 선행되어야 합니다.
    *   예를 들어, `/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/planning_module.py`의 `ActionPlannerComponent`는 `PerceptionOutput`에 포함된 HSV 차선 감지 결과(`white_line_hsv_metrics`, `yellow_line_hsv_metrics`)를 직접 사용하여 조향각을 결정합니다.

### 2.3. `PredictionModule` (예측 모듈)

*   **제공 정보**: 주변의 다른 동적 객체들(주로 다른 차량이나 보행자)이 미래에 어떻게 행동할지에 대한 예측 정보. 예를 들어, 특정 차량의 예상 경로, 차선 변경 가능성 등 (`BehavioralPredictionOutput` 데이터 구조 참고).
*   **`PlanningModule`에서의 중요성**:
    *   **"주변의 다른 객체들은 앞으로 어떻게 움직일 것인가?"**: 단순히 현재 상태만으로는 안전하고 효율적인 계획을 세우기 어렵습니다.
    *   다른 차량의 예상 경로를 미리 파악하면 충돌을 회피하기 위한 선제적인 감속, 차선 변경 등의 계획을 세울 수 있습니다.
    *   이를 통해 보다 부드럽고 안전하며, 교통 흐름에 맞는 주행 전략을 수립할 수 있습니다.

## 3. 코드상의 의존성 확인

`/home/xytron/xycar_ws/src/kookmin/driver/Original/Modules/planning_module.py` 파일 내 `PlanningModule` 클래스의 `run` 메서드를 살펴보면 다음과 같은 코드가 있습니다:

```python
            # Only proceed if we have essential data (at least localization)
            if self._latest_localization and self._latest_prediction and self._latest_perception :
                # ... (계획 수립 로직) ...
```

이 조건문은 `LocalizationModule`, `PredictionModule`, `PerceptionModule`로부터 최신의 데이터를 모두 수신했을 때만 `PlanningModule`이 본격적인 계획 수립 단계를 진행하도록 명시하고 있습니다. 이는 이 세 가지 정보가 다음 단계의 계획을 위해 누락되어서는 안 될 필수적인 요소임을 코드 수준에서도 뒷받침합니다.

## 4. 결론

`PlanningModule`이 신뢰할 수 있고 안전하며 효율적인 주행 계획을 수립하기 위해서는 차량 자신의 정확한 상태(`Localization`), 주변 환경에 대한 상세한 이해(`Perception`), 그리고 다른 객체들의 미래 행동에 대한 예측(`Prediction`)이 반드시 필요합니다. 이 세 가지 정보는 상호 보완적으로 작용하여 복잡한 실제 도로 상황에 대응할 수 있는 지능적인 주행 판단의 기초를 제공합니다.