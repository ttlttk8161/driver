### 2.1 중심선 구축 (Construct Center Line)

논문 내용에 기반하여 '중심선 구축' 단계의 슈도코드(Pseudocode)를 제공해 드리겠습니다. 이 단계는 사전에 정의된 웨이포인트로부터 자율 주행 차량이 따라갈 도로의 중심선을 생성하고, 이를 호 길이(arc length)를 매개변수로 하는 입방 스플라인으로 표현하는 과정입니다.

```
Function ConstructCenterLine(predefined_waypoints):
  // 2.1. Center line construction (중심선 구축)
  // 입력: predefined_waypoints (도로의 경계 또는 차선을 나타내는 웨이포인트들의 리스트)
  // 출력: center_line_segments (호 길이로 매개변수화된 입방 스플라인 구간들의 리스트)
  //       cumulative_arc_lengths (각 중심 웨이포인트까지의 누적 호 길이 리스트)

  // 1. 중심 웨이포인트 식별 및 생성
  // 차선 수준 지도에서 얻은 웨이포인트 쌍의 중심점을 계산하여 중심 웨이포인트를 생성합니다.
  center_waypoints = []
  For each pair (waypoint_left, waypoint_right) in predefined_waypoints:
    center_x = (waypoint_left.x + waypoint_right.x) / 2
    center_y = (waypoint_left.y + waypoint_right.y) / 2
    Add (center_x, center_y) to center_waypoints

  // 2. 각 구간에 대한 매개변수 입방 스플라인 구축 및 호 길이 매개변수화
  center_line_segments = []
  cumulative_arc_lengths = [0.0] // 시작점의 누적 호 길이는 0

  For i from 0 to length(center_waypoints) - 2:
    start_point = center_waypoints[i]
    end_point = center_waypoints[i+1]

    // 현재 중심 웨이포인트 구간 (start_point 에서 end_point 까지) 에 대한 임시 입방 스플라인 생성
    // 이는 일반적인 매개변수 t를 사용하는 형태 (x_temp(t), y_temp(t)) 일 수 있습니다.
    // (참고: 실제 구현에서는 전역 스플라인을 먼저 만들고 구간별로 처리하거나,
    // 각 구간 스플라인을 만들 때 연속성을 위한 경계 조건 처리가 필요합니다.)
    temp_spline_segment = FitCubicSplineSegment(start_point, end_point, continuity_constraints)

    // 생성된 임시 스플라인 구간의 총 호 길이 수치적 계산 (적분 방법 사용)
    segment_arc_length = CalculateArcLengthNumerically(temp_spline_segment)

    // 누적 호 길이 업데이트
    Add cumulative_arc_lengths[-1] + segment_arc_length to cumulative_arc_lengths

    // 임시 스플라인 구간을 호 길이(s_local)로 매개변수화된 형태로 변환
    // 즉, (x_temp(t), y_temp(t)) 형태를 (x0(s_local), y0(s_local)) 형태로 변환합니다.
    // 여기서 s_local은 이 구간 시작점(start_point)으로부터의 호 길이이며 범위는 [0, segment_arc_length] 입니다.
    // 이 과정을 통해 식 (1)의 계수 (ax, bx, cx, dx, ay, by, cy, dy)가 구해집니다.
    arc_length_parameterized_segment = ReparameterizeSegmentByArcLength(temp_spline_segment, segment_arc_length)

    // 호 길이로 매개변수화된 구간 스플라인을 리스트에 추가
    Add arc_length_parameterized_segment to center_line_segments

  // 최종 누적 호 길이 리스트는 center_waypoints의 개수와 동일한 크기가 됩니다.
  // center_line_segments 리스트는 center_waypoints 개수보다 하나 적은 크기가 됩니다.

  Return center_line_segments, cumulative_arc_lengths

// Helper Functions (세부 구현이 추상화된 개념적 함수)

Function FitCubicSplineSegment(start_point, end_point, continuity_constraints):
  // 입력: start_point, end_point (스플라인 구간의 시작 및 끝 웨이포인트)
  //       continuity_constraints (인접 구간과의 연속성을 보장하기 위한 미분 값 등의 제약 조건)
  // 출력: temp_spline_segment (일반 매개변수 t를 사용하는 입방 스플라인 구간 표현)
  // 이 함수는 start_point와 end_point를 지나며 주어진 제약 조건을 만족하는 입방 스플라인 구간을 계산합니다.
  // 이는 보간법이나 최적화 문제를 통해 구현될 수 있습니다.
  Pass // 이 함수의 구체적인 구현은 논문의 범위를 넘어설 수 있습니다.

Function CalculateArcLengthNumerically(spline_segment):
  // 입력: spline_segment (일반 매개변수 t를 사용하는 입방 스플라인 구간)
  // 출력: segment_arc_length (해당 구간의 총 호 길이)
  // 이 함수는 스플라인 구간의 호 길이 (Integral of sqrt((dx/dt)^2 + (dy/dt)^2) dt)를 수치적으로 계산합니다.
  // 논문에서는 구적법(quadrature method)을 사용한다고 언급하고 있습니다.
  Pass

Function ReparameterizeSegmentByArcLength(temp_spline_segment, segment_length):
  // 입력: temp_spline_segment (일반 매개변수 t를 사용하는 입방 스플라인 구간)
  //       segment_length (해당 구간의 총 호 길이)
  // 출력: arc_length_parameterized_segment (호 길이 s_local로 매개변수화된 입방 스플라인 구간 표현 및 계수)
  // 이 함수는 기존의 스플라인 구간 표현을, 해당 구간의 시작점으로부터의 호 길이 s_local을
  // 매개변수로 하는 새로운 입방 스플라인 표현 (식 (1) 형태)으로 변환합니다.
  // 이는 s_local에 따른 x0, y0 값을 계산하고, 이 점들을 사용하여 새로운 입방 스플라인을 피팅하는 방식으로 구현될 수 있습니다.
  // 이 과정을 통해 식 (1)의 계수 ax, bx, cx, dx, ay, by, cy, dy가 도출됩니다.
  Pass
```

**슈도코드 설명:**

1.  **중심 웨이포인트 생성**: 입력받은 웨이포인트들로부터 도로의 중심에 해당하는 지점들(중심 웨이포인트)을 계산하여 리스트로 만듭니다.
2.  **구간 스플라인 구축 및 매개변수화**: 연속된 중심 웨이포인트 두 개를 하나의 구간으로 삼아, 각 구간별로 독립적인 입방 스플라인을 만듭니다.
    *   먼저 일반적인 매개변수(예: `t`)를 사용하는 임시 스플라인을 피팅합니다. 이때 전체 중심선이 부드럽게 이어지도록 인접 구간과의 연속성 제약 조건(예: 접선 벡터나 곡률의 연속성)을 고려하여 스플라인을 만듭니다.
    *   생성된 임시 스플라인 구간의 실제 호 길이(arc length)를 수치적 적분 방법을 사용하여 계산합니다.
    *   이 임시 스플라인을 해당 구간의 시작점으로부터 측정한 호 길이 `s_local`을 매개변수로 하는 새로운 입방 스플라인으로 변환합니다. 이 변환 과정을 통해 논문의 식 (1)에 해당하는 계수들(`ax, bx, ...`)이 계산됩니다.
    *   각 구간의 스플라인과 해당 구간 끝점까지의 누적 호 길이를 저장합니다.
3.  **최종 출력**: 모든 구간에 대해 이 과정을 반복하면, 호 길이로 매개변수화된 입방 스플라인 구간들의 리스트와 각 중심 웨이포인트까지의 누적 호 길이 리스트를 얻게 됩니다. 이 정보를 통해 중심선 상의 임의의 지점을 호 길이를 이용해 정확하게 파악할 수 있습니다.



### 2.2 경로 후보 생성 (Path Candidates Generation)

```pseudocode
// Section 2.2: Path candidates generation (경로 후보 생성)

// Input:
//   vehicle_position_cartesian: 차량의 현재 위치 (x, y)
//   vehicle_heading_angle: 차량의 현재 진행 방향 각도
//   center_line_params: 중심선을 표현하는 파라미터 (예: 3차 스플라인 계수)
//   center_line_function(s): 중심선 위의 s 위치에 해당하는 카르테시안 좌표 (x0, y0)를 반환하는 함수
//   center_line_tangent_angle_function(s): 중심선 위의 s 위치에 해당하는 접선 각도 (h0)를 반환하는 함수
//   center_line_curvature_function(s): 중심선 위의 s 위치에 해당하는 곡률 (j0)를 반환하는 함수
//   s_end_arc_length: 경로 후보의 목표 호 길이 증가량 (s_start로부터의 거리)
//   target_lateral_offsets_q_end: 경로 후보 끝점의 목표 횡방향 오프셋 리스트 (예: [-3.0, -2.5, ..., 2.5, 3.0])
//   num_points_per_path: 각 경로 후보를 표현할 점의 개수 (좌표 변환 시 사용)

// Output:
//   list_of_path_candidates_cartesian: 생성된 각 경로 후보의 카르테시안 좌표 점들 리스트

FUNCTION GeneratePathCandidates(vehicle_position_cartesian, vehicle_heading_angle, center_line_params, center_line_function, center_line_tangent_angle_function, center_line_curvature_function, s_end_arc_length, target_lateral_offsets_q_end, num_points_per_path):

    list_of_path_candidates_cartesian = []

    // 2.2.1 Localization on the center line (중심선 상 로컬라이제이션)
    // 차량의 현재 위치를 중심선 상의 s-q 좌표로 변환
    // (이 과정은 논문 [41]을 참조하여 구현)
    CALL LocalizeOnCenterLine(vehicle_position_cartesian, center_line_function, center_line_tangent_angle_function, center_line_curvature_function)
    GET s_start, q_start, center_line_tangent_at_s_start FROM LocalizeOnCenterLine_Result

    // 현재 차량 진행 방향과 중심선 접선 방향 간의 각도 차이
    delta_h_start = vehicle_heading_angle - center_line_tangent_at_s_start

    // 경로 후보의 목표 s 값 (s_start로부터의 거리)
    s_end = s_start + s_end_arc_length

    // 2.2.2 Path candidates generation in the s-q coordinate system (s-q 좌표계에서 경로 후보 생성)
    FOR EACH q_end IN target_lateral_offsets_q_end:

        // 각 q_end에 대해 3차 함수 q(s)의 계수 (a, b, c) 계산
        // 경계 조건:
        // q(s_start) = q_start
        // q(s_end) = q_end
        // q'(s_start) = tan(delta_h_start)
        // q'(s_end) = 0
        // 이 4개의 조건을 만족하는 3차 함수 q(s) = a*(s-s_start)^3 + b*(s-s_start)^2 + c*(s-s_start) + q_start 에 대해
        // a, b, c를 계산 (선형 시스템 해법 사용)
        CALL SolveCubicCoefficients(s_start, q_start, s_end, q_end, delta_h_start)
        GET a, b, c FROM SolveCubicCoefficients_Result

        // 2.2.3 Coordinates conversion of path candidate points (경로 후보 점들의 좌표 변환)
        // s-q 좌표계에서 정의된 경로 후보를 카르테시안 좌표계로 변환
        current_path_candidate_cartesian = []
        current_x = vehicle_position_cartesian.x // 시작점 x
        current_y = vehicle_position_cartesian.y // 시작점 y
        current_heading = vehicle_heading_angle // 시작점 heading

        // s_start부터 s_end까지 일정한 간격으로 점들을 생성
        FOR i FROM 0 TO num_points_per_path:
            s_current = s_start + (s_end_arc_length / num_points_per_path) * i

            // 현재 s 값에서의 q(s), q'(s), q''(s) 값 계산
            q_current = a*(s_current-s_start)^3 + b*(s_current-s_start)^2 + c*(s_current-s_start) + q_start
            // q'(s) 계산 (s-s_start로 미분)
            dqds_current = 3*a*(s_current-s_start)^2 + 2*b*(s_current-s_start) + c
            // q''(s) 계산 (s-s_start로 두 번 미분)
            d2qds2_current = 6*a*(s_current-s_start) + 2*b

            // 중심선 상의 s_current 위치 정보 가져오기
            // (x0_current, y0_current) = center_line_function(s_current) // 이 값들은 변환 자체에 직접 사용되지는 않지만 디버깅/시각화에 유용
            j0_current = center_line_curvature_function(s_current) // 중심선 곡률

            // 식 (8)에 따라 A, B 계산
            A = SQRT(dqds_current^2 + (1 - q_current * j0_current)^2)
            B = SIGN(1 - q_current * j0_current) // sgn 함수

            // 식 (7)에 따라 경로 곡률 j 계산
            j_current = (B/A) * ( (1 - q_current * j0_current) * d2qds2_current + j0_current * dqds_current^2 ) / A^2

            // 식 (6)의 미분 값을 계산 (ds에 대한 변화량)
            dx_ds = A * COS(current_heading)
            dy_ds = A * SIN(current_heading)
            dh_ds = A * j_current

            // 작은 ds 간격 (ds = s_end_arc_length / num_points_per_path)에 대해 적분 (오일러 적분 등)
            delta_s = s_end_arc_length / num_points_per_path
            current_x = current_x + dx_ds * delta_s
            current_y = current_y + dy_ds * delta_s
            current_heading = current_heading + dh_ds * delta_s

            // 카르테시안 좌표 점 추가
            ADD (current_x, current_y) TO current_path_candidate_cartesian

        // 변환된 경로 후보를 리스트에 추가
        ADD current_path_candidate_cartesian TO list_of_path_candidates_cartesian

    // 모든 경로 후보 생성 완료
    RETURN list_of_path_candidates_cartesian

END FUNCTION

// Helper function to localize vehicle on the center line (논문 [41] 참조)
FUNCTION LocalizeOnCenterLine(vehicle_position_cartesian, center_line_function, center_line_tangent_angle_function, center_line_curvature_function):
    // 구현 세부 사항은 논문 [41] 참조
    // vehicle_position_cartesian (x_veh, y_veh)와 가장 가까운
    // 중심선 상의 점 (x0, y0)을 찾고, 그 점의 s 값 (s_start),
    // (x_veh, y_veh)에서 (x0, y0)까지의 거리 q_start (횡방향 오프셋),
    // 그리고 s_start에서의 중심선 접선 각도 h0_start를 반환
    // ... 구현 ...
    RETURN s_start, q_start, h0_start
END FUNCTION

// Helper function to solve for cubic coefficients (a, b, c) (식 (5) 기반)
FUNCTION SolveCubicCoefficients(s_start, q_start, s_end, q_end, delta_h_start):
    // q(s) = a*(s-s_start)^3 + b*(s-s_start)^2 + c*(s-s_start) + q_start
    // q'(s) = 3*a*(s-s_start)^2 + 2*b*(s-s_start) + c
    // let ds = s_end - s_start
    // q(s_end) = q_end  => a*ds^3 + b*ds^2 + c*ds + q_start = q_end
    // q'(s_end) = 0    => 3*a*ds^2 + 2*b*ds + c = 0
    // q'(s_start) = tan(delta_h_start) => 3*a*(s_start-s_start)^2 + 2*b*(s_start-s_start) + c = tan(delta_h_start) => c = tan(delta_h_start)

    // From c = tan(delta_h_start):
    // 3*a*ds^2 + 2*b*ds + tan(delta_h_start) = 0
    // a*ds^3 + b*ds^2 + tan(delta_h_start)*ds + q_start = q_end

    // Solve the 2x2 system for a and b:
    // 3*a*ds^2 + 2*b*ds = -tan(delta_h_start)
    // a*ds^3 + b*ds^2 = q_end - q_start - tan(delta_h_start)*ds

    // ... solve for a, b using matrix inversion or substitution ...

    // Return a, b, c
    c = tan(delta_h_start)
    // Example solution structure (simplified, need to implement actual linear system solve):
    // [ 3*ds^2  2*ds ] [ a ] = [ -tan(delta_h_start)         ]
    // [ ds^3    ds^2 ] [ b ]   [ q_end - q_start - c*ds ]
    // Use numpy.linalg.solve or similar in Python
    // For pseudocode, just indicate the solve operation
    a, b = SOLVE([ [3*ds^2, 2*ds], [ds^3, ds^2] ], [-tan(delta_h_start), q_end - q_start - c*ds])

    RETURN a, b, c
END FUNCTION

// Helper functions for math (assuming standard availability)
FUNCTION SQRT(x): // Square root
FUNCTION COS(angle): // Cosine (angle in radians)
FUNCTION SIN(angle): // Sine (angle in radians)
FUNCTION TAN(angle): // Tangent (angle in radians)
FUNCTION SIGN(x): // Sign function (returns +1, -1, or 0)
FUNCTION SOLVE(matrix, vector): // Solves a linear system represented by matrix * [a, b] = vector
```

**슈도코드 설명:**

1.  **`GeneratePathCandidates` 함수:** 전체 경로 후보 생성 과정을 담당하는 메인 함수입니다. 필요한 모든 입력 데이터를 받아 경로 후보들의 카르테시안 좌표 목록을 반환합니다.
2.  **2.2.1 Localization on the center line:**
    *   차량의 현재 카르테시안 위치 (`vehicle_position_cartesian`)를 입력받습니다.
    *   `LocalizeOnCenterLine` 보조 함수를 호출하여 이 위치에 가장 가까운 중심선 상의 점을 찾습니다.
    *   결과로 그 점의 호 길이 `s_start`, 차량에서 그 점까지의 횡방향 오프셋 `q_start`, 그리고 `s_start`에서의 중심선 접선 각도 `center_line_tangent_at_s_start`를 얻습니다.
    *   차량의 현재 헤딩(`vehicle_heading_angle`)과 중심선 접선 각도의 차이인 `delta_h_start`를 계산합니다. 이는 경로 시작점에서의 각도 조건을 설정하는 데 사용됩니다.
    *   경로 후보의 끝점이 될 중심선 상의 목표 호 길이 `s_end`를 계산합니다.
3.  **2.2.2 Path candidates generation in the s-q coordinate system:**
    *   미리 정의된 여러 개의 목표 횡방향 오프셋 값 리스트 `target_lateral_offsets_q_end`를 순회합니다. 이 값들이 각기 다른 경로 후보의 끝점 오프셋 `q_end`가 됩니다.
    *   각 `q_end` 값에 대해 `SolveCubicCoefficients` 보조 함수를 호출합니다. 이 함수는 시작점 (`s_start`, `q_start`, `delta_h_start`) 및 끝점 (`s_end`, `q_end`, 끝점 기울기 0)의 경계 조건을 만족하는 3차 함수 `q(s)`의 계수 `a`, `b`, `c`를 계산합니다.
4.  **2.2.3 Coordinates conversion of path candidate points:**
    *   `s-q` 좌표계에서 `a`, `b`, `c` 계수로 정의된 각 경로 후보에 대해, 이를 카르테시안 좌표로 변환합니다.
    *   `s_start`부터 `s_end`까지 일정한 간격으로 `num_points_per_path` 개수의 `s_current` 값들을 생성합니다.
    *   각 `s_current` 값에 대해 해당 경로 후보의 `q_current`, `dqds_current`, `d2qds2_current` 값을 계산합니다.
    *   `s_current`에서의 중심선 곡률 `j0_current`를 가져옵니다.
    *   식 (8)에 따라 보조 변수 `A`와 `B`를 계산합니다.
    *   식 (7)에 따라 현재 점에서의 경로 곡률 `j_current`를 계산합니다.
    *   식 (6)은 카르테시안 좌표와 헤딩 각도의 `s`에 대한 미분(`dx/ds`, `dy/ds`, `dh/ds`)을 나타냅니다. 이 미분 값들을 계산합니다.
    *   `s`의 작은 간격(`delta_s`)에 대해 이 미분 값들을 적분(슈도코드에서는 오일러 적분처럼 표현)하여 다음 점의 `current_x`, `current_y`, `current_heading`를 계산합니다. 이 과정은 `s_start`에서 시작하여 `s_end`까지 반복되며, 각 반복에서 얻은 `(current_x, current_y)` 좌표가 경로 후보를 구성하는 점이 됩니다.
    *   계산된 카르테시안 좌표 점들을 `current_path_candidate_cartesian` 리스트에 추가합니다.
    *   하나의 `q_end`에 대한 처리가 끝나면, 완성된 `current_path_candidate_cartesian` 리스트를 최종 결과 리스트 `list_of_path_candidates_cartesian`에 추가합니다.
5.  **보조 함수 (`LocalizeOnCenterLine`, `SolveCubicCoefficients`):** 논문에서 사용된 특정 방법(LocalizeOnCenterLine은 [41] 참조, SolveCubicCoefficients는 식 (5) 기반 선형 시스템 해법)을 캡슐화한 함수입니다. 슈도코드에서는 그 역할과 입/출력만 명시하고 내부 구현 세부 사항은 생략했습니다.

이 슈도코드는 논문의 설명을 따랐으며, 실제 구현 시에는 각 단계별 세부 계산 및 수치 해석 방법을 정확히 적용해야 합니다.

#### 2.2.1. Localization on the center line


논문의 "Material and methods" 섹션(Section 2)에 설명된 경로 계획 방법을 기반으로 한 슈도코드(Pseudocode)를 제공해 드리겠습니다. 이 슈도코드는 논문의 주요 단계와 계산 과정을 요약한 것입니다.

```pseudocode
Function DynamicPathPlanning(waypoints, current_vehicle_state, static_obstacles, moving_obstacles, previous_path)

  // current_vehicle_state: 차량의 현재 위치(x, y), 속도, 방향 정보
  // waypoints: 도로의 중심을 나타내는 미리 정의된 지점 집합 (lane-level map에서 얻음)
  // static_obstacles: 정지된 장애물 정보 (위치, 크기 등)
  // moving_obstacles: 움직이는 장애물 정보 (위치, 속도, 방향 등)
  // previous_path: 직전 계획 단계에서 선택된 경로

  // 1. 중심선 구축 (Center line construction, Section 2.1)
  center_line = ConstructCenterLine(waypoints) // Cubic spline fitting 사용, arc length로 매개변수화
  
  // 2. 차량 위치를 s-q 좌표계로 변환 및 중심선 상 위치 찾기 (Localization, Section 2.2.1)
  (s_start, q_start) = MapVehiclePositionToSQ(current_vehicle_state.position, center_line) // Quadratic minimization + Newton's method
  relative_heading_start = current_vehicle_state.heading - GetCenterLineHeading(center_line, s_start) // Dh_start

  // 3. 경로 후보 생성 (Path candidates generation, Section 2.2)
  path_candidates = []
  possible_q_end_values = GenerateFiniteSetOfQEndValues() // q_end: 경로 끝 지점의 횡방향 오프셋 값 집합
  arc_length_delta = DeterminePathLength(current_vehicle_state.speed) // s_end - s_start
  s_end = s_start + arc_length_delta
  
  For each q_end in possible_q_end_values:
    // s-q 좌표계에서 경로 정의 (Section 2.2.2)
    // q(s) = a(s - s_start)^3 + b(s - s_start)^2 + c(s - s_start) + q_start
    // 경계 조건: q(s_start)=q_start, q(s_end)=q_end, dq/ds(s_start)=tan(relative_heading_start), dq/ds(s_end)=0
    (a, b, c) = SolveCubicPolynomialCoefficients(s_start, q_start, relative_heading_start, s_end, q_end)
    
    current_path_candidate_sq = FunctionQ(s, a, b, c) // s에 대한 q 값 계산 함수
    
    // s-q 좌표계 경로를 Cartesian 좌표계로 변환 (Section 2.2.3)
    current_path_candidate_cartesian = ConvertSQToCartesian(current_path_candidate_sq, center_line) // 수식 (6), (7), (8) 사용
    
    path_candidates.Add(current_path_candidate_cartesian)

  // 4. 최적 경로 선택 (Path selection, Section 2.3)
  min_total_cost = Infinity
  optimal_path = null
  optimal_acceleration = null
  
  For each path_candidate_i in path_candidates:
    // 해당 경로 후보에 대한 비용 계산
    
    // 4.1. 정적 안전 비용 계산 (Cost function for static safety, Section 2.3.1)
    collision_check_values = CalculateCollisionCheck(path_candidate_i, road_edges, lane_lines, static_obstacles) // 0, 0.2, 0.5, 1 값 할당
    static_safety_cost_i = ApplyGaussianConvolution(collision_check_values) // 수식 (10), (11) 사용
    
    // 4.2. 편안함 비용 계산 (Cost function for comfortability, Section 2.3.2)
    smoothness_cost_i = CalculateSmoothnessCost(path_candidate_i) // 수식 (12) 사용 (곡률 제곱 적분)
    consistency_cost_i = CalculateConsistencyCost(path_candidate_i, previous_path) // 수식 (13) 사용 (이전 경로와의 헤딩 차이 적분)
    comfortability_cost_i = alpha * smoothness_cost_i + beta * consistency_cost_i // 수식 (14) 사용 (alpha, beta는 가중치)
    
    // 4.3. 동적 안전 비용 계산 (Cost function for dynamic safety, Section 2.3.3)
    // 움직이는 장애물과의 충돌 가능성 및 회피 전략 고려 (따라가기 전략 가정)
    potential_collision_points = FindCollisionPointsWithMovingObstacles(path_candidate_i, moving_obstacles)
    
    If potential_collision_points Exist:
      time_to_collision_i = CalculateTimeToCollision(current_vehicle_state.speed, distance_to_collision_point) // 수식 (15)
      required_acceleration_i = CalculateRequiredAcceleration(current_vehicle_state.speed, distance_to_collision_point, time_to_collision_i, following_distance) // 수식 (16), (17)
      
      // 속도 제한 적용 및 가속도 조정 (수식 18, 19, 20, 21)
      speed_limit_i = CalculateSpeedLimit(path_candidate_i, road_info)
      adjusted_acceleration_i = AdjustAccelerationBasedOnSpeedLimit(required_acceleration_i, current_vehicle_state.speed, speed_limit_i, arc_length_delta)
      
      dynamic_safety_cost_i = abs(adjusted_acceleration_i) * (distance_traveled_on_path) // 수식 (22) (거리 = Ds - Lf 근사 또는 다른 기준 사용)
    Else:
      dynamic_safety_cost_i = 0 // 움직이는 장애물 관련 비용 없음
      adjusted_acceleration_i = CalculateOptimalAccelerationWithoutMovingObstacles(...) // 정적/편안함 고려 가속도
    
    // 가속도 스무딩 (Gaussian filtering) - 여러 계획 단계에 걸쳐 적용될 수 있음
    smoothed_acceleration_i = SmoothAcceleration(adjusted_acceleration_i, past_accelerations) // 논문 (44) 참고
    
    // 4.4. 총 비용 계산 및 최적 경로 업데이트 (Section 2.3.4)
    // 비용 함수들을 0-1 사이 값으로 정규화
    normalized_static_safety_cost_i = Normalize(static_safety_cost_i)
    normalized_comfortability_cost_i = Normalize(comfortability_cost_i)
    normalized_dynamic_safety_cost_i = Normalize(dynamic_safety_cost_i)
    
    // 총 비용 계산 (ws, wc, wd는 가중치)
    total_cost_i = ws * normalized_static_safety_cost_i + wc * normalized_comfortability_cost_i + wd * normalized_dynamic_safety_cost_i // 수식 (23)
    
    If total_cost_i < min_total_cost:
      min_total_cost = total_cost_i
      optimal_path = path_candidate_i
      optimal_acceleration = smoothed_acceleration_i // 스무딩된 가속도 선택

  // 5. 결과 반환
  Return optimal_path, optimal_acceleration, DerivedOptimalSpeed(current_vehicle_state.speed, optimal_acceleration, arc_length_delta)

End Function
```

**설명:**

*   이 슈도코드는 논문에서 제시된 자율 주행 차량의 동적 경로 계획 방법의 핵심 로직을 단계별로 나타냅니다.
*   `ConstructCenterLine`, `MapVehiclePositionToSQ`, `SolveCubicPolynomialCoefficients`, `ConvertSQToCartesian` 등과 같이 실제 계산 로직이 필요한 부분은 해당 기능을 수행하는 함수 호출로 표현했습니다. 이 함수들의 구체적인 구현은 논문의 해당 섹션에 설명되어 있습니다.
*   비용 계산 부분(`CalculateCollisionCheck`, `ApplyGaussianConvolution`, `CalculateSmoothnessCost`, `CalculateConsistencyCost`, `FindCollisionPointsWithMovingObstacles`, `CalculateTimeToCollision`, `CalculateRequiredAcceleration`, `CalculateSpeedLimit`, `AdjustAccelerationBasedOnSpeedLimit`, `SmoothAcceleration`) 또한 논문의 해당 수식과 설명을 기반으로 구현될 상세 로직을 추상화한 것입니다.
*   비용 함수의 가중치(`ws`, `wc`, `wd`, `alpha`, `beta`)는 논문에서 예시 값(ws=0.5, wc=0.2, wd=0.3)과 결정 방법(주행 스타일에 따라 결정, 머신러닝으로 최적화 가능성 언급)이 제시되어 있습니다.
*   이 과정은 실시간 시스템 요구사항을 충족하기 위해 매 시간 단계마다 반복 실행됩니다 (논문에서는 15Hz 언급).

이 슈도코드를 통해 논문의 경로 계획 알고리즘이 어떻게 구성되고 각 단계가 어떤 역할을 하는지 이해하는 데 도움이 되셨기를 바랍니다.

#### 2.2.2. Path candidates generation in the s  q coordinate system

이 슈도코드는 논문의 2.2.2 절 "Path candidates generation in the s-q coordinate system"의 내용을 기반으로 작성되었습니다. 중심선을 기준으로 차량의 현재 상태와 계획 종료 지점의 횡방향 오프셋을 바탕으로 여러 개의 경로 후보를 생성하는 과정을 설명합니다.

```pseudocode
프로세스 PathCandidateGeneration(vehicle_state, center_line_info, planning_horizon)
  // 입력:
  //   vehicle_state: 현재 차량의 Cartesian 좌표 (x_veh, y_veh), 헤딩 각도 (heading_veh)
  //   center_line_info: 중심선을 표현하는 cubic spline 함수 (s를 입력으로 x0(s), y0(s), h0(s), j0(s) 및 미분값 반환)
  //   planning_horizon: 경로를 계획할 호 길이 간격 (Delta_s) 또는 종료 호 길이 (s_end)

  // 출력:
  //   path_candidates: 생성된 경로 후보 리스트 (각 후보는 Cubic Spline 계수 (a, b, c)와 관련 정보 포함)

  path_candidates = empty list

  // 1. 중심선 상의 차량 위치 파악 (섹션 2.2.1)
  //    - 차량의 Cartesian 좌표 (x_veh, y_veh)를 s-q 좌표계로 변환
  //    - 중심선에서 가장 가까운 점 p0를 찾고, 해당 점의 호 길이 s_start와 횡방향 오프셋 q_start 계산
  //    - 차량의 헤딩 각도와 p0에서의 중심선 접선 각도 h0(s_start)의 차이 Delta_h_start 계산
  (s_start, q_start) = ConvertCartesianToSQ(vehicle_state.x, vehicle_state.y, center_line_info)
  tangent_angle_at_s_start = center_line_info.get_heading(s_start)
  Delta_h_start = vehicle_state.heading - tangent_angle_at_s_start
  tan_Delta_h_start = tan(Delta_h_start)

  // 2. 계획 종료 호 길이 s_end 결정
  s_end = s_start + planning_horizon.Delta_s // 또는 planning_horizon에서 직접 s_end를 얻음

  // 3. 경로 후보를 생성할 끝점 횡방향 오프셋(q_end) 값들 설정
  //    - 예: 현재 차선 폭, 도로 종류 등을 고려하여 가능한 q_end 값의 범위를 정하고 일정한 간격으로 샘플링
  q_end_set = GenerateQEndValues(current_lane_width, road_type, number_of_candidates)

  // 4. 각 q_end 값에 대해 경로 후보 생성
  for each q_end in q_end_set
    // Cubic Spline q(s) = a*(s - s_start)^3 + b*(s - s_start)^2 + c*(s - s_start) + q_start 의 계수 a, b, c 결정

    // 경계 조건 (eq 5):
    // q(s_start) = q_start  (자동 만족)
    // q(s_end) = q_end
    // q'(s_start) = tan(Delta_h_start)
    // q'(s_end) = 0

    // 미분 q'(s) = 3a*(s - s_start)^2 + 2b*(s - s_start) + c

    // 조건 대입:
    // s = s_start 일 때 q'(s_start) = c
    c = tan_Delta_h_start

    // s = s_end 일 때 q(s_end) = q_end
    // q_end = a*(s_end - s_start)^3 + b*(s_end - s_start)^2 + c*(s_end - s_start) + q_start
    // q_end - q_start - c*(s_end - s_start) = a*(s_end - s_start)^3 + b*(s_end - s_start)^2
    delta_s = s_end - s_start
    RHS_eq1 = q_end - q_start - c * delta_s
    // a*(delta_s)^3 + b*(delta_s)^2 = RHS_eq1  (eq 1)

    // s = s_end 일 때 q'(s_end) = 0
    // 0 = 3a*(s_end - s_start)^2 + 2b*(s_end - s_start) + c
    // -c = 3a*(delta_s)^2 + 2b*(delta_s)  (eq 2)

    // eq 1과 eq 2를 연립하여 a와 b에 대해 풀기
    // 행렬 형태로 표현:
    // [ (delta_s)^3, (delta_s)^2 ] [ a ] = [ RHS_eq1 ]
    // [ 3*(delta_s)^2, 2*delta_s ] [ b ] = [ -c      ]

    // Solve the linear system for a and b (e.g., using Cramer's rule or matrix inversion)
    // Determinant = (delta_s)^3 * (2*delta_s) - (delta_s)^2 * (3*(delta_s)^2)
    // Determinant = 2*(delta_s)^4 - 3*(delta_s)^4 = -(delta_s)^4
    // Assume delta_s is not zero

    if abs(delta_s) > epsilon // delta_s가 0이 아닌 경우
        a = (RHS_eq1 * 2*delta_s - (-c) * (delta_s)^2) / (-(delta_s)^4)
        b = ((delta_s)^3 * (-c) - RHS_eq1 * 3*(delta_s)^2) / (-(delta_s)^4)
    else // delta_s가 0에 가까운 경우 (단일 지점)
        // 이 경우 Cubic Spline 정의가 의미 없어짐. 실제 구현에서는 delta_s > 0 보장 필요.
        // 또는 단일 지점 경로는 예외 처리.
        a = 0
        b = 0
        c = tan_Delta_h_start // 또는 0 (도로 중심과 정렬되도록)
        q_end = q_start // 경로가 이동하지 않음
        // 다음 q_end 루프로 건너뛰기
        continue_loop
    end if

    // 5. 경로 후보 정보 저장
    //    - Cubic Spline 계수 (a, b, c)
    //    - 시작/끝 s 값 (s_start, s_end)
    //    - 시작 q 값 (q_start)
    //    - 끝 q 값 (q_end)
    path_candidate = { coefficients: (a, b, c), start_s: s_start, end_s: s_end, start_q: q_start, end_q: q_end }
    Add path_candidate to path_candidates

  // 6. 생성된 경로 후보 리스트 반환
  return path_candidates

// --- 도우미 함수 (이 슈도코드의 범위를 벗어남) ---
// ConvertCartesianToSQ(x, y, center_line_info): Cartesian 좌표를 s-q 좌표로 변환하는 함수
// GenerateQEndValues(current_lane_width, road_type, number_of_candidates): 경로 후보의 끝점 횡방향 오프셋 값들을 생성하는 함수
//   - 예: [-lane_width/2, lane_width/2] 범위에서 number_of_candidates 만큼 균등하게 샘플링
```

#### 2.2.3. Coordinates conversion of path candidate points

's-q' 좌표계로 정의된 경로 후보를 'x-y' 좌표계로 변환하는 과정에 대한 슈도 코드를 제공해 드리겠습니다. 이 변환은 논문의 수식 (6), (7), (8)을 기반으로 하며, 경로를 따라가면서 미분 방정식을 수치적으로 적분하여 'x', 'y', 'h'(진행 방향 각도) 값을 계산하는 방식입니다.

여기서는 중심선 아크 길이 's'를 작은 간격 \(\Delta s\)으로 나누어 순차적으로 계산하는 간단한 오일러 방법을 사용한 슈도 코드를 제시합니다. 실제 구현에서는 더 정확한 수치 적분 방법(예: 룽게-쿠타 방법)이 사용될 수 있습니다.

```pseudocode
함수 경로_후보_sq_에서_xy_로_변환(
  경로_후보_매개변수,         // q(s), dq/ds, d2q/ds2 계산에 필요한 매개변수 (예: 수식 4의 a, b, c 등)
  중심선_함수들,            // 중심선 x0(s), y0(s), j0(s) 정보를 제공하는 함수들
  s_시작,                 // 경로 후보의 시작점 s 값 (차량의 현재 위치에 해당)
  s_끝,                   // 경로 후보의 끝점 s 값
  점의_개수,               // 변환된 경로에 포함될 점의 개수
  초기_xyh                 // 차량의 현재 상태 [초기_x, 초기_y, 초기_h]
):

  // 변환된 경로의 x, y 좌표를 저장할 리스트 선언
  변환된_경로 = 빈 (x, y) 점 리스트

  // 현재 계산 중인 상태 변수 초기화
  현재_s = s_시작
  현재_x = 초기_xyh[0]
  현재_y = 초기_xyh[1]
  현재_h = 초기_xyh[2] // 초기 진행 방향 각도는 차량의 현재 각도

  // s 방향으로의 작은 이동 간격 계산
  delta_s = (s_끝 - s_시작) / (점의_개수 - 1)

  // 시작점 추가
  변환된_경로에 (현재_x, 현재_y) 추가

  // s 방향으로 스텝을 진행하며 경로 계산
  반복 i 를 1 부터 (점의_개수 - 1) 까지:
    // 현재 s 값에서의 경로 및 중심선 속성 계산
    q_s = 매개변수에서_q_계산(경로_후보_매개변수, 현재_s)
    dq_ds = 매개변수에서_dq_ds_계산(경로_후보_매개변수, 현재_s)
    d2q_ds2 = 매개변수에서_d2q_ds2_계산(경로_후보_매개변수, 현재_s)
    j0_s = 중심선_곡률_가져오기(중심선_함수들, 현재_s) // 중심선의 곡률 (j0)

    // 수식 (8)을 사용하여 A 계산
    A = sqrt(dq_ds * dq_ds + (1 - q_s * j0_s) * (1 - q_s * j0_s))
    // 수식 (8)을 사용하여 B 계산
    B = 부호(1 - q_s * j0_s)

    // 수식 (7)을 사용하여 경로의 곡률 kappa 계산
    // 논문에 제시된 형태 그대로 사용:
    kappa = (B / A) * (j0_s + ((1 - q_s * j0_s) * d2q_ds2 + j0_s * dq_ds * dq_ds) / (A * A))

    // 수식 (6)의 미분값을 사용하여 다음 스텝 예측 (오일러 방법)
    dx_ds = A * cos(현재_h)
    dy_ds = A * sin(현재_h)
    dh_ds = A * kappa

    dx = dx_ds * delta_s
    dy = dy_ds * delta_s
    dh = dh_ds * delta_s

    // 상태 업데이트
    현재_x = 현재_x + dx
    현재_y = 현재_y + dy
    현재_h = 현재_h + dh // 각도는 필요에 따라 정규화할 수 있습니다.
    현재_s = 현재_s + delta_s

    // 계산된 점을 경로에 추가
    변환된_경로에 (현재_x, 현재_y) 추가

  // 변환된 경로 반환
  변환된_경로 반환

// --- 도우미 함수 (실제 구현 방식에 따라 달라짐) ---

함수 매개변수에서_q_계산(매개변수, s):
  // 경로 후보 매개변수를 사용하여 s에 해당하는 q 값을 계산합니다.
  // 예: 수식 (4) q(s) = a(s-s_start)^3 + b(s-s_start)^2 + c(s-s_start) + q_start_path 와 같은 함수를 구현
  // 실제 구현 내용 ...
  결과 q_값 반환

함수 매개변수에서_dq_ds_계산(매개변수, s):
  // 경로 후보 매개변수를 사용하여 s에 해당하는 dq/ds 값을 계산합니다.
  // 예: q(s)를 s에 대해 미분한 함수를 구현
  // 실제 구현 내용 ...
  결과 dq_ds_값 반환

함수 매개변수에서_d2q_ds2_계산(매개변수, s):
  // 경로 후보 매개변수를 사용하여 s에 해당하는 d2q/ds2 값을 계산합니다.
  // 예: q(s)를 s에 대해 두 번 미분한 함수를 구현
  // 실제 구현 내용 ...
  결과 d2q_ds2_값 반환

함수 중심선_곡률_가져오기(중심선_함수들, s):
  // 중심선 함수들을 사용하여 s에 해당하는 중심선의 곡률 j0 값을 가져옵니다.
  // 실제 구현 내용 ...
  결과 j0_값 반환

함수 부호(값):
  // 값의 부호를 반환하는 함수
  만약 값 > 0 이면 1 반환
  그렇지 않고 만약 값 < 0 이면 -1 반환
  그렇지 않으면 0 반환

```

**슈도 코드 설명:**

1.  `경로_후보_sq_에서_xy_로_변환` 함수는 s-q 좌표계로 정의된 단일 경로 후보와 중심선 정보를 입력받아 x-y 좌표계의 점들로 구성된 경로를 출력합니다.
2.  입력 매개변수에는 경로 후보를 정의하는 데 사용된 값들, 중심선 정보를 제공하는 함수들, 경로의 시작과 끝 s 값, 그리고 생성할 점의 개수 및 차량의 초기 위치와 진행 방향 각도가 포함됩니다.
3.  함수 내부에서는 `s_시작`부터 `s_끝`까지 `delta_s` 간격으로 s 값을 증가시키면서 반복합니다.
4.  각 s 값에서, 해당 경로 후보의 `q(s)`, `dq/ds`, `d2q/ds2`와 중심선의 `j0(s)` 값을 계산합니다.
5.  계산된 값들을 사용하여 수식 (8)에 따라 `A`와 `B` 값을 구합니다.
6.  수식 (7)에 따라 경로의 곡률 `kappa`를 계산합니다.
7.  수식 (6)에 제시된 `dx/ds`, `dy/ds`, `dh/ds` 값을 현재 상태(`현재_x`, `현재_y`, `현재_h`) 및 계산된 `A`, `kappa` 값을 사용하여 계산합니다.
8.  계산된 미분값에 `delta_s`를 곱하여 x, y, h의 변화량(`dx`, `dy`, `dh`)을 구하고, 이를 현재 상태에 더하여 다음 스텝의 `현재_x`, `현재_y`, `현재_h`를 업데이트합니다.
9.  업데이트된 (x, y) 좌표를 `변환된_경로` 리스트에 추가합니다.
10. 모든 s 구간에 대해 계산이 완료되면, `변환된_경로` 리스트를 반환합니다.
11. `도우미 함수`들은 경로 후보(수식 4)와 중심선(수식 3)의 정의에 따라 `q(s)`, `dq/ds`, `d2q/ds2`, `j0(s)` 등을 실제로 계산하는 부분을 나타냅니다. 이 부분은 논문에 제시된 수식을 바탕으로 구현해야 합니다.

이 슈도 코드는 s-q 좌표계에서 정의된 경로의 형태를 x-y 좌표계 상의 실제 물리적 경로로 매핑하는 기본적인 수치 변환 과정을 보여줍니다.





### 2.3 최적 경로 선택 (Optimal Path Selection)

이 섹션은 자율 주행 시스템이 생성한 여러 경로 후보들 중에서 최적의 경로를 선택하는 과정을 상세하게 설명합니다. 이 과정은 주로 정의된 비용 함수를 최소화하는 경로와 그에 해당하는 가속도를 찾는 방식으로 이루어집니다.

```pseudocode
입력:
    - path_candidates: 경로 후보들의 집합 (각 후보 ri는 경로의 좌표, 곡률, 헤딩 정보 등을 포함)
    - static_obstacles: 정적 장애물 정보 (위치, 크기)
    - road_edges: 도로 경계선 정보
    - lane_lines: 차선 정보 (주행 차선, 반대 차선 등)
    - moving_obstacles: 움직이는 장애물 정보 (현재 위치, 속도, 궤적)
    - previous_path: 이전 계획 단계에서 선택된 경로 (일관성 비용 계산용)
    - vehicle_state: 현재 차량 상태 (위치, 속도 v0, 헤딩 등)
    - parameters: 경로 선택을 위한 설정 값들
        - ws, wc, wd: 정적 안전성, 편안함, 동적 안전성 비용의 가중치
        - sigma_r: 정적 안전성 비용 계산을 위한 가우시안 커널의 표준 편차
        - alpha, beta: 편안함 비용 계산을 위한 부드러움, 일관성 비용의 가중치
        - jaljmax: 최대 횡방향 가속도 한계
        - ksafe: 속도 조절을 위한 안전 이득 (휴리스틱 값)
        - vcurve: 경로의 참조 속도 (실험적 매개변수)
        - vsign: 도로의 최고 속도 제한 표지판 값
        - L0: 목표 추종 거리 (Lf 계산용)
        - Lc: 충돌 범위 (움직이는 장애물과 차량 반경의 합)

출력:
    - optimal_path: 선택된 최적 경로
    - optimal_acceleration: 최적 경로에 해당하는 적절한 가속도
    - optimal_speed: 최적 경로에 해당하는 적절한 속도

변수 초기화:
    num_candidates = path_candidates의 개수
    static_safety_costs = 크기 num_candidates인 배열, 0으로 초기화
    smoothness_costs = 크기 num_candidates인 배열, 0으로 초기화
    consistency_costs = 크기 num_candidates인 배열, 0으로 초기화
    dynamic_safety_costs = 크기 num_candidates인 배열, 0으로 초기화
    calculated_accelerations = 크기 num_candidates인 배열 (각 경로에 대해 계산된 가속도 저장)

단계 1: 경로 후보별 비용 계산
FOR 각 path_candidate ri (인덱스 i = 0 to num_candidates - 1):

    // --- 1.1 정적 안전성 비용 계산 (fs(ri)) ---
    // 이 비용은 도로 경계선, 차선, 정적 장애물과의 충돌 위험을 평가합니다.
    // 가우시안 컨볼루션을 사용하여 인접 경로의 위험도 영향을 고려합니다.

    // 충돌 체크 (R[k]) 계산 - 모든 경로 후보에 대해 미리 수행하거나 여기서 계산
    // R[k]는 경로 후보 rk의 충돌 위험 수준을 나타내는 값 (0, 0.2, 0.5, 1)
    // 0: 충돌 없음, 0.2: 주행 차선 경계선 통과, 0.5: 반대 차선 경계선 통과, 1: 정적 장애물 또는 도로 경계 충돌

    // R 배열은 모든 경로 후보에 대해 사전에 계산되었다고 가정합니다.
    // R = [충돌_체크(r0), 충돌_체크(r1), ..., 충돌_체크(r_num_candidates-1)]

    // 가우시안 커널 G 생성 (표준 컨볼루션 해석 사용)
    // N = (num_candidates - 1) / 2  (또는 컨볼루션 범위)
    // G = 크기 2N+1인 배열
    // FOR k = -N to N:
    //     G[k+N] = exp(-(k^2) / (2 * sigma_r^2)) / sqrt(2 * PI * sigma_r^2) // 가우시안 함수 값
    // END FOR
    // G 정규화 (선택 사항이지만 일반적): sum_G = sum(G), G = G / sum_G

    // 컨볼루션을 통한 fs(ri) 계산 (표준 컨볼루션 해석 사용)
    // fs(ri) = sum(R[k] * G[i-k]) for k where R[k] and G[i-k] are valid indices
    // Paper's formula (10) and (11) interpretation (non-standard convolution):
    // g_i_kernel = 크기 2N+1인 배열 (k=-N to N)
    // FOR k = -N to N:
    //     g_i_kernel[k+N] = exp(-(k-i)^2 / (2 * sigma_r^2)) / sqrt(2 * PI * sigma_r^2) // Note: kernel depends on i
    // END FOR
    // static_safety_costs[i] = sum(g_i_kernel[k+N] * R[k+i+N]) for k = -N to N, ensuring k+i+N is valid R index
    // Out of bounds for R index (k+i+N) means collision check is 1.0 (paper desc)

    // Let's use the standard convolution interpretation which matches the figures better
    N_conv = (num_candidates - 1) // assuming num_candidates is odd for center 0
    gaussian_kernel = 크기 (2 * N_conv + 1)인 배열
    sum_kernel = 0
    FOR k_idx = 0 to 2*N_conv:
        k = k_idx - N_conv // k ranges from -N_conv to N_conv
        gaussian_kernel[k_idx] = exp(-(k^2) / (2 * sigma_r^2))
        sum_kernel += gaussian_kernel[k_idx]
    // Normalize kernel
    FOR k_idx = 0 to 2*N_conv:
         gaussian_kernel[k_idx] /= sum_kernel

    static_safety_cost_i = 0
    FOR k_offset = -N_conv to N_conv:
        R_index = i + k_offset
        collision_val = 0.0
        IF R_index < 0 or R_index >= num_candidates:
            collision_val = 1.0 // Out of bounds is a collision (or high risk)
        ELSE:
            collision_val = R[R_index] // Pre-calculated collision check value
        static_safety_cost_i += gaussian_kernel[k_offset + N_conv] * collision_val // G[k_offset] * R[i + k_offset]
    static_safety_costs[i] = static_safety_cost_i


    // --- 1.2 편안함 비용 계산 (fc(ri)) ---
    // 이 비용은 경로의 부드러움과 이전 경로와의 일관성을 평가합니다.

    // 부드러움 비용 (fsm(ri))
    smoothness_cost_i = 0
    FOR 각 점 p on path_candidate ri:
        curvature_p = p의 곡률 정보
        delta_s_p = 해당 점에서의 미소 경로 길이
        smoothness_cost_i += curvature_p^2 * delta_s_p // 곡률 제곱 적분 근사
    smoothness_costs[i] = smoothness_cost_i

    // 일관성 비용 (fco(ri))
    consistency_cost_i = 0
    overlap_start_s, overlap_end_s = current_path와 previous_path의 겹치는 구간 (s-q 좌표계의 s 기준)
    overlap_length = overlap_end_s - overlap_start_s

    IF overlap_length > 0:
        sum_heading_diff = 0
        FOR 각 점 p on path_candidate ri within overlap [overlap_start_s, overlap_end_s]:
            s_p = 해당 점의 s 값
            delta_s_p = 해당 점에서의 미소 경로 길이
            heading_i_p = p의 헤딩 각도
            heading_pre_p = previous_path 상의 동일 s 값 위치에서의 헤딩 각도 (보간 필요)
            sum_heading_diff += abs(heading_pre_p - heading_i_p) * delta_s_p
        consistency_cost_i = sum_heading_diff / overlap_length
    ELSE:
        consistency_cost_i = 큰 값 (겹치는 구간이 없으면 일관성 낮음)
    consistency_costs[i] = consistency_cost_i

    // fc(ri) 계산
    comfortability_costs[i] = alpha * smoothness_costs[i] + beta * consistency_costs[i]


    // --- 1.3 동적 안전성 비용 계산 (fd(ri, a(ri))) ---
    // 이 비용은 움직이는 장애물 회피를 위해 필요한 가속도/감속도 관련 비용을 평가합니다.

    dynamic_safety_cost_i = 0
    calculated_acceleration_i = 0 // 이 경로에 대해 계산된 가속도

    // 움직이는 장애물과의 잠재적 충돌 지점 탐색
    potential_collision_points = find_collision_points(path_candidate ri, moving_obstacles)

    IF potential_collision_points가 존재:
        // 가장 가까운 잠재적 충돌 지점 Ds(ri) 선택
        Ds_ri = closest_collision_point의 path_candidate ri 시작점에서의 거리
        // Lc와 Lf (L0) 사용
        Lf = L0 // 논문에 따라 Lf는 L0 값 사용 또는 Ds(ri)와 비교 후 사용 (수식 17 해석)

        // 잠재적 충돌 시간 t(ri) 계산 (논문 수식 15)
        // Note: 수식 15 v0 * t(ri) = Ds(ri) - Lc는 물리적으로 직관적이지 않음.
        // 하지만 논문을 따릅니다: t_ri = (Ds_ri - Lc) / v0

        // 필요한 가속도 a(ri) 계산 (논문 수식 16 - 텍스트 버전 사용)
        // a_ri = 2 * (Lc - Lf) * v0^2 / (Ds_ri - Lc)^2
        // Ds_ri - Lc가 0 또는 음수가 되지 않도록 체크 필요 (즉, 이미 충돌했거나 너무 가까우면 유효하지 않은 경로)
        IF Ds_ri - Lc > 0:
             t_ri = (Ds_ri - Lc) / vehicle_state.current_speed // vehicle_state.current_speed는 v0
             // 수식 16의 텍스트 버전: a_ri = 2 * (Lc - Lf) * vehicle_state.current_speed^2 / (Ds_ri - Lc)^2
             // Wait, 다시 확인. 텍스트 버전 (16)은 2(Lc - Lf) / (Ds(ri) - Lc)^2 * v0^2 임. 이게 위와 동일.
             // But, the image formula (16) looks different and seems to imply something like v_final^2 = v0^2 + 2*a*d, maybe?
             // Given the confusion, let's strictly re-implement the text formula 16 derived earlier:
             // a_ri = 2 * (Lc - Lf) * vehicle_state.current_speed^2 / (Ds_ri - Lc)^2

             // 다시 논문 텍스트 수식 16을 살펴보면 Ds(ri)와 Lc 부분이 분모에 제곱으로 있고 v0^2이 곱해져 있음.
             // a(ri) = 2(Lc - Lf) / (Ds(ri) - Lc)^2 v0^2
             // 이것은 a(ri) = (2(Lc - Lf) / (Ds(ri) - Lc)^2) * v0^2 로 해석하는 것이 자연스러움.

             // a_ri = 2 * (Lc - Lf) / ((Ds_ri - Lc)^2) * vehicle_state.current_speed^2 // Use TEXT formula 16

             // 계산된 가속도 저장
             calculated_acceleration_i = a_ri

             // 동적 안전성 비용 계산 (논문 수식 22)
             // f_d(ri, a(ri)) = abs(a(ri)) * (Ds(ri) - Lf)
             dynamic_safety_cost_i = abs(calculated_acceleration_i) * (Ds_ri - Lf)
        ELSE:
             // Ds_ri - Lc <= 0 인 경우, 즉 충돌 지점이 현재 위치에 너무 가깝거나 뒤에 있는 경우
             // 이 경로는 피해야 하므로 매우 높은 비용 부여
             dynamic_safety_cost_i = 무한대 또는 매우 큰 값
             calculated_acceleration_i = 0 // 또는 적절한 기본값

    ELSE: // 잠재적 충돌 지점이 없는 경우
        dynamic_safety_cost_i = 0 // 움직이는 장애물 회피 관련 비용은 없음
        calculated_acceleration_i = 0 // 또는 순항 가속도 등 적절한 기본값

    dynamic_safety_costs[i] = dynamic_safety_cost_i
    calculated_accelerations[i] = calculated_acceleration_i // 계산된 가속도 저장 (제한 적용 전)

END FOR // 경로 후보별 비용 계산 종료


단계 2: 비용 정규화 및 총 비용 계산
// 논문에 따라 ws, wc, wd 가중치를 적용하기 전에 각 비용을 0-1 사이로 정규화합니다.

max_fs = max(static_safety_costs)
max_fc = max(comfortability_costs)
max_fd = max(dynamic_safety_costs)

normalized_total_costs = 크기 num_candidates인 배열

FOR i = 0 to num_candidates - 1:
    normalized_fs = static_safety_costs[i]
    IF max_fs > 0: normalized_fs /= max_fs

    normalized_fc = comfortability_costs[i]
    IF max_fc > 0: normalized_fc /= max_fc

    normalized_fd = dynamic_safety_costs[i]
    IF max_fd > 0: normalized_fd /= max_fd

    // 총 비용 계산 (논문 수식 23)
    // f(ri, a(ri)) = ws * fs(ri) + wc * fc(ri) + wd * fd(ri, a(ri))
    normalized_total_costs[i] = ws * normalized_fs + wc * normalized_fc + wd * normalized_fd

END FOR


단계 3: 최적 경로 선택
// 총 비용이 가장 낮은 경로를 선택합니다.

min_cost = 무한대
optimal_path_index = -1

FOR i = 0 to num_candidates - 1:
    IF normalized_total_costs[i] < min_cost:
        min_cost = normalized_total_costs[i]
        optimal_path_index = i
END FOR

optimal_path = path_candidates[optimal_path_index]
optimal_acceleration_calculated = calculated_accelerations[optimal_path_index]


단계 4: 선택된 경로에 대한 가속도 및 속도 결정 및 제한 적용
// 선택된 최적 경로에 대해 계산된 가속도를 사용하되, 속도 제한을 고려합니다.

// 속도 제한 계산 (논문 수식 18)
// vlimit(ri) = min[vj(ri), vr(ri), vsign]
// vj(ri) = sqrt(abs(jaljmax / max_curvature(optimal_path)))
// vr(ri) = (1 - ksafe * static_safety_costs[optimal_path_index]^2) * vcurve

max_curvature_optimal_path = calculate_max_curvature(optimal_path)
vj_optimal = 무한대
IF max_curvature_optimal_path != 0:
   vj_optimal = sqrt(abs(jaljmax / max_curvature_optimal_path))

vr_optimal = (1 - ksafe * static_safety_costs[optimal_path_index]^2) * vcurve

vlimit_optimal_path = min(vj_optimal, vr_optimal, vsign)

// 가속도 제한 적용 (논문 수식 21)
// a(ri) <= (vlimit(ri)^2 - v0^2) / (2 * (Ds(ri) - Lf))
// Note: 이 제한은 움직이는 장애물 회피 가속도 계산 시 사용되는 Ds와 Lf를 다시 필요로 함.
// 최적 경로에 대해 계산된 Ds와 Lf 값을 사용해야 합니다.
// (이전 단계에서 계산된 calculated_accelerations에 사용된 Ds와 Lf 필요)
// 편의상 최적 경로에 대한 Ds와 Lf를 다시 찾거나 저장된 값을 사용합니다.
Ds_optimal_path = find_distance_to_closest_collision_point(optimal_path, moving_obstacles)
Lf_optimal_path = L0 // 또는 수식 17에 따라 Ds_optimal_path와 비교

accel_limit = (vlimit_optimal_path^2 - vehicle_state.current_speed^2) / (2 * (Ds_optimal_path - Lf_optimal_path)) // 분모 0 체크 필요

// 최종 가속도는 계산된 가속도와 가속도 제한 중 작은 값입니다.
// Note: 논문은 단순히 '제한된다'고만 언급하며, 계산된 a(ri)가 이 제한을 만족해야 함을 시사합니다.
// 여기서는 계산된 가속도가 이 제한을 초과하지 않는다고 가정하거나, 초과 시 제한 값으로 클리핑합니다.
// 클리핑 방식을 사용합니다.
optimal_acceleration = min(optimal_acceleration_calculated, accel_limit) // 양의 가속도 제한
optimal_acceleration = max(optimal_acceleration, -abs(accel_limit)) // 음의 가속도 제한 (감속도 제한)
// Note: 논문은 양의 가속도 제한 수식만 제시했으나, 실제 구현에서는 감속도 제한도 필요합니다.
// 논문 2.3.1 마지막 문단 "Appropriate acceleration... can be simultaneously provided"를 보면
// 계산된 a(ri) 자체가 이미 제한을 고려한 결과일 수도 있습니다.
// 하지만 수식 21은 'limit'로 표현되어 있으므로, 여기서 최종 클리핑을 적용하는 것이 안전합니다.

// 최적 속도 계산 (계산된 가속도 및 계획 스텝 시간/거리 기반)
// 예: 다음 스텝 시작 시 예상 속도 (단순화)
// optimal_speed = vehicle_state.current_speed + optimal_acceleration * planning_step_time
// 실제 구현에서는 경로를 따라가면서 속도가 동적으로 업데이트됩니다.
// 논문에서는 vlimit(ri)를 계산하여 최적 속도로 제시할 수도 있습니다.
// "Appropriate acceleration and speed for every planning can be simultaneously provided"
// 여기서는 계산된 가속도를 사용하되, 평균 속도나 다음 스텝 시작 속도 등을 제시할 수 있습니다.
// vlimit_optimal_path를 이 경로에서 목표로 할 수 있는 최고 속도로 간주할 수 있습니다.
// 하지만 가속도가 주어졌으므로, 현재 속도 v0와 optimal_acceleration을 사용하여
// 경로를 따라 이동했을 때의 속도 프로파일을 계산하는 것이 더 정확합니다.
// 간단하게, 논문에서 제시된 vlimit_optimal_path를 "appropriate speed"로 이해하고 제시할 수 있습니다.
optimal_speed = vlimit_optimal_path // 최적 경로에서 목표할 수 있는 최대 안전 속도

// (선택 사항) 선택된 optimal_acceleration에 대해 시간 가우시안 필터링 적용 (논문 2.3.3 마지막 문단 언급)
// 이것은 현재 단계의 출력이 아니라, 연속적인 계획 단계에서 가속도 값의 부드러움을 위해 사용됩니다.
// 즉, 이 함수의 출력을 저장해두고 다음 실행 시 이전 값들과 함께 필터링합니다.

```

#### 2.3.1. Cost function for static safety

본 논문에서 제안하는 동적 경로 계획 방법의 상세한 슈도코드는 다음과 같습니다. 이 방법은 크게 세 단계(중심선 구성, 경로 후보 생성, 경로 선택)로 이루어집니다.

```pseudocode
Function DynamicPathPlanning(waypoints, static_obstacles, moving_obstacles, road_edges, lane_lines, current_vehicle_state, previous_path):
    // 입력:
    //   waypoints: 차선 중앙을 나타내는 미리 정의된 웨이포인트 집합
    //   static_obstacles: 정적 장애물 정보 (위치, 크기)
    //   moving_obstacles: 움직이는 장애물 정보 (위치, 크기, 속도, 방향)
    //   road_edges: 도로 경계선 정보
    //   lane_lines: 차선 경계선 정보
    //   current_vehicle_state: 현재 차량 상태 (위치, 속도, 방향)
    //   previous_path: 이전 계획 단계에서 선택된 경로 (경로 일관성 계산용)

    // 출력:
    //   optimal_path: 선택된 최적 경로 (카테시안 좌표)
    //   appropriate_acceleration: 최적 경로에 대한 적정 가속도
    //   appropriate_speed: 최적 경로에 대한 적정 속도

    // 단계 1: 중심선 구성 (Center Line Construction)
    center_line = ConstructCenterLine(waypoints)
    //   - 웨이포인트를 사용하여 매개변수화된 3차 스플라인 곡선으로 중심선 구성
    //   - 곡선의 매개변수를 호 길이(arc length) s로 설정

    // 단계 2: 경로 후보 생성 (Path Candidates Generation)
    path_candidates = GeneratePathCandidates(center_line, current_vehicle_state)
    //   - 현재 차량 위치를 중심선 상의 s-q 좌표로 변환 (Localization)
    //     - s_start: 중심선 상의 호 길이 시작점
    //     - q_start: 중심선으로부터의 횡방향 오프셋 시작점
    //     - Dh_start: 차량 방향과 중심선 접선 방향의 상대 각도
    //   - 경로 끝점의 횡방향 오프셋 q_end 값 집합 정의 (미리 설정된 개수, 예: 21개)
    //   - 각 q_end 값에 대해:
    //     - 시작 및 경계 조건 (s_start, q_start, Dh_start, s_end, q_end, dq/ds_end=0)을 사용하여
    //       호 길이 s에 대한 횡방향 오프셋 q(s)를 정의하는 3차 다항식의 계수 (a, b, c) 계산
    //     - 이 q(s) 함수는 s-q 좌표계에서의 경로 후보를 정의함
    //     - 경로 후보의 각 점을 s-q 좌표에서 카테시안 좌표 (x, y)로 변환 (식 (6)-(8) 사용)
    //   - 생성된 모든 경로 후보 집합을 반환

    // 단계 3: 경로 선택 (Path Selection)
    min_total_cost = infinity
    optimal_path = null
    appropriate_acceleration = 0
    appropriate_speed = 0

    For each path_candidate in path_candidates:
        // 3.1. 정적 안전성 비용 계산 (Cost function for static safety)
        collision_check_values = PerformCollisionCheck(path_candidate, static_obstacles, road_edges, lane_lines)
        //   - 경로 후보와 정적 장애물, 도로 경계선, 차선 경계선 간의 충돌 여부 확인
        //   - 단일 차선/다차선 도로 규칙에 따라 충돌 확인 값 (0, 0.2, 0.5, 1) 할당

        static_safety_cost = CalculateStaticSafetyCost(collision_check_values)
        //   - 이산 가우시안 컨볼루션 적용 (식 (10), (11) 사용)
        //   - 경로 근접 위험을 고려하여 정적 안전성 비용 계산

        // 3.2. 편안함 비용 계산 (Cost function for comfortability)
        smoothness_cost = CalculateSmoothnessCost(path_candidate)
        //   - 경로의 곡률 제곱 적분 (식 (12) 사용)
        //   - 경로의 부드러움 평가

        consistency_cost = CalculateConsistencyCost(path_candidate, previous_path)
        //   - 현재 경로 후보와 이전 경로 간의 중첩 구간에서의 방향 차이 적분 (식 (13) 사용)
        //   - 경로 일관성 평가

        comfortability_cost = CalculateComfortabilityCost(smoothness_cost, consistency_cost)
        //   - smoothness_cost와 consistency_cost의 가중합 (식 (14) 사용)

        // 3.3. 동적 안전성 비용 계산 (Cost function for dynamic safety)
        is_available_path = CheckAvailability(path_candidate, static_obstacles, road_edges)
        //   - 정적 장애물이나 도로 경계를 침범하지 않는 사용 가능한 경로인지 확인

        If is_available_path:
            potential_collision_points = FindCollisionPoints(path_candidate, moving_obstacles)
            //   - 움직이는 장애물과의 잠재적 충돌 지점 탐색 (사용 가능한 경로에 대해서만)

            If potential_collision_points exist:
                potential_collision_time = CalculateCollisionTime(path_candidate, current_vehicle_state.speed, potential_collision_points)
                //   - 현재 속도로 잠재적 충돌 지점까지 도달하는 시간 계산 (식 (15) 사용)

                required_acceleration = CalculateRequiredAcceleration(path_candidate, potential_collision_time, current_vehicle_state.speed)
                //   - 충돌 회피를 위해 필요한 가속도 계산 (후행 방식, 식 (16), (17) 사용)

                speed_limit = CalculateSpeedLimit(path_candidate, static_safety_cost)
                //   - 경로 곡률, 안전 위험, 속도 제한 표지판 등을 고려한 속도 제한 계산 (식 (18)-(20) 사용)

                limited_acceleration = LimitAcceleration(required_acceleration, speed_limit, current_vehicle_state.speed)
                //   - 속도 제한에 따른 가속도 제한 적용 (식 (21) 사용)

                dynamic_safety_cost = CalculateDynamicSafetyCost(limited_acceleration, path_candidate)
                //   - 가속도 크기와 이동 거리를 고려한 동적 안전성 비용 계산 (식 (22) 사용)

                // 가속도 값 스무딩 (논문 언급, Gaussian filtering 사용)
                // smoothed_acceleration = ApplyGaussianSmoothing(limited_acceleration, past_accelerations)
                // dynamic_safety_cost 계산 시 smoothed_acceleration 사용 가능 (슈도코드에서는 limited_acceleration 사용)
            Else:
                // 움직이는 장애물과의 잠재적 충돌 없음
                dynamic_safety_cost = 0 // 또는 최소 비용 값
                limited_acceleration = AppropriateAccelerationForNormalDriving(path_candidate, current_vehicle_state.speed)
                // 정적/편안함 비용만 고려하여 적정 가속도 결정 또는 기본값 설정
        Else:
            // 사용 불가능한 경로 (정적 위험)
            dynamic_safety_cost = infinity // 또는 매우 높은 값
            limited_acceleration = 0 // 또는 의미 없는 값

        // 3.4. 총 비용 계산 및 최적 경로 선택 (Total cost function for path selection)
        normalized_static_safety_cost = Normalize(static_safety_cost) // 0~1 범위로 정규화
        normalized_comfortability_cost = Normalize(comfortability_cost) // 0~1 범위로 정규화
        normalized_dynamic_safety_cost = Normalize(dynamic_safety_cost) // 0~1 범위로 정규화

        ws, wc, wd = GetCostWeights() // 비용 함수별 가중치 (미리 설정, 예: 0.5, 0.2, 0.3)

        total_cost = ws * normalized_static_safety_cost + wc * normalized_comfortability_cost + wd * normalized_dynamic_safety_cost
        // 총 비용 계산 (식 (23) 사용)

        // 최적 경로 업데이트
        If total_cost < min_total_cost:
            min_total_cost = total_cost
            optimal_path = path_candidate
            appropriate_acceleration = limited_acceleration // 해당 경로에 대한 적정 가속도
            appropriate_speed = CalculateSpeedAtEndOfPath(path_candidate, current_vehicle_state.speed, limited_acceleration) // 경로 끝점 예상 속도

    // 선택된 최적 경로와 적정 가속도/속도 반환
    Return optimal_path, appropriate_acceleration, appropriate_speed

```

#### 2.3.2. Cost function for comfortability

이 슈도코드는 논문 Dynamic path planning for autonomous driving on various roads with avoidance of static and moving obstacles에 제시된 동적 경로 계획 방법의 핵심 단계를 기반으로 작성되었습니다. 이 알고리즘은 실시간으로 반복 실행됩니다 (예: 초당 15회).

```pseudocode
// 전역 변수 및 상수 정의
GLOBAL_WAYPOINTS: 디지털 지도에서 얻은 차선 레벨의 전역 웨이포인트 집합
STATIC_OBSTACLES: 센서 또는 지도에서 감지된 정적 장애물 목록
MOVING_OBSTACLES: 센서에서 감지된 동적 장애물 목록 (위치, 속도, 방향 포함)
ROAD_EDGES: 도로 경계선 정보
LANE_LINES: 차선 구분선 정보 (실선, 점선 등)
PREVIOUS_PATH: 이전 계획 단계에서 선택된 경로 (연속성 평가에 사용)

PLANNING_HORIZON_S: 경로 계획의 전방 거리 (중심선 호 길이 기준)
NUM_PATH_CANDIDATES: 생성할 경로 후보의 개수
WEIGHT_STATIC_SAFETY (ws): 정적 안전 비용의 가중치 (예: 0.5)
WEIGHT_COMFORTABILITY (wc): 쾌적성 비용의 가중치 (예: 0.2)
WEIGHT_DYNAMIC_SAFETY (wd): 동적 안전 비용의 가중치 (예: 0.3)
SMOOTHNESS_WEIGHT_A (a): 부드러움 비용 가중치
CONSISTENCY_WEIGHT_B (b): 연속성 비용 가중치
COLLISION_RISK_STD_DEV (r): 정적 안전성 계산을 위한 가우시안 분포 표준 편차
LATERAL_ACCELERATION_LIMIT: 허용 최대 횡방향 가속도
SPEED_LIMIT_SIGN: 도로 제한 속도

// 메인 경로 계획 루프 (예: 초당 15회 실행)
LOOP:
    // 1. 현재 차량 상태 감지
    CURRENT_VEHICLE_STATE: { position (x, y), speed (v), heading (h), acceleration (a) }
    SENSE_ENVIRONMENT() // 센서를 통해 장애물 및 도로 정보 업데이트

    // 2. 중심선 구축 (Section 2.1)
    CENTER_WAYPOINTS = GET_CENTER_WAYPOINTS(GLOBAL_WAYPOINTS)
    CENTER_LINE_SPLINE = FIT_CUBIC_SPLINE_ARC_LENGTH(CENTER_WAYPOINTS) // 호 길이로 매개변수화된 스플라인

    // 3. 경로 후보군 생성 (Section 2.2)
    PATH_CANDIDATES = GENERATE_PATH_CANDIDATES(CENTER_LINE_SPLINE, CURRENT_VEHICLE_STATE, PLANNING_HORIZON_S, NUM_PATH_CANDIDATES)

    // 4. 경로 선택 (Section 2.3)
    BEST_PATH = NULL
    MIN_TOTAL_COST = INFINITY
    BEST_ACCELERATION = 0
    BEST_SPEED = CURRENT_VEHICLE_STATE.speed

    // 4.1 정적 안전 비용 계산을 위한 충돌 확인
    COLLISION_CHECKS = [] // 각 경로 후보에 대한 충돌 확인 결과 리스트
    FOR EACH path_candidate IN PATH_CANDIDATES:
        COLLISION_CHECK_VALUE = CHECK_STATIC_COLLISION(path_candidate, ROAD_EDGES, LANE_LINES, STATIC_OBSTACLES) // 0, 0.2, 0.5, 1 중 하나
        ADD COLLISION_CHECK_VALUE TO COLLISION_CHECKS

    // 4.2 충돌 확인 결과를 이용한 정적 안전 위험 계산 (Section 2.3.1)
    STATIC_SAFETY_COSTS = CALCULATE_STATIC_SAFETY_COSTS(COLLISION_CHECKS, COLLISION_RISK_STD_DEV) // 가우시안 컨볼루션 적용 (Eq 10, 11)

    // 4.3 각 경로 후보에 대한 비용 계산 및 평가
    FOR EACH path_candidate ri IN PATH_CANDIDATES (index i):
        // 4.3.1 쾌적성 비용 계산 (Section 2.3.2)
        SMOOTHNESS_COST = CALCULATE_SMOOTHNESS_COST(ri) // 곡률 제곱 적분 (Eq 12)
        CONSISTENCY_COST = CALCULATE_CONSISTENCY_COST(ri, PREVIOUS_PATH) // 이전 경로와의 방향 차이 적분 (Eq 13)
        COMFORTABILITY_COST = SMOOTHNESS_WEIGHT_A * SMOOTHNESS_COST + CONSISTENCY_WEIGHT_B * CONSISTENCY_COST // (Eq 14)

        // 4.3.2 동적 안전 비용 계산 (Section 2.3.3)
        // 이동 장애물 회피를 위한 필요 가속도 계산 (개념적, 논문의 Eq 16 유도는 불분명함)
        REQUIRED_ACCEL_FOR_DYNAMIC_SAFETY = CALCULATE_REQUIRED_ACCELERATION_FOR_COLLISION_AVOIDANCE(ri, MOVING_OBSTACLES, CURRENT_VEHICLE_STATE)
        DYNAMIC_SAFETY_COST = ABS(REQUIRED_ACCEL_FOR_DYNAMIC_SAFETY) * EFFECTIVE_TRAVEL_DISTANCE(ri, MOVING_OBSTACLES) // (Eq 22)

        // 4.3.3 속도 제한 계산 (Section 2.3.3)
        SPEED_LIMIT_CURVATURE = CALCULATE_SPEED_LIMIT_BASED_ON_CURVATURE(ri, LATERAL_ACCELERATION_LIMIT) // (Eq 19)
        SPEED_LIMIT_RISK = CALCULATE_SPEED_LIMIT_BASED_ON_RISK(STATIC_SAFETY_COSTS[i], REFERENCE_SPEED_CURVE) // (Eq 20)
        // 참고: 논문에는 risk 기반 속도 제한 계산 시 static safety cost를 사용하는 것으로 보임 (Eq 20)
        CURRENT_PATH_SPEED_LIMIT = MIN(SPEED_LIMIT_SIGN, SPEED_LIMIT_CURVATURE, SPEED_LIMIT_RISK) // (Eq 18)

        // 계획된 가속도가 속도 제한을 초과하지 않도록 조정 (Section 2.3.3, Eq 21)
        // 논문의 Eq 21은 Lf와 Lc를 포함한 복잡한 형태이나, 개념적으로는 가속도를 적용했을 때
        // 계획 시간 내에 도달하는 속도가 제한 속도를 넘지 않도록 제약하는 과정임.
        // 여기서는 필요 가속도를 계산하고, 해당 가속도로 도달하는 속도가 제한 속도를 넘으면
        // 가속도를 낮추는 것으로 표현함.
        POTENTIAL_FINAL_SPEED = CURRENT_VEHICLE_STATE.speed + REQUIRED_ACCEL_FOR_DYNAMIC_SAFETY * PLANNING_STEP_TIME
        IF POTENTIAL_FINAL_SPEED > CURRENT_PATH_SPEED_LIMIT:
             // 필요 가속도를 조정하거나, 실제 사용될 가속도를 제한 속도를 만족하도록 계산
             // 실제 시스템에서는 PID 제어기 등으로 추종
             ADJUSTED_ACCELERATION = (CURRENT_PATH_SPEED_LIMIT^2 - CURRENT_VEHICLE_STATE.speed^2) / (2 * EFFECTIVE_TRAVEL_DISTANCE(ri, MOVING_OBSTACLES)) // 근사치 계산
        ELSE:
             ADJUSTED_ACCELERATION = REQUIRED_ACCEL_FOR_DYNAMIC_SAFETY

        // 가속도 평활화 (Section 2.3.3, 가우시안 필터 언급)
        // ADJUSTED_ACCELERATION = SMOOTH_ACCELERATION(ADJUSTED_ACCELERATION, PAST_ACCELERATIONS)

        // 4.3.4 총 비용 계산 (Section 2.3.4)
        // 정적, 쾌적성, 동적 비용을 [0, 1] 범위로 정규화 (구현 세부 사항은 논문에 명시되지 않음)
        NORMALIZED_STATIC_SAFETY_COST = NORMALIZE(STATIC_SAFETY_COSTS[i])
        NORMALIZED_COMFORTABILITY_COST = NORMALIZE(COMFORTABILITY_COST)
        NORMALIZED_DYNAMIC_SAFETY_COST = NORMALIZE(DYNAMIC_SAFETY_COST) // 조정 전 필요 가속도로 계산하는 것이 논리상 맞을 수 있음

        TOTAL_COST = WEIGHT_STATIC_SAFETY * NORMALIZED_STATIC_SAFETY_COST + ...
                     WEIGHT_COMFORTABILITY * NORMALIZED_COMFORTABILITY_COST + ...
                     WEIGHT_DYNAMIC_SAFETY * NORMALIZED_DYNAMIC_SAFETY_COST // (Eq 23)

        // 4.4 최적 경로 선택
        IF TOTAL_COST < MIN_TOTAL_COST:
            MIN_TOTAL_COST = TOTAL_COST
            BEST_PATH = ri
            // 선택된 경로에 대한 최종 목표 가속도 및 속도 결정
            // 이는 위에서 계산된 ADJUSTED_ACCELERATION 또는 REQUIRED_ACCEL_FOR_DYNAMIC_SAFETY를 기반으로 하며,
            // 실제 차량 제어에 사용될 값임. 논문은 "appropriate acceleration and speed"를 제공한다고 함.
            // 여기서는 조정된 가속도를 목표 가속도로 사용한다고 가정.
            BEST_ACCELERATION = ADJUSTED_ACCELERATION
            BEST_SPEED = CURRENT_PATH_SPEED_LIMIT // 또는 v0 + BEST_ACCELERATION * dt 를 제한 속도로 캡핑

    // 5. 결과 출력 (차량 제어 시스템으로 전달)
    OUTPUT SELECTED_PATH = BEST_PATH
    OUTPUT TARGET_ACCELERATION = BEST_ACCELERATION
    OUTPUT TARGET_SPEED = BEST_SPEED

    PREVIOUS_PATH = BEST_PATH // 다음 계획 단계를 위해 현재 선택된 경로 저장

END LOOP

// 하위 함수/프로시저 (상세 구현은 논문 및 외부 라이브러리 참고)

FUNCTION GET_CENTER_WAYPOINTS(global_waypoints):
    // 각 도로 구간의 양쪽 가장자리 웨이포인트를 이용하여 중심점 계산
    // 논문 Fig. 1 참조
    RETURN list_of_center_waypoints

FUNCTION FIT_CUBIC_SPLINE_ARC_LENGTH(waypoints):
    // 주어진 웨이포인트에 호 길이로 매개변수화된 3차 스플라인 피팅
    // 논문 Section 2.1 참조 (Eq 1)
    // 수치 적분을 통해 호 길이 계산 및 스플라인 매개변수 재설정
    RETURN cubic_spline_representation

FUNCTION GENERATE_PATH_CANDIDATES(center_line_spline, vehicle_state, horizon_s, num_candidates):
    // 현재 차량 위치를 중심선에 매핑 (Section 2.2.1)
    (s_start, q_start, Dh_start) = MAP_VEHICLE_TO_SQ_COORDINATES(center_line_spline, vehicle_state.position)

    s_end = s_start + horizon_s
    q_end_values = GENERATE_DISCRETE_LATERAL_OFFSETS(num_candidates, road_width) // 도로 폭 등을 고려하여 q_end 후보 값들 생성

    path_candidates_list = []
    FOR EACH q_end IN q_end_values:
        // s-q 좌표계에서 3차 다항식 q(s) 계수 계산 (Section 2.2.2, Eq 4, 5)
        // 경계 조건: q(s_start)=q_start, q(s_end)=q_end, q'(s_start)=tan(Dh_start), q'(s_end)=0
        (a, b, c) = SOLVE_POLYNOMIAL_COEFFICIENTS(s_start, q_start, Dh_start, s_end, q_end)

        // s-q 경로 후보를 카르테시안 좌표로 변환 (Section 2.2.3, Eq 6, 7, 8)
        cartesian_path_points = CONVERT_SQ_TO_CARTESIAN(center_line_spline, a, b, c, s_start, s_end)
        ADD cartesian_path_points TO path_candidates_list

    RETURN path_candidates_list

FUNCTION CHECK_STATIC_COLLISION(path, road_edges, lane_lines, static_obstacles):
    // 경로가 도로 경계, 차선, 정적 장애물과 충돌하는지 확인 (Section 2.3.1)
    // 충돌 유형에 따라 다른 값 반환 (예: 도로 경계/정적 장애물 = 1, 반대 차선 = 0.5, 주행 차선 구분선 = 0.2, 충돌 없음 = 0)
    // 차량 및 장애물을 원으로 근사하여 충돌 확인 수행
    RETURN collision_value

FUNCTION CALCULATE_STATIC_SAFETY_COSTS(collision_checks, std_dev):
    // 충돌 확인 결과 리스트에 이산 가우시안 컨볼루션 적용 (Section 2.3.1, Eq 10, 11)
    // 주변 경로의 충돌 위험을 고려하여 각 경로의 정적 안전 비용 계산
    RETURN list_of_static_safety_costs

FUNCTION CALCULATE_SMOOTHNESS_COST(path):
    // 경로의 곡률을 따라 곡률 제곱을 적분하여 부드러움 비용 계산 (Section 2.3.2, Eq 12)
    RETURN smoothness_cost

FUNCTION CALCULATE_CONSISTENCY_COST(current_path, previous_path):
    // 현재 경로와 이전 경로가 겹치는 구간에서 방향 각도 차이 적분 (Section 2.3.2, Eq 13)
    // 겹치는 구간 길이로 나누어 정규화
    // 이전 경로가 없으면 (첫 단계) 0 반환
    IF previous_path IS NULL:
        RETURN 0
    ELSE:
        RETURN consistency_cost

FUNCTION CALCULATE_REQUIRED_ACCELERATION_FOR_COLLISION_AVOIDANCE(path, moving_obstacles, vehicle_state):
    // 경로 상에서 이동 장애물과의 잠재적 충돌 지점 및 시간 계산 (Section 2.3.3)
    // 충돌을 피하기 위해 필요한 차량 가속도/감속도 계산 (논문의 Eq 15, 16 관련 개념)
    // 논문의 Eq 16 유도가 불분명하므로, 여기서는 개념적으로 표현
    // 예: 충돌 회피에 필요한 거리와 시간, 현재 속도를 기반으로 필요한 가속도 추정
    RETURN required_acceleration

FUNCTION EFFECTIVE_TRAVEL_DISTANCE(path, moving_obstacles):
    // 동적 안전 비용 계산 시 사용되는 유효 주행 거리 (Section 2.3.3, Ds(ri) - Lf)
    // 충돌 지점까지의 거리에서 이동 장애물을 따라가야 하는 거리 Lf를 뺀 값
    RETURN effective_distance

FUNCTION CALCULATE_SPEED_LIMIT_BASED_ON_CURVATURE(path, lateral_accel_limit):
    // 경로의 최대 곡률을 기반으로 안전 주행 속도 제한 계산 (Section 2.3.3, Eq 19)
    RETURN speed_limit

FUNCTION CALCULATE_SPEED_LIMIT_BASED_ON_RISK(static_safety_cost, reference_speed):
    // 정적 안전 비용을 기반으로 속도 제한 조정 (Section 2.3.3, Eq 20)
    // 안전 위험이 높을수록 속도 제한 감소
    RETURN speed_limit

FUNCTION NORMALIZE(value):
    // 주어진 값을 0과 1 사이로 정규화 (논문에 구체적인 방법은 명시되지 않음, 예: 최대/최소 값 사용)
    RETURN normalized_value

FUNCTION SMOOTH_ACCELERATION(current_acceleration, past_accelerations):
    // 과거 가속도 값들을 사용하여 현재 가속도 값을 평활화 (Section 2.3.3, 가우시안 필터 언급)
    RETURN smoothed_acceleration
```

**슈도코드 설명:**

1.  **메인 루프:** 자율 주행 시스템의 주기적인 경로 계획 과정을 나타냅니다. 매 주기마다 최신 차량 상태와 환경 정보를 받아 경로 계획을 수행합니다.
2.  **중심선 구축:** 디지털 지도에서 얻은 웨이포인트를 기반으로 차량이 따라가야 할 도로의 중심선을 계산합니다. 이 중심선은 호 길이(arc length)로 매개변수화되어 경로 생성을 용이하게 합니다.
3.  **경로 후보군 생성:** 현재 차량 위치를 중심선에 매핑한 후, s-q 좌표계(중심선 방향 s, 중심선에서의 횡방향 오프셋 q)를 사용하여 다양한 횡방향 오프셋을 갖는 여러 개의 경로 후보군을 생성합니다. 이 후보군들은 3차 다항식으로 표현되며, 최종적으로 차량 제어를 위해 카르테시안 좌표로 변환됩니다.
4.  **경로 선택:** 생성된 경로 후보군들 중에서 가장 적합한 경로를 선택하는 과정입니다. 이를 위해 다음 세 가지 비용 함수를 계산하고 가중치 합을 통해 총 비용을 산출합니다.
    *   **정적 안전 비용:** 도로 경계, 차선, 정지된 장애물과의 충돌 위험을 평가합니다. 가우시안 컨볼루션을 사용하여 인접한 경로의 위험도까지 고려합니다.
    *   **쾌적성 비용:** 경로의 부드러움(곡률)과 이전 단계 경로와의 연속성을 평가하여 탑승자의 편안함을 고려합니다.
    *   **동적 안전 비용:** 이동하는 장애물과의 충돌을 피하기 위해 필요한 가속도/감속도의 크기를 평가합니다. 이는 속도 조절을 통한 동적 회피 전략을 반영합니다.
    *   각 비용은 정규화된 후 미리 정의된 가중치와 곱해져 총 비용에 합산됩니다.
    *   총 비용이 가장 낮은 경로가 최종 최적 경로로 선택됩니다.
5.  **결과 출력:** 선택된 최적 경로와 해당 경로를 따라가기 위한 목표 가속도, 목표 속도를 차량 제어 시스템으로 전달합니다. 이 정보는 차량이 실제로 조향, 가감속을 수행하는 데 사용됩니다.
6.  **이전 경로 저장:** 다음 계획 단계에서 연속성 비용을 계산하기 위해 현재 선택된 경로를 저장해 둡니다.

이 슈도코드는 논문의 핵심적인 아이디어와 흐름을 따르지만, 각 세부 함수의 구체적인 수학적 계산이나 구현 방법(예: 스플라인 피팅, 좌표 변환의 미분 계산, 가우시안 컨볼루션, 비선형 방정식 해법 등)은 논문의 해당 섹션 또는 관련 참고 문헌([39-44] 등)을 통해 더 자세히 파악해야 합니다. 특히 동적 안전 비용 계산 부분에서 필요한 가속도 산출 방식(논문의 Eq 16)은 논문 설명만으로는 완전하게 이해하기 어려워 개념적인 설명으로 대체했습니다.

#### 2.3.3. Cost function for dynamic safety

논문의 내용을 기반으로 자율 주행을 위한 동적 경로 계획 방법의 상세한 슈도코드(Pseudocode)를 제공해 드리겠습니다. 이 슈도코드는 논문의 2장 "Material and methods"에 설명된 세 단계를 따릅니다.

```pseudocode
FUNCTION DynamicPathPlanning(CurrentVehicleState, MapData, ObstacleData, PreviousSelectedPath)
  // 입력:
  //   CurrentVehicleState: 현재 차량의 위치, 속도, 방향 정보
  //   MapData: 차선 레벨 지도에서 얻은 미리 정의된 웨이포인트 집합
  //   ObstacleData: 센서에서 감지된 정적 및 동적 장애물 정보 (위치, 크기, 속도)
  //   PreviousSelectedPath: 이전 계획 단계에서 선택된 경로 (일관성 유지를 위해 필요)

  // 출력:
  //   OptimalPath: 다음 계획 주기 동안 따라야 할 최적의 경로 (점들의 시퀀스)
  //   OptimalAcceleration: OptimalPath에 해당하는 적절한 가속도
  //   OptimalSpeed: OptimalPath에 해당하는 적절한 속도

  // 매개변수 (논문에서 정의된 값):
  //   NumPathCandidates: 생성할 경로 후보 개수 (예: 21)
  //   Ds: 경로 후보의 아크 길이 (계획 거리)
  //   q_end_values: 경로 후보 끝점의 횡방향 오프셋 값 집합
  //   r: 정적 안전성 계산을 위한 Gaussian 분포의 표준 편차
  //   a, b: Comfortability Cost 계산을 위한 가중치
  //   ws, wc, wd: Total Cost 계산을 위한 정적 안전성, 편안함, 동적 안전성 가중치
  //   Lc: 충돌 범위 (차량 및 장애물 반지름 합)
  //   Lf: 움직이는 장애물 뒤따르기 안전 거리
  //   v_sign: 도로 제한 속도
  //   jaljmax: 최대 허용 횡방향 가속도
  //   ksafe: 속도 조절을 위한 안전 이득
  //   vcurve: 경로 곡률 기반 기준 속도

  // 1. Center Line Construction (중심선 구성) (Section 2.1)
  // MapData의 웨이포인트에서 중심 웨이포인트를 추출하고 Cubic Spline 피팅
  CenterWaypoints = ExtractCenterWaypoints(MapData)
  CenterLineSpline = FitParametricCubicSpline(CenterWaypoints)
  CenterLineSpline = ParameterizeSplineByArcLength(CenterLineSpline)
  // CenterLineSpline은 s에 대한 (x0, y0) 함수 형태로 표현됨 (Equation 1)

  // 2. Path Candidates Generation (경로 후보 생성) (Section 2.2)

  // 2.1 Localization on the center line (중심선 상에서의 현재 위치 찾기) (Section 2.2.1)
  (s_start, q_start) = MapCartesianToSQ(CurrentVehicleState.Position, CenterLineSpline)
  Dh_start = CalculateRelativeHeading(CurrentVehicleState.Heading, s_start, CenterLineSpline) // 현재 차량 방향과 중심선 접선 방향의 차이

  // 2.2 Path candidates generation in the s-q coordinate system (s-q 좌표계에서 경로 후보 생성) (Section 2.2.2)
  PathCandidates_sq = []
  FOR each q_end IN q_end_values DO
    // s-q 함수 q(s)의 계수 a, b, c 결정 (Equation 4)
    // 경계 조건 사용: q(s_start)=q_start, q(s_end)=q_end, dq/ds(s_start)=tan(Dh_start), dq/ds(s_end)=0 (Equation 5)
    coefficients = SolveCubicPolynomialCoefficients(s_start, q_start, Dh_start, s_start + Ds, q_end, 0)
    // s-q 함수 q(s) 정의
    q_function = CreateSQPathFunction(coefficients, s_start, s_start + Ds, q_end)
    Add q_function to PathCandidates_sq
  ENDFOR

  // 2.3 Coordinates conversion of path candidate points (경로 후보 점들의 좌표 변환) (Section 2.2.3)
  PathCandidates_Cartesian = []
  FOR each q_function IN PathCandidates_sq DO
    Path_Cartesian = []
    FOR s_point from s_start to s_start + Ds step D_s_small DO // 작은 간격으로 s 증가
      q_point = q_function(s_point)
      (x, y, heading, curvature) = ConvertSQToCartesian(s_point, q_point, CenterLineSpline) // Equation 6, 7, 8 사용
      Add (x, y, heading, curvature) to Path_Cartesian
    ENDFOR
    Add Path_Cartesian to PathCandidates_Cartesian
  ENDFOR

  // 3. Path Selection (경로 선택) (Section 2.3)
  MinTotalCost = Infinity
  OptimalPath = null
  OptimalAcceleration = null
  OptimalSpeed = null

  FOR each PathCandidate_i (index i) IN PathCandidates_Cartesian DO
    r_i = PathCandidate_i

    // 3.1 Cost function for static safety (정적 안전성 비용 함수) (Section 2.3.1)
    CollisionCheck_i = [] // PathCandidate_i의 각 지점에 대한 충돌 체크 값 시퀀스
    FOR each point in r_i DO
      CHECK collision with RoadEdges, LaneLines, StaticObstacles based on point position
      Assign value based on collision type (1, 0.5, 0.2, 0)
      Add value to CollisionCheck_i
    ENDFOR
    // Collision Check 값 시퀀스 R 구성 (Equation 10의 R[k+i]에 해당)
    R_sequence = GenerateCollisionRiskSequence(CollisionCheck_i)
    // Inverted Gaussian 함수 gi[k] 생성 (Equation 11)
    gi_function = CreateInvertedGaussianFunction(i, r, NumPathCandidates)
    // Static safety cost fs(ri) 계산 (Equation 10) - 이산 컨볼루션
    fs_ri = CalculateConvolution(gi_function, R_sequence)

    // 3.2 Cost function for comfortability (편안함 비용 함수) (Section 2.3.2)
    // Smoothness cost fsm(ri) 계산 (Section 2.3.2)
    fsm_ri = CalculateSmoothnessCost(r_i) // Equation 12 (curvature 사용)
    // Consistency cost fco(ri) 계산 (Section 2.3.2)
    fco_ri = CalculateConsistencyCost(r_i, PreviousSelectedPath) // Equation 13 (heading angle 차이 사용)
    // Comfortability cost fc(ri) 계산 (Equation 14)
    fc_ri = a * fsm_ri + b * fco_ri

    // 3.3 Cost function for dynamic safety (동적 안전성 비용 함수) (Section 2.3.3)
    // 기본적으로 '뒤따르기' 전략으로 회피 가속도 계산
    fd_ri = 0 // 초기화
    Calculated_a_ri = 0 // 초기화
    IsAvailablePath = CHECK if r_i is free of static obstacles/edges (CollisionCheck_i 모든 값이 < 1)

    IF IsAvailablePath THEN
      FOR each MovingObstacle in ObstacleData DO
        Find potential collision points on r_i with MovingObstacle trajectory
        IF potential collision points found THEN
          Ds_ri = CalculateArcLengthToClosestCollisionPoint(r_i, CollisionPoints)
          IF Ds_ri > Lc THEN // 충돌 범위보다 멀리 있는 경우 고려
             // 잠재적 충돌 시간 t(ri) 계산 (Equation 15) - v0는 현재 차량 속도
             t_ri = (Ds_ri - Lc) / CurrentVehicleState.Speed
             // 충돌 회피를 위한 필요 가속도 계산 (Equation 16)
             a_ri = (2 * (Lc - Lf) - CurrentVehicleState.Speed^2) / (Ds_ri - Lc)^2
             Calculated_a_ri = a_ri // 임시 저장 (여러 장애물 시 최소 가속도 사용 등 전략 필요)
             // 실제 구현에서는 가장 제약이 큰(가장 감속이 필요한) 움직이는 장애물을 기준으로 가속도를 결정하거나, 여러 움직이는 장애물을 고려하는 로직 필요
             BREAK // 단순화: 첫 번째 감지된 움직이는 장애물에 대해 계산
          ENDIF
        ENDIF
      ENDFOR
      
      // 계산된 가속도 평활화 (Gaussian filtering) (Section 2.3.3)
      // 실제 구현에서는 시간 경과에 따른 가속도 값을 누적하여 필터링 적용
      Smoothed_a_ri = ApplyGaussianFiltering(Calculated_a_ri, PastAccelerationValues)

      // 속도 제한 계산 (Section 2.3.3)
      vj_ri = CalculateSpeedLimitByCurvature(r_i, jaljmax) // Equation 19
      vr_ri = CalculateSpeedLimitByRisk(fs_ri, ksafe, vcurve) // Equation 20
      v_limit_ri = MIN(vj_ri, vr_ri, v_sign) // Equation 18

      // 필요 가속도를 속도 제한 내로 조정 (Equation 21)
      Adjusted_a_ri = AdjustAccelerationForSpeedLimit(Smoothed_a_ri, CurrentVehicleState.Speed, v_limit_ri, Ds)

      // Dynamic safety cost fd(ri, a(ri)) 계산 (Equation 22)
      // Ds(ri)는 경로 전체 아크 길이 사용 또는 충돌 지점까지의 아크 길이 사용 여부 명확히 필요 (논문 Equation 22는 Ds(ri)-Lf 사용)
      // 여기서는 Equation 22 그대로 사용
      fd_ri = ABS(Adjusted_a_ri) * (Ds - Lf) // Ds는 경로 후보의 총 계획 길이로 가정

      Current_a_ri = Adjusted_a_ri // 이 경로 후보에 대한 최종 가속도
    ELSE
      // 정적 충돌 경로인 경우, 동적 안전 비용을 매우 높게 설정하거나 건너뜀
      // 논문은 "available paths"에 대해서만 고려하므로, 여기서는 건너뜀
      CONTINUE // 다음 경로 후보로 이동
    ENDIF

    // 3.4 Total cost function for path selection (경로 선택을 위한 총 비용 함수) (Section 2.3.4)
    // 비용 함수 값 정규화 [0, 1]
    Normalized_fs = Normalize(fs_ri, min_fs, max_fs) // min_fs, max_fs는 예상 범위
    Normalized_fc = Normalize(fc_ri, min_fc, max_fc)
    Normalized_fd = Normalize(fd_ri, min_fd, max_fd)

    // 총 비용 계산 (Equation 23)
    TotalCost_i = ws * Normalized_fs + wc * Normalized_fc + wd * Normalized_fd

    // 최적 경로 선택 (Equation 9)
    IF TotalCost_i < MinTotalCost THEN
      MinTotalCost = TotalCost_i
      OptimalPath = r_i
      OptimalAcceleration = Current_a_ri
      // OptimalSpeed는 OptimalAcceleration과 현재 속도, 계획 시간(Ds에 해당)을 기반으로 계산
      OptimalSpeed = CalculateSpeedAfterPlanningStep(CurrentVehicleState.Speed, Current_a_ri, Ds / CurrentVehicleState.Speed) // 단순 가정, 실제는 ds/v 적분 또는 등가속 운동 공식 사용
      // 속도 제한 적용
      OptimalSpeed = MIN(OptimalSpeed, v_limit_ri)
    ENDIF

  ENDFOR // PathCandidates_Cartesian FOR 루프 종료

  // 선택된 최적 경로, 가속도, 속도 반환
  RETURN OptimalPath, OptimalAcceleration, OptimalSpeed

END FUNCTION // DynamicPathPlanning 함수 종료

// --- Helper Functions (도우미 함수) ---
FUNCTION ExtractCenterWaypoints(MapData)
  // 미리 정의된 도로 가장자리 웨이포인트 쌍에서 중심점 추출
  RETURN list of center points
END FUNCTION

FUNCTION FitParametricCubicSpline(Points)
  // 주어진 점들을 통과하는 매개변수화된 Cubic Spline 곡선 피팅
  RETURN CubicSpline object
END FUNCTION

FUNCTION ParameterizeSplineByArcLength(Spline)
  // Cubic Spline을 호 길이(arc length) 's'로 다시 매개변수화
  RETURN ArcLengthParameterizedSpline object
END FUNCTION

FUNCTION MapCartesianToSQ(Position_Cartesian, CenterLineSpline)
  // Cartesian 좌표를 s-q 좌표계로 변환
  // CenterLineSpline에서 Position_Cartesian에 가장 가까운 점(s, q) 찾기 (Section 2.2.1)
  RETURN (s, q)
END FUNCTION

FUNCTION CalculateRelativeHeading(VehicleHeading, s, CenterLineSpline)
  // 현재 차량 방향과 중심선 s 지점에서의 접선 방향 사이의 각도 차이 계산
  CenterLineTangentHeading = GetTangentHeading(s, CenterLineSpline)
  RETURN AngleDifference(VehicleHeading, CenterLineTangentHeading)
END FUNCTION

FUNCTION SolveCubicPolynomialCoefficients(s_start, q_start, Dh_start, s_end, q_end, Dh_end)
  // 주어진 경계 조건(s, q, dq/ds)을 만족하는 3차 다항식 q(s)의 계수 (a, b, c) 계산
  // Dh_end=0은 끝점에서 중심선과 평행하게 되는 조건
  RETURN coefficients (a, b, c)
END FUNCTION

FUNCTION CreateSQPathFunction(coefficients, s_start, s_end, default_q_end)
  // 주어진 계수로 s에 대한 q 값을 계산하는 함수 생성 (Equation 4)
  RETURN function q(s)
END FUNCTION

FUNCTION ConvertSQToCartesian(s, q, CenterLineSpline)
  // s-q 좌표를 Cartesian 좌표 (x, y), heading, curvature로 변환 (Section 2.2.3)
  // 중심선 spline에서 s 지점의 (x0, y0), heading0, curvature0 사용
  // Equation 6, 7, 8 사용하여 계산
  RETURN (x, y, heading, curvature)
END FUNCTION

FUNCTION CHECK collision(Point, ObstacleList)
  // 주어진 점과 장애물 목록 간의 충돌 여부 확인
  RETURN true if collision, false otherwise
END FUNCTION

FUNCTION GenerateCollisionRiskSequence(CollisionCheckSequence)
  // Collision Check 값 시퀀스를 기반으로 Gaussian 컨볼루션 적용 전 위험 시퀀스 생성
  // Equation 10의 R[k+i]에 해당, 일반적으로 CollisionCheckSequence와 동일
  RETURN risk sequence
END FUNCTION

FUNCTION CreateInvertedGaussianFunction(center_index, std_dev, length)
  // 주어진 중심 인덱스, 표준 편차, 길이로 이산 역 Gaussian 함수 g_i[k] 생성 (Equation 11)
  RETURN function gi[k]
END FUNCTION

FUNCTION CalculateConvolution(function1, sequence)
  // 함수와 시퀀스 간의 이산 컨볼루션 계산 (Equation 10)
  RETURN convolution result
END FUNCTION

FUNCTION CalculateSmoothnessCost(PathCandidate)
  // 경로 후보의 곡률을 적분하여 Smoothness Cost 계산 (Equation 12)
  RETURN smoothness cost
END FUNCTION

FUNCTION CalculateConsistencyCost(PathCandidate, PreviousPath)
  // 현재 경로 후보와 이전 경로 간의 일관성 비용 계산 (Equation 13)
  RETURN consistency cost
END FUNCTION

FUNCTION FindPotentialCollisionPoints(PathCandidate, MovingObstacleTrajectory)
  // 경로 후보와 움직이는 장애물 궤적의 교차점 찾기
  RETURN list of points
END FUNCTION

FUNCTION CalculateArcLengthToClosestCollisionPoint(PathCandidate, CollisionPoints)
  // 경로 후보 시작점에서 가장 가까운 충돌 지점까지의 호 길이 계산
  RETURN arc length
END FUNCTION

FUNCTION ApplyGaussianFiltering(AccelerationValue, PastAccelerationValues)
  // 과거 가속도 값과 현재 계산된 가속도 값을 사용하여 평활화 (Gaussian Filtering)
  RETURN smoothed acceleration
END FUNCTION

FUNCTION CalculateSpeedLimitByCurvature(PathCandidate, jaljmax)
  // 경로 곡률 기반 속도 제한 계산 (Equation 19)
  RETURN speed limit
END FUNCTION

FUNCTION CalculateSpeedLimitByRisk(StaticSafetyCost, ksafe, vcurve)
  // 정적 안전성 비용 기반 속도 제한 계산 (Equation 20)
  RETURN speed limit
END FUNCTION

FUNCTION AdjustAccelerationForSpeedLimit(Acceleration, CurrentSpeed, SpeedLimit, PlanningDistance)
  // 계산된 가속도를 속도 제한 내로 조정 (Equation 21 기반)
  // 현재 속도에서 PlanningDistance 이동 후 속도가 SpeedLimit을 초과하지 않도록 가속도 조정
  RETURN adjusted acceleration
END FUNCTION

FUNCTION CalculateSpeedAfterPlanningStep(CurrentSpeed, Acceleration, PlanningTime)
  // 등가속 운동 공식으로 계획 시간 후의 속도 계산 (v = v0 + at) 또는 다른 적분 방법 사용
  RETURN calculated speed
END FUNCTION

FUNCTION Normalize(value, min_value, max_value)
  // 값을 0과 1 사이로 정규화
  RETURN normalized value
END FUNCTION

```

이 슈도코드는 논문에서 설명하는 알고리즘의 주요 흐름과 구성 요소를 개념적으로 나타낸 것입니다. 실제 구현 시에는 각 도우미 함수(Helper Function) 내에서 필요한 수학적 계산 및 데이터 처리가 상세히 이루어져야 합니다. 예를 들어 `ConvertSQToCartesian`, `CalculateSmoothnessCost`, `ApplyGaussianFiltering` 등은 논문의 관련 섹션과 수식을 참고하여 구체적인 코드로 작성되어야 합니다. 또한, 움직이는 장애물이 여러 개일 경우 동적 안전성 비용을 계산하고 최적의 가속도를 결정하는 로직은 더 복잡해질 수 있습니다.

#### 2.3.4. Total cost function for path selection

논문에서 제안하는 동적 경로 계획 방법의 전체적인 절차를 나타내는 슈도코드는 다음과 같습니다. 이 방법은 크게 세 단계로 나뉩니다: 중심선 구축, 경로 후보 생성, 경로 선택.

```pseudocode
// 전역 파라미터 (예시 값)
실행_주기 = 0.067초 // 논문에서 언급된 계획 주기 (약 15Hz)
정적_안전_가중치 = 0.5
편안함_가중치 = 0.2
동적_안전_가중치 = 0.3
// 편안함 비용 내부 가중치 (논문 공식 (14)의 a, b)
부드러움_가중치 = 1.0 // 예시
일관성_가중치 = 1.0 // 예시

Function DynamicPathPlanning(경유지_목록, 현재_차량_상태, 정적_장애물_목록, 동적_장애물_목록, 도로_경계, 차선_정보, 이전_경로):
  // 입력:
  // 경유지_목록: 전역 경로의 기준이 되는 지점 목록
  // 현재_차량_상태: {위치: (x, y), 속도: v, 방향: h}
  // 정적_장애물_목록: 움직이지 않는 장애물 정보
  // 동적_장애물_목록: 움직이는 장애물 정보 (위치, 속도 등)
  // 도로_경계: 도로의 물리적 경계 정보
  // 차선_정보: 차선의 종류 및 위치 정보
  // 이전_경로: 이전 계획 스텝에서 선택된 경로

  // 출력:
  // 최적_경로: 현재 스텝에서 선택된 경로
  // 최적_가속도: 최적 경로에 적합한 가속도
  // 최적_속도: 최적 경로에 적합한 목표 속도

  // 1. 중심선 구축 (Center Line Construction)
  // 사전에 정의된 경유지 목록으로부터 도로의 중심선을 생성합니다.
  // 중심선은 호 길이(arc length) 's'로 매개변수화됩니다.
  중심선 = 중심선_구축(경유지_목록)

  // 2. 경로 후보 생성 (Path Candidates Generation)
  // 현재 차량 상태와 중심선 정보를 바탕으로 여러 개의 경로 후보를 생성합니다.
  // 경로는 s-q 좌표계에서 횡방향 오프셋(q) 함수로 정의된 후 Cartesian 좌표계로 변환됩니다.
  경로_후보_목록 = 경로_후보_생성(중심선, 현재_차량_상태)

  // 3. 경로 선택 (Path Selection)
  // 생성된 경로 후보들에 대해 비용 함수를 계산하여 가장 비용이 낮은 경로를 선택합니다.
  // 동시에 해당 경로에 적합한 가속도를 결정합니다.
  최적_경로, 최적_필요_가속도 = 최적_경로_선택(
      경로_후보_목록,
      현재_차량_상태,
      정적_장애물_목록,
      동적_장애물_목록,
      도로_경계,
      차선_정보,
      이전_경로,
      중심선 // 비용 계산에 필요
  )

  // 최적 가속도와 속도 제한을 고려하여 최종 목표 속도를 결정합니다.
  최적_속도 = 최적_속도_계산(
      최적_필요_가속도,
      현재_차량_상태.속도,
      최적_경로, // 경로의 곡률 등 고려
      정적_장애물_목록, // 위험 기반 속도 제한 고려
      동적_장애물_목록, // 위험 기반 속도 제한 고려
      도로_경계, // 위험 기반 속도 제한 고려
      차선_정보 // 제한 속도 표지판 등 고려
  )


  Return 최적_경로, 최적_필요_가속도, 최적_속도

// --- 하위 함수 ---

Function 중심선_구축(경유지_목록):
  // 입력 경유지에 파라메트릭 3차 스플라인을 맞춥니다.
  스플라인 = 3차_스플라인_맞춤(경유지_목록)
  // 스플라인을 호 길이 's'로 재매개변수화합니다.
  중심선 = 호_길이로_매개변수화(스플라인)
  // 중심선을 따라 각 's' 값에 대한 방향(heading) 및 곡률(curvature) 정보를 계산합니다. (공식 (2), (3) 관련)
  중심선_속성_계산(중심선)
  Return 중심선

Function 경로_후보_생성(중심선, 현재_차량_상태):
  // 현재 차량 위치를 중심선 상에 투영하여 s-q 좌표를 찾습니다. (공식 (1) 관련)
  s_시작, q_시작 = 차량_중심선에_투영(중심선, 현재_차량_상태.위치)
  // 차량 방향과 중심선 접선 방향 간의 상대 각도 차이를 계산합니다.
  중심선_방향_s_시작 = 중심선_방향_가져오기(중심선, s_시작)
  Dh_시작 = 현재_차량_상태.방향 - 중심선_방향_s_시작

  // 경로 후보의 끝점을 정의할 호 길이 전망 거리(lookahead distance) Ds를 계산합니다.
  Ds = 전망_거리_계산(...) // 차량 속도 등에 따라 달라질 수 있음
  s_끝 = s_시작 + Ds

  // 경로 후보들의 가능한 횡방향 끝점 오프셋 q_end 집합을 생성합니다.
  끝점_오프셋_q = 가능한_끝점_오프셋_생성(...) // 예를 들어 차선 폭 내에서 일정한 간격의 값들

  경로_후보_목록 = []
  For 각 q_끝 In 끝점_오프셋_q:
    // 3차 다항식 q(s) = a(s-s_시작)^3 + b(s-s_시작)^2 + c(s-s_시작) + q_시작 의 경계 조건을 정의합니다.
    // 시작점: q(s_시작) = q_시작, dq/ds(s_시작) = tan(Dh_시작)
    // 끝점:   q(s_끝) = q_끝, dq/ds(s_끝) = 0 // 부드러운 전환을 위해 끝점에서 기울기 0 가정 (논문 공식 (5) 관련)
    경계_조건 = { 's_시작': s_시작, 'q_시작': q_시작, 'dqds_시작': tan(Dh_시작),
                's_끝': s_끝, 'q_끝': q_끝, 'dqds_끝': 0 }

    // 경계 조건을 만족하는 3차 다항식의 계수 (a, b, c)를 풉니다.
    계수 = 3차_계수_풀기(경계_조건)

    // s-q 표현의 경로 후보 함수를 생성합니다. (논문 공식 (4) 관련)
    경로_sq_함수 = 경로_sq_함수_생성(계수, s_시작, s_끝)

    // s-q 경로를 Cartesian 좌표계 (x, y)로 변환하고, 해당 경로의 속성(방향, 곡률)을 계산합니다. (논문 공식 (6)-(8) 관련)
    경로_cartesian = sq_경로_cartesian으로_변환(경로_sq_함수, 중심선, s_시작, s_끝)

    경로_후보_목록.Append(경로_cartesian)

  Return 경로_후보_목록

Function 최적_경로_선택(경로_후보_목록, 현재_차량_상태, 정적_장애물_목록, 동적_장애물_목록, 도로_경계, 차선_정보, 이전_경로, 중심선):
  최소_총_비용 = 무한대
  최적_경로_후보 = None
  최적_필요_가속도_후보 = None

  정적_안전_비용_목록 = []
  편안함_비용_목록 = []
  동적_안전_비용_목록 = []
  필요_가속도_목록 = [] // 동적 안전 요구사항에 따라 각 경로에 대해 계산된 가속도

  For 각 r_i In 경로_후보_목록:
    // 정적 안전 비용 (fs) 계산
    // 정적 장애물, 도로 경계, 차선과의 충돌 및 근접성 확인
    충돌_체크_값_목록 = 충돌_체크_수행(r_i, 정적_장애물_목록, 도로_경계, 차선_정보) // 경로를 따라 위험 값 목록 반환
    fs_i = 정적_안전_비용_계산(r_i, 충돌_체크_값_목록) // 위험 값을 집계 (가우시안 컨볼루션 등 사용, 공식 (10) 관련)
    정적_안전_비용_목록.Append(fs_i)

    // 편안함 비용 (fc) 계산
    부드러움_비용 = 부드러움_비용_계산(r_i) // 경로 곡률 제곱 적분 (공식 (12) 관련)
    일관성_비용 = 일관성_비용_계산(r_i, 이전_경로) // 이전 경로와의 방향 차이 적분 (공식 (13) 관련)
    fc_i = 부드러움_가중치 * 부드러움_비용 + 일관성_가중치 * 일관성_비용 // 공식 (14) 사용
    편안함_비용_목록.Append(fc_i)

    // 동적 안전 비용 (fd) 및 필요 가속도 (a_i) 계산
    // 동적 장애물과의 잠재적 충돌 시간/지점을 찾습니다.
    // 충돌 회피(추월/따라가기)에 필요한 가속도를 계산합니다.
    // 곡률, 정적 위험, 제한 속도 표지판 등을 기반으로 속도 제한을 고려합니다. (공식 (18), (19), (20) 관련)
    // 필요 가속도와 거리를 기반으로 fd를 계산합니다. (공식 (22) 관련)
    fd_i, a_i = 동적_안전_비용_가속도_계산(
        r_i,
        현재_차량_상태,
        동적_장애물_목록,
        정적_장애물_목록, // 속도 제한(vr) 계산에 필요
        도로_경계, // 속도 제한(vr) 계산에 필요
        차선_정보, // 속도 제한(vr) 계산에 필요
        중심선 // 동적 장애물 투영 등에 필요할 수 있음
    )
    동적_안전_비용_목록.Append(fd_i)
    필요_가속도_목록.Append(a_i)


  // 계산된 비용 (fs, fc, fd)을 모든 경로 후보에 대해 [0, 1] 범위로 정규화합니다.
  // 이는 주요 가중치(ws, wc, wd)를 사용하여 결합하기 전에 수행됩니다.
  정규화된_정적_안전_비용 = 비용_정규화(정적_안전_비용_목록)
  정규화된_편안함_비용 = 비용_정규화(편안함_비용_목록)
  정규화된_동적_안전_비용 = 비용_정규화(동적_안전_비용_목록)

  // 공식 (23)을 사용하여 각 경로 후보에 대한 총 비용을 계산합니다.
  총_비용_목록 = []
  For i From 0 To len(경로_후보_목록) - 1:
    총_비용_i = (정적_안전_가중치 * 정규화된_정적_안전_비용[i] +
                편안함_가중치 * 정규화된_편안함_비용[i] +
                동적_안전_가중치 * 정규화된_동적_안전_비용[i])
    총_비용_목록.Append(총_비용_i)

    // 총 비용이 가장 낮은 경로 후보를 찾습니다.
    If 총_비용_i < 최소_총_비용:
      최소_총_비용 = 총_비용_i
      최적_경로_인덱스 = i

  // 최소 비용을 가진 경로 후보와 해당하는 필요 가속도를 선택합니다.
  최적_경로_후보 = 경로_후보_목록[최적_경로_인덱스]
  최적_필요_가속도_후보 = 필요_가속도_목록[최적_경로_인덱스]

  // 선택된 최적 가속도에 과거 이력을 고려한 가우시안 필터링을 적용하여 부드럽게 할 수 있습니다. (논문 텍스트 언급)
  // 최적_필요_가속도_후보 = 가우시안_스무딩_적용(최적_필요_가속도_후보, 가속도_이력)


  Return 최적_경로_후보, 최적_필요_가속도_후보

Function 최적_속도_계산(최적_필요_가속도, 현재_속도, 최적_경로, 정적_장애물_목록, 동적_장애물_목록, 도로_경계, 차선_정보):
    // 최적 경로의 속성 및 환경을 기반으로 속도 제한을 결정합니다.
    // vsign: 게시된 제한 속도
    // vj: 경로 곡률 기반 속도 제한 (공식 (19) 관련)
    // vr: 충돌 위험 기반 속도 제한 (공식 (20) 관련)
    속도_제한 = 속도_제한_결정(최적_경로, 정적_장애물_목록, 동적_장애물_목록, 도로_경계, 차선_정보) // 공식 (18), (19), (20) 사용

    // 최적 가속도를 한 스텝 동안 적용했을 때 예상되는 속도를 계산합니다.
    예상_속도 = 현재_속도 + 최적_필요_가속도 * 실행_주기

    // 논문의 제약 조건 (공식 (21))은 계산된 'a(r_i)'가 v_limit을 초과하는 속도를 발생시키지 않아야 함을 의미합니다.
    // 따라서 최적_필요_가속도는 이미 계산 과정에서 이 제한을 고려하고 있지만, 명시적으로 상한을 적용할 수 있습니다.
    최적_속도 = min(예상_속도, 속도_제한)
    최적_속도 = max(최적_속도, 0) // 속도는 음수가 될 수 없음

    Return 최적_속도


// --- 상세 구현이 필요한 하위 함수 (플레이스홀더) ---
// Function 3차_스플라인_맞춤(경유지_목록): ...
// Function 호_길이로_매개변수화(스플라인): ...
// Function 중심선_속성_계산(중심선): ...
// Function 차량_중심선에_투영(중심선, 위치): ... (공식 (1) 관련)
// Function 중심선_방향_가져오기(중심선, s): ...
// Function 전망_거리_계산(...): ...
// Function 가능한_끝점_오프셋_생성(...): ...
// Function 3차_계수_풀기(경계_조건): ... 경계 조건을 만족하는 3차 다항식 계수 풀이
// Function 경로_sq_함수_생성(계수, s_시작, s_끝): ...
// Function sq_경로_cartesian으로_변환(경로_sq_함수, 중심선, s_시작, s_끝): ... (공식 (6)-(8) 구현)
// Function 충돌_체크_수행(r_i, 정적_장애물_목록, 도로_경계, 차선_정보): ... 경로와 환경 요소 간의 충돌/근접성 확인
// Function 정적_안전_비용_계산(r_i, 충돌_체크_값_목록): ... (공식 (10) 구현)
// Function 부드러움_비용_계산(r_i): ... (공식 (12) 구현)
// Function 일관성_비용_계산(r_i, 이전_경로): ... (공식 (13) 구현)
// Function 동적_안전_비용_가속도_계산(r_i, ...): ... (공식 (15)-(22) 구현, 필요 가속도 및 fd 계산)
// Function 속도_제한_결정(경로, ...): ... (공식 (18)-(20) 구현)
// Function 비용_정규화(비용_목록): ... 비용 목록의 값을 [0, 1] 범위로 정규화
// Function 가우시안_스무딩_적용(가속도, 이력): ... 가속도 값에 가우시안 필터링 적용

```

이 슈도코드는 논문의 주요 알고리즘 흐름을 보여줍니다. 각 하위 함수 내에는 논문에서 설명하거나 인용한 구체적인 수학적 계산 및 알고리즘(예: 스플라인 피팅, 최적화 문제 풀이, 가우시안 컨볼루션, 충돌 감지 등)이 구현되어야 합니다.