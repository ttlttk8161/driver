이 의사 코드는 그림 2에서 설명된 중심선 구성 및 중심선 상에서의 차량 위치 파악 과정을 나타냅니다.
// 자율 주행 차량의 동적 경로 계획 과정 (일부)

// 입력: 미리 정의된 중심 웨이포인트 목록 (List of center waypoints)
// 출력: 구성된 중심선, 차량의 현재 s-q 좌표

Function DynamicPathPlanningStep(currentVehiclePosition):
    // Stage 1: Center Line Construction (중심선 구성)
    // 이 단계는 일반적으로 초기 설정 또는 지도가 변경될 때 한 번 수행되지만, 논문에서는 매 계획 단계마다 사용 가능한 정보를 기반으로 한다고 설명합니다.
    // 여기서는 이미 웨이포인트가 주어져 있다고 가정합니다.
    
    centerWaypoints = GetCenterWaypointsFromMap() // 지도에서 중심 웨이포인트 목록 가져오기
    
    If centerLineNotConstructedYet(): // 중심선이 아직 구성되지 않았다면
        ConstructCenterLine(centerWaypoints) // 웨이포인트를 사용하여 중심선 구성
    End If
    
    // ConstructCenterLine Function (세부 단계)
    Function ConstructCenterLine(waypoints):
        // 웨이포인트를 연결하여 3차 스플라인(cubic spline) 곡선 생성
        splineCurve = FitCubicSplineToWaypoints(waypoints)
        
        // 스플라인 곡선을 호 길이(arc length)로 매개변수화 (parameterize by arc length)
        // 각 세그먼트의 호 길이 계산 및 누적
        ParameterizedSpline = ParameterizeByArcLength(splineCurve)
        
        // 중심선(centerLine)으로 저장
        SetCenterLine(ParameterizedSpline)
    End Function
    
    // Stage 2: Vehicle Localization on the Center Line (중심선 상에서의 차량 위치 파악)
    
    centerLine = GetCenterLine() // 이전에 구성된 중심선 가져오기
    
    // 차량의 현재 위치(카르테시안 좌표)를 s-q 좌표계로 변환
    // 중심선에서 가장 가까운 점(p0) 찾기
    closestPointOnCenterLine (p0) = FindClosestPoint(centerLine, currentVehiclePosition) // 이차 최소화 및 뉴턴 방법 등 사용
    
    // 가장 가까운 점(p0)에 해당하는 중심선 상의 호 길이(s) 계산
    s_coordinate = GetArcLengthOfPoint(centerLine, p0)
    
    // 차량 현재 위치와 가장 가까운 점(p0) 사이의 거리(rho) 계산
    q_coordinate = CalculateDistance(currentVehiclePosition, p0) // 횡방향 오프셋 (lateral offset)
    
    // 차량의 현재 위치를 s-q 좌표로 표현
    vehicle_sq_position = (s_coordinate, q_coordinate)
    
    // 이 s-q 좌표와 중심선 정보를 사용하여 다음 단계인 경로 후보군 생성 진행
    // ... Path candidates generation process starts here ...

    Return vehicle_sq_position // 차량의 s-q 좌표 반환
    
End Function

// Helper Functions (도우미 함수 - 논문에서 언급된 기법)
Function FitCubicSplineToWaypoints(waypoints):
    // 주어진 웨이포인트들을 통과하는 3차 스플라인 곡선을 계산하는 로직 (수식 (1)의 계수 결정)
    // 경계 조건 (위치, 접선 방향 등) 고려
    Return computedSplineCurve
End Function

Function ParameterizeByArcLength(splineCurve):
    // 스플라인 곡선을 따라가며 호 길이를 계산하고, 각 점을 호 길이로 매핑할 수 있도록 데이터 구조를 생성
    // 논문에서 언급된 수치 적분(quadrature) 방법 사용
    Return arcLengthParameterizedSpline
End Function

Function FindClosestPoint(centerLine, position):
    // 주어진 위치에서 중심선 상의 가장 가까운 점을 찾는 알고리즘 구현 (이차 최소화 + 뉴턴 방법 등)
    Return closestPointOnCenterLine
End Function

Function GetArcLengthOfPoint(centerLine, point):
    // 호 길이로 매개변수화된 중심선에서 주어진 점에 해당하는 호 길이를 찾아 반환
    Return arcLengthS
End Function

Function CalculateDistance(point1, point2):
    // 두 점 사이의 유클리드 거리를 계산
    Return distance
End Function