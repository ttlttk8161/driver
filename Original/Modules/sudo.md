이 기하학적 계산 부분의 주요 목표는 차선 경계 정보로부터 차량 경로의 핵심 웨이포인트가 될 중간 지점들을 추출하는 것입니다.
알고리즘 단계 (기하학적 계산 부분):

입력:

감지된 좌측 차선 경계 (점들의 집합 또는 표현)
감지된 우측 차선 경계 (점들의 집합 또는 표현)
경로 중간 지점을 추출할 기준이 되는 수평선들의 y-좌표 (논문에서는 3개 사용)
차량의 중심점 (전면 범퍼 중심점) 좌표


처리:

각 수평선에 대해 다음을 수행합니다.
해당 수평선이 좌측 차선 경계와 만나는 교차점의 x-좌표를 찾습니다.
해당 수평선이 우측 차선 경계와 만나는 교차점의 x-좌표를 찾습니다.
좌측 교차점의 x-좌표와 우측 교차점의 x-좌표를 사용하여 두 교차점의 중간 지점 x-좌표를 계산합니다. 중간 지점의 y-좌표는 수평선의 y-좌표와 같습니다.
이렇게 계산된 중간 지점을 경로 웨이포인트 목록에 추가합니다.


차량의 중심점 좌표를 경로 웨이포인트 목록의 시작점으로 추가합니다.


출력:

차량 중심점과 차선 경계에서 추출된 중간 지점들로 구성된 경로 웨이포인트 목록

# 가정:
# - left_lane_boundary: 좌측 차선 경계를 표현하는 데이터 구조 (예: (x, y) 점들의 리스트 또는 곡선 함수)
# - right_lane_boundary: 우측 차선 경계를 표현하는 데이터 구조
# - horizontal_line_ys: 중간 지점을 추출할 수평선들의 y-좌표 리스트 (예: [y1, y2, y3])
# - car_center_point: 차량 중심점의 (x, y) 좌표 (예: (car_x, car_y))
# - find_x_at_y_on_boundary 함수: 주어진 y 좌표에서 차선 경계와 수평선이 만나는 x 좌표를 찾는 함수 (실제 구현에서는 차선 표현 방식에 따라 달라짐)

def extract_waypoints_from_lane_boundaries(left_lane_boundary, right_lane_boundary, horizontal_line_ys, car_center_point):
    """
    차선 경계와 수평선을 이용하여 경로 웨이포인트를 추출하는 함수 (개념적)
    """
    waypoints = []

    # 1. 차량 중심점을 경로의 시작점으로 추가
    waypoints.append(car_center_point)

    # 2. 각 수평선에 대해 중간 지점 계산 및 웨이포인트로 추가
    for y_h in horizontal_line_ys:
        # 주어진 y 좌표에서 좌측 차선 경계의 x 좌표 찾기
        # find_x_at_y_on_boundary 함수는 실제 구현 필요 (예: 보간 또는 가장 가까운 점 사용)
        x_left = find_x_at_y_on_boundary(left_lane_boundary, y_h)

        # 주어진 y 좌표에서 우측 차선 경계의 x 좌표 찾기
        x_right = find_x_at_y_on_boundary(right_lane_boundary, y_h)

        # 좌우 차선 경계 사이의 중간 지점 (x, y_h) 계산
        mid_x = (x_left + x_right) / 2.0
        mid_point = (mid_x, y_h)

        # 계산된 중간 지점을 웨이포인트 리스트에 추가
        waypoints.append(mid_point)

    # 추출된 웨이포인트 리스트 반환 (차량 중심점 + 중간 지점들)
    return waypoints

# 예시 사용 (실제 데이터는 여기서 주어지지 않음)
# lane_waypoints = extract_waypoints_from_lane_boundaries(
#     detected_left_boundary_data,
#     detected_right_boundary_data,
#     [10, 20, 30], # 예시 y 좌표
#     (0, 5) # 예시 차량 중심점 좌표
# )
# print(lane_waypoints) # 이 웨이포인트들이 스플라인 보간의 입력이 됨