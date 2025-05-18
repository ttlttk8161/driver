import math
from enum import Enum

# 장애물 유형 정의
class ObstacleType(Enum):
    SMALL_VEHICLE = 1
    LARGE_VEHICLE = 2
    OTHER_OBSTACLE = 3 # 논문에서는 대형/소형 차량만 언급했지만, 확장 가능

# 장애물 중요도에 따른 가중치 (예시 값)
# 대형 차량이 소형 차량보다 회피 중요도가 높다고 가정
IMPORTANCE_WEIGHTS = {
    ObstacleType.SMALL_VEHICLE: 1.0,
    ObstacleType.LARGE_VEHICLE: 2.5, # 대형 차량에 더 높은 가중치 부여
    ObstacleType.OTHER_OBSTACLE: 1.5 # 다른 장애물 유형도 고려 가능
}

# 장애물 정보 구조체 (간단화)
class Obstacle:
    def __init__(self, x, y, obstacle_type: ObstacleType, distance: float = float('inf')):
        self.x = x # 라이다 좌표계 기준 x
        self.y = y # 라이다 좌표계 기준 y
        self.obstacle_type = obstacle_type
        self.distance = distance # 차량으로부터의 직선 거리 (옵션)

    def get_position(self):
        return (self.x, self.y)

# 차량 위치 정보 구조체 (간단화) - 여기서는 차량의 현재 위치를 (0,0)으로 간주하고 장애물은 상대 위치로 표현
# class Vehicle:
#     def __init__(self, x, y):
#         self.x = x
#         self.y = y
#
#     def get_position(self):
#         return (self.x, self.y)

# 두 점 사이의 유클리드 거리 계산
def calculate_distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

# 장애물 중요도에 따른 가중치 가져오기
def get_importance_weight(obstacle_type: ObstacleType):
    return IMPORTANCE_WEIGHTS.get(obstacle_type, 1.0) # 기본값 설정

# 장애물과의 거리에 기반한 비용 함수 (예시)
# 거리가 가까울수록 비용이 높아지도록 설정
# 분모가 0이 되는 것을 방지하기 위해 작은 상수 epsilon 추가
def distance_cost_function(distance):
    epsilon = 0.1 # 아주 가까울 때 발산 방지
    # 거리가 매우 작으면 비용을 매우 크게, 거리가 멀면 비용을 작게
    if distance < epsilon: # 매우 가까운 경우 (거의 충돌)
        return float('inf') # 매우 큰 비용
    return 1.0 / (distance + epsilon)

# 논문의 아이디어: 장애물 중요도와 거리를 조합한 비용 계산
# 중요도 가중치 * 거리 기반 비용
def calculate_single_obstacle_avoidance_cost(vehicle_relative_pos, obstacle: Obstacle):
    # vehicle_relative_pos는 장애물 회피 판단 시점의 차량 위치 (여기서는 (0,0)으로 가정)
    # obstacle.get_position()은 차량 기준 장애물의 상대 위치
    distance = calculate_distance(vehicle_relative_pos, obstacle.get_position())
    importance_weight = get_importance_weight(obstacle.obstacle_type)

    # 거리가 특정 임계값 이상이면 비용이 0이 되도록 설정할 수도 있음
    # safe_distance_threshold = 5.0 # 예시: 5미터
    # if distance > safe_distance_threshold:
    #     return 0

    cost = importance_weight * distance_cost_function(distance)
    return cost

# 주어진 경로(위치들의 리스트)에 대한 총 장애물 회피 비용 계산
# path는 차량의 예상 이동 경로 상의 (상대) 좌표들
def evaluate_path_obstacle_cost(path, obstacles):
    total_obstacle_cost = 0
    # 경로 상의 각 위치(또는 주요 지점)에 대해 장애물 비용 누적
    for vehicle_pos_on_path in path:
        for obstacle in obstacles:
            total_obstacle_cost += calculate_single_obstacle_avoidance_cost(vehicle_pos_on_path, obstacle)
    
    return total_obstacle_cost

if __name__ == "__main__":
    # 예시 장애물 생성 (차량 기준 상대 좌표라고 가정)
    obstacles_example = [
        Obstacle(x=1.0, y=0.5, obstacle_type=ObstacleType.SMALL_VEHICLE), # 전방 1m, 우측 0.5m
        Obstacle(x=1.2, y=-0.6, obstacle_type=ObstacleType.LARGE_VEHICLE), # 전방 1.2m, 좌측 0.6m
        Obstacle(x=0.8, y=0.0, obstacle_type=ObstacleType.OTHER_OBSTACLE),  # 전방 0.8m 중앙
    ]

    # 차량의 현재 위치는 항상 (0,0)으로 간주 (상대 좌표계)
    vehicle_current_relative_pos = (0, 0)

    print(f"차량 현재 상대 위치: {vehicle_current_relative_pos}")
    print("-" * 20)

    # 각 장애물에 대한 회피 비용 계산 (현재 위치 기준)
    for i, obs in enumerate(obstacles_example):
        cost = calculate_single_obstacle_avoidance_cost(vehicle_current_relative_pos, obs)
        dist = calculate_distance(vehicle_current_relative_pos, obs.get_position())
        print(f"장애물 {i+1} ({obs.obstacle_type.name}, 상대위치: {obs.get_position()}):")
        print(f"  - 거리: {dist:.2f}")
        print(f"  - 중요도 가중치: {get_importance_weight(obs.obstacle_type)}")
        print(f"  - 현재 위치에서의 회피 비용: {cost:.2f}")

    print("-" * 20)

    # 예시 경로 (차량 중심 기준 상대 좌표 경로)
    # 경로 1: 약간 우측으로 회피
    example_path_1 = [(0,0), (0.5, 0.1), (1.0, 0.2), (1.5, 0.3), (2.0, 0.4)]
    # 경로 2: 약간 좌측으로 회피
    example_path_2 = [(0,0), (0.5, -0.1), (1.0, -0.2), (1.5, -0.3), (2.0, -0.4)]

    print("예시 경로 1 평가:")
    cost1 = evaluate_path_obstacle_cost(example_path_1, obstacles_example)
    print(f"  - 총 장애물 회피 비용: {cost1:.2f}")

    print("예시 경로 2 평가:")
    cost2 = evaluate_path_obstacle_cost(example_path_2, obstacles_example)
    print(f"  - 총 장애물 회피 비용: {cost2:.2f}")

    # 비용이 낮은 경로가 더 선호됨.
    # 실제 적용 시에는 차선 유지 비용, 경로 길이, 부드러움 등 다른 비용/보상과 결합하여
    # 최종 경로를 선택하거나 조향각/속도를 결정해야 합니다.