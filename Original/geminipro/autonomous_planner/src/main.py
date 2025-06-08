import math
import json # For loading example waypoints
from autonomous_planner.planner import DynamicPathPlanner
from autonomous_planner.utils.data_structures import Waypoint, VehicleState, Obstacle, MovingObstacle

def load_example_waypoints(filepath="data/waypoints_example.json") -> list:
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
        # Assuming JSON is a list of [[lx,ly], [rx,ry]] pairs
        return [(Waypoint(pair[0][0], pair[0][1]), Waypoint(pair[1][0], pair[1][1])) for pair in data]
    except FileNotFoundError:
        print(f"Waypoint file {filepath} not found. Using default straight road.")
        # Default: Straight road along x-axis, 3m wide
        # Lane waypoints (left/right pairs)
        wps = []
        for x in range(0, 101, 10): # 100m long road, points every 10m
            wps.append((Waypoint(float(x), 1.5), Waypoint(float(x), -1.5)))
        return wps
    except Exception as e:
        print(f"Error loading waypoints: {e}. Using default.")
        wps = []
        for x in range(0, 101, 10): 
            wps.append((Waypoint(float(x), 1.5), Waypoint(float(x), -1.5)))
        return wps

def run_example():
    # Initialize planner
    planner = DynamicPathPlanner()

    # Example predefined waypoints (e.g., from a map)
    # Each element is a tuple (waypoint_left_lane_edge, waypoint_right_lane_edge)
    predefined_wps = load_example_waypoints()
    if not predefined_wps:
        print("No waypoints available. Exiting.")
        return

    # Example current vehicle state
    # Initial position near the start of the first waypoint, heading along x-axis
    initial_x = predefined_wps[0][0].x + (predefined_wps[0][1].x - predefined_wps[0][0].x) / 2
    initial_y = predefined_wps[0][0].y + (predefined_wps[0][1].y - predefined_wps[0][0].y) / 2

    vehicle_state = VehicleState(
        position=Waypoint(initial_x, initial_y + 0.1), # Slightly off-center
        heading_angle=math.radians(2.0),      # Slight angle offset
        speed=5.0                          # 5 m/s
    )

    # Example obstacles
    static_obstacles = [
        Obstacle(position=Waypoint(20.0, 0.5), radius=0.7)
    ]
    moving_obstacles = [
        MovingObstacle(position=Waypoint(30.0, 0.0), radius=0.8, velocity=Waypoint(3.0, 0.0)) # Moving along x
    ]

    # Road geometry (simplified, not used in detail by current collision checks)
    road_edges = [] # E.g. [[Waypoint(0,-2), Waypoint(100,-2)], [Waypoint(0,2), Waypoint(100,2)]]
    lane_lines = [] # E.g. [[Waypoint(0,0), Waypoint(100,0)]] type: dashed

    print(f"Initial Vehicle State: pos=({vehicle_state.position.x:.1f},{vehicle_state.position.y:.1f}), "
          f"hdg={math.degrees(vehicle_state.heading_angle):.1f}deg, spd={vehicle_state.speed:.1f}m/s")
    
    # Run one planning cycle
    optimal_path, target_accel, target_speed = planner.plan_path(
        predefined_wps,
        vehicle_state,
        static_obstacles,
        moving_obstacles,
        road_edges,
        lane_lines
    )

    if optimal_path:
        print("\n--- Optimal Path Summary ---")
        print(f"Number of points: {len(optimal_path.points_cartesian)}")
        print(f"Start point: ({optimal_path.points_cartesian[0].x:.2f}, {optimal_path.points_cartesian[0].y:.2f})")
        print(f"End point: ({optimal_path.points_cartesian[-1].x:.2f}, {optimal_path.points_cartesian[-1].y:.2f})")
        print(f"Calculated length: {optimal_path.get_length():.2f}m")
        print(f"Target Acceleration: {target_accel:.2f} m/s^2")
        print(f"Target Speed: {target_speed:.2f} m/s")

        # TODO: Add visualization if matplotlib is available
        # import matplotlib.pyplot as plt
        # # Plot center line
        # if planner.center_line_cache and planner.center_line_cache.segments:
        #     cl_x = [p.x for s_glob in np.linspace(0, planner.center_line_cache.segments[-1].start_s_global + planner.center_line_cache.segments[-1].segment_arc_length, 200) for p in [planner.center_line_cache.get_cartesian_coords(s_glob)]]
        #     cl_y = [p.y for s_glob in np.linspace(0, planner.center_line_cache.segments[-1].start_s_global + planner.center_line_cache.segments[-1].segment_arc_length, 200) for p in [planner.center_line_cache.get_cartesian_coords(s_glob)]]
        #     plt.plot(cl_x, cl_y, 'k--', label="Center Line")
        # # Plot optimal path
        # opt_path_x = [p.x for p in optimal_path.points_cartesian]
        # opt_path_y = [p.y for p in optimal_path.points_cartesian]
        # plt.plot(opt_path_x, opt_path_y, 'g-', label="Optimal Path")
        # # Plot vehicle start
        # plt.plot(vehicle_state.position.x, vehicle_state.position.y, 'ro', label="Vehicle Start")
        # # Plot static obstacles
        # for obs in static_obstacles:
        #     circle = plt.Circle((obs.position.x, obs.position.y), obs.radius, color='red', alpha=0.5)
        #     plt.gca().add_patch(circle)
        # plt.xlabel("X (m)")
        # plt.ylabel("Y (m)")
        # plt.legend()
        # plt.axis('equal')
        # plt.title("Autonomous Path Planning")
        # plt.grid(True)
        # plt.show()

    else:
        print("No path planned.")

if __name__ == "__main__":
    # Create dummy data file if it doesn't exist for the example
    import os
    if not os.path.exists("data"):
        os.makedirs("data")
    if not os.path.exists("data/waypoints_example.json"):
        default_wps_data = []
        for x_coord in range(0, 101, 10):
            default_wps_data.append([[float(x_coord), 1.5], [float(x_coord), -1.5]])
        with open("data/waypoints_example.json", "w") as f:
            json.dump(default_wps_data, f)
            
    run_example()