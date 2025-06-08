import math

# --- Path Generation Parameters (Section 2.2) ---
S_END_ARC_LENGTH: float = 30.0  # Target arc length increment for path candidates (m)
# Example: [-1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5] for a 3m wide lane, considering center
TARGET_LATERAL_OFFSETS_Q_END: list[float] = [-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0]
NUM_POINTS_PER_PATH: int = 20  # Number of points to discretize each path candidate

# --- Cost Function Parameters (Section 2.3) ---
# Weights for total cost (Eq. 23)
WS: float = 0.5  # Static safety
WC: float = 0.2  # Comfortability
WD: float = 0.3  # Dynamic safety

# Static Safety (Eq. 10, 11)
SIGMA_R: float = 1.0  # Standard deviation for Gaussian convolution in static safety
COLLISION_VALUE_NONE: float = 0.0
COLLISION_VALUE_LANE_LINE_DRIVING: float = 0.2 # Passing own lane line
COLLISION_VALUE_LANE_LINE_OPPOSITE: float = 0.5 # Passing opposite lane line / solid line
COLLISION_VALUE_OBSTACLE_OR_EDGE: float = 1.0 # Collision with obstacle or road edge

# Comfortability (Eq. 14)
ALPHA: float = 1.0  # Weight for smoothness cost
BETA: float = 0.5   # Weight for consistency cost

# Dynamic Safety (Eq. 15-22)
LC: float = 2.5  # Collision range (vehicle radius + obstacle radius) (m)
L0: float = 5.0  # Predefined following distance (m)
# Note: Lf is derived from L0 and Ds(ri) using Eq. 17

# Speed Limits (Eq. 18-20)
VSIGN: float = 15.0  # Road speed limit sign (m/s) ~54 km/h
JALJMAX: float = 2.0  # Max allowed lateral acceleration (m/s^2)
KSAFE: float = 0.1    # Safety gain for risk-based speed adjustment (heuristic)
VCURVE: float = 10.0  # Reference speed for curve speed calculation (m/s) ~36 km/h

PLANNING_DT: float = 0.1 # Planning cycle time in seconds (e.g. for 10Hz)

# General
MAX_DECELERATION = -5.0 # m/s^2
MAX_ACCELERATION = 2.0  # m/s^2

# For `ConstructCenterLine`
NUM_INTEGRATION_STEPS_ARC_LENGTH = 100 # For numerical arc length calculation

# Convolution length for static safety cost (must be odd)
# (2N+1) in the paper. If num_candidates is M, N is related to M.
# Let's say we convolve over the nearest X candidates on each side.
# If CONVOLUTION_WINDOW_HALF_SIZE = 2, total window size is 5.
CONVOLUTION_WINDOW_HALF_SIZE = 2 # N in the paper's sum from -N to N