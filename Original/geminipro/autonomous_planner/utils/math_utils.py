import math
import numpy as np

def sqrt(x: float) -> float:
    return math.sqrt(x)

def cos(angle_rad: float) -> float:
    return math.cos(angle_rad)

def sin(angle_rad: float) -> float:
    return math.sin(angle_rad)

def tan(angle_rad: float) -> float:
    return math.tan(angle_rad)

def atan2(y: float, x: float) -> float:
    return math.atan2(y, x)

def sign(x: float) -> int:
    if x > 0: return 1
    if x < 0: return -1
    return 0

def solve_linear_system(matrix_A: np.ndarray, vector_b: np.ndarray) -> np.ndarray:
    """Solves Ax = b for x."""
    try:
        return np.linalg.solve(matrix_A, vector_b)
    except np.linalg.LinAlgError:
        # Fallback or error handling if matrix is singular
        print("Warning: Singular matrix in solve_linear_system. Using pseudo-inverse.")
        return np.linalg.pinv(matrix_A) @ vector_b


def normalize_angle(angle_rad: float) -> float:
    """Normalize an angle to the range [-pi, pi]."""
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad