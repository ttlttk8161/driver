from dataclasses import dataclass, field
from typing import List, Tuple

@dataclass
class Waypoint:
    x: float
    y: float

@dataclass
class VehicleState:
    position: Waypoint
    heading_angle: float  # radians
    speed: float          # m/s

@dataclass
class Obstacle: # Simplified
    position: Waypoint
    radius: float = 0.5 # Assuming circular for simplicity

@dataclass
class MovingObstacle(Obstacle):
    velocity: Waypoint = field(default_factory=lambda: Waypoint(0, 0)) # vx, vy

@dataclass
class SplineSegment:
    # Coefficients for x(s_local) = ax*s_local^3 + bx*s_local^2 + cx*s_local + dx
    # Coefficients for y(s_local) = ay*s_local^3 + by*s_local^2 + cy*s_local + dy
    # s_local is arc length from the start of this segment, range [0, segment_arc_length]
    ax: float
    bx: float
    cx: float
    dx: float
    ay: float
    by: float
    cy: float
    dy: float
    segment_arc_length: float
    start_s_global: float # Cumulative arc length from the very beginning of the center line

    def get_coords(self, s_local: float) -> Waypoint:
        x = self.ax * s_local**3 + self.bx * s_local**2 + self.cx * s_local + self.dx
        y = self.ay * s_local**3 + self.by * s_local**2 + self.cy * s_local + self.dy
        return Waypoint(x, y)

    def get_derivative_coords(self, s_local: float) -> Waypoint: # dx/ds_local, dy/ds_local
        dx_ds = 3 * self.ax * s_local**2 + 2 * self.bx * s_local + self.cx
        dy_ds = 3 * self.ay * s_local**2 + 2 * self.by * s_local + self.cy
        return Waypoint(dx_ds, dy_ds)

    def get_second_derivative_coords(self, s_local: float) -> Waypoint: # d2x/ds_local2, d2y/ds_local2
        d2x_ds2 = 6 * self.ax * s_local + 2 * self.bx
        d2y_ds2 = 6 * self.ay * s_local + 2 * self.by
        return Waypoint(d2x_ds2, d2y_ds2)

@dataclass
class CenterLine:
    segments: List[SplineSegment] = field(default_factory=list)
    cumulative_arc_lengths: List[float] = field(default_factory=list) # Corresponds to end of each center_waypoint

    def get_global_s_and_segment(self, s_global: float) -> Tuple[SplineSegment, float, int]:
        if not self.segments:
            raise ValueError("Center line has no segments.")
        if s_global < 0:
            s_global = 0
        
        segment_idx = -1
        for i, seg in enumerate(self.segments):
            if seg.start_s_global <= s_global < seg.start_s_global + seg.segment_arc_length:
                segment_idx = i
                break
        
        if segment_idx == -1: # s_global is beyond the last segment
            segment_idx = len(self.segments) - 1
            s_local = self.segments[segment_idx].segment_arc_length
        else:
             s_local = s_global - self.segments[segment_idx].start_s_global
        
        # Ensure s_local is within segment bounds for safety, though logic above should handle it
        s_local = max(0, min(s_local, self.segments[segment_idx].segment_arc_length))
        return self.segments[segment_idx], s_local, segment_idx


    def get_cartesian_coords(self, s_global: float) -> Waypoint:
        """Get (x0, y0) at global arc length s."""
        segment, s_local, _ = self.get_global_s_and_segment(s_global)
        return segment.get_coords(s_local)

    def get_tangent_angle(self, s_global: float) -> float:
        """Get h0 at global arc length s."""
        segment, s_local, _ = self.get_global_s_and_segment(s_global)
        deriv = segment.get_derivative_coords(s_local)
        # Since s_local is arc length, dx/ds and dy/ds are cos(h0) and sin(h0)
        return math.atan2(deriv.y, deriv.x)

    def get_curvature(self, s_global: float) -> float:
        """Get j0 at global arc length s. (Eq. 3 - corrected standard form)"""
        segment, s_local, _ = self.get_global_s_and_segment(s_global)
        # If s_local is arc length, x'^2 + y'^2 = 1 (ideally)
        # j0 = x'y'' - y'x''
        # (or standard kappa = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2) )
        # Using x'y'' - y'x'' assuming s_local is arc length and direction matters for j0
        
        xp, yp = segment.get_derivative_coords(s_local).x, segment.get_derivative_coords(s_local).y
        xpp, ypp = segment.get_second_derivative_coords(s_local).x, segment.get_second_derivative_coords(s_local).y
        
        denominator = (xp**2 + yp**2)**1.5
        if abs(denominator) < 1e-6: # Avoid division by zero
            return 0.0 
        
        # Standard formula for signed curvature
        curvature = (xp * ypp - xpp * yp) / denominator
        return curvature


@dataclass
class PathCandidate:
    points_cartesian: List[Waypoint] = field(default_factory=list) # x,y points
    s_values: List[float] = field(default_factory=list)       # s value for each point
    q_values: List[float] = field(default_factory=list)       # q value for each point (relative to center line)
    heading_values: List[float] = field(default_factory=list) # heading at each point
    curvature_values: List[float] = field(default_factory=list)# curvature j at each point
    
    # Coefficients for q(s) = a(s-s_start)^3 + b(s-s_start)^2 + c(s-s_start) + q_start
    # s is global s on the center line
    s_start_global: float = 0.0
    q_start: float = 0.0
    coeff_a: float = 0.0
    coeff_b: float = 0.0
    coeff_c: float = 0.0
    
    cost_static_safety: float = 0.0
    cost_comfortability: float = 0.0
    cost_dynamic_safety: float = 0.0
    total_cost: float = float('inf')
    associated_acceleration: float = 0.0 # a(ri) for this path

    def get_length(self) -> float:
        if not self.points_cartesian or len(self.points_cartesian) < 2:
            return 0.0
        length = 0.0
        for i in range(len(self.points_cartesian) - 1):
            p1 = self.points_cartesian[i]
            p2 = self.points_cartesian[i+1]
            length += math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        return length