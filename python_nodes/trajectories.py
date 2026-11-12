import math

def generate_trajectory(points, v, dt):
    """
    Generates relative (dx, dy) steps from point to point using relative offsets.
    
    Args:
        points: List of relative offset waypoints [(dx0, dy0), (dx1, dy1), ...].
        v: Maximum speed in m/s.
        dt: Time step duration in seconds (default: 0.01s / 100Hz).
        
    Yields:
        Tuple[float, float]: Relative step (dx, dy) per time step.
    """
    max_step = v * dt
    if max_step <= 0:
        return

    for target_x, target_y in points:
        # Distance to the next target relative to current position
        dist = math.hypot(target_x, target_y)
        
        # Track remaining offset for this waypoint segment
        rem_x, rem_y = target_x, target_y
        
        while dist > 1e-9:
            if dist <= max_step:
                # Target reachable in a single step
                yield (rem_x, rem_y)
                break
            else:
                # Take a step bounded by max_step along the vector
                ratio = max_step / dist
                step_x = rem_x * ratio
                step_y = rem_y * ratio
                
                yield (step_x, step_y)
                
                # Update remaining distance and vector for this segment
                rem_x -= step_x
                rem_y -= step_y
                dist -= max_step

def verify(traj, speed, verbose=True):
    for point in traj:
        if verbose:
            print(point)
        if math.hypot(*point) > speed:
            return False
    return True


def relative_circle_points(n, r):
    """Return n relative (delta) coordinates describing n equally spaced
    points on a circle of radius r.

    Element k is the displacement from point k-1 to point k
    (point -1 wraps around to point n-1).
    """
    # Absolute points, evenly spaced, starting at angle 0
    abs_pts = [
        (r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n))
        for k in range(n)
    ]

    # Differences between each point and the previous one (cyclic)
    return [
        (abs_pts[k][0] - abs_pts[k - 1][0], abs_pts[k][1] - abs_pts[k - 1][1])
        for k in range(n)
    ]

out = relative_circle_points(12, 0.03)
print([(round(x, 10) + 0.0, round(y, 10) + 0.0) for x, y in out])
