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
