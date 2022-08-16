from math import pi

def normalize_angle(angle):
    # type: (float) -> float
    if angle < -pi:
        angle += 2 * pi
    elif angle > pi:
        angle -= 2 * pi
    return angle
