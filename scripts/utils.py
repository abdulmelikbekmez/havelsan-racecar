def refactor_angle(angle):
    # type: (float) -> float
    if angle < -180:
        angle += 360
    elif angle > 180:
        angle -= 360
    return angle

def refactor_angle2(angle):
    # type: (float) -> float
    if -90 > angle >= -180:
        angle += 180
        angle *=-1
    elif angle < -180:
        angle += 180
    elif 90 < angle <= 180:
        angle -= 180
        angle *= -1
    elif angle > 180:
        angle -= 180
    return angle