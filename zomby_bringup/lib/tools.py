"""Tool functions for the package."""

def map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def saturate(value: float, max: float, min: float):
    sat_value: float
    if value > max:
        sat_value = max
    elif value < min:
        sat_value = min
    else:
        sat_value = value

    return sat_value