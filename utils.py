import math

def kmph_to_mps(speed):
    return speed/3.6


def square_area(edge):
    return edge**2

def diagonal_from_square_area(area):
    edge = math.sqrt(area)
    diag = math.sqrt(2)*edge
    return diag

def circle_area(radius):
    return math.pi * radius**2

def inch_to_m(len):
    return len*2.54/100