import math

def proportional(v, v_ref, kp=1.0, a_max=0.5):
    return max(min(kp*(v_ref-v), a_max), -a_max)