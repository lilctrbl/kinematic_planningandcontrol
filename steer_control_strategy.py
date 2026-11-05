import math

def stanley(e_y, e_th, v, k=1.0):
    # e_y>0 表示车辆在轨迹左侧，delta>0 表示左转，因此横向与航向修正项都需要取负号，否则会反向远离轨迹
    return -(e_th + math.atan2(k*e_y, max(v, 0.1)))

def pure_pursuit(e_y, ld=1.0):
    #ld:预瞄距离
    return math.atan2(-2*0.175*e_y, ld*ld) 

def proportional(e_y, kp=1.0):
    return -kp*e_y