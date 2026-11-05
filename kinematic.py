import math
import steer_control_strategy
import speed_control_strategy
import matplotlib.pyplot as plt
# 坐标系与初值
x_k       = 0
y_k       = 0
v_k       = 0
T_s       = 0.001
a_max     = 0.5
v_max     = 1.5
theta_k   = math.pi/3      #航向角
delta_k   = math.pi/180*0   #前轮转角初始量
delta_max = math.pi/180*30  #前轮转角最大值
# 车辆参数
l         = 0.175 #轴距
tau_steer = 0.150 #一阶系统参数
#跟踪轨迹
r_ref     = 2.0
s1        = 5.0
s2        = math.pi/2*r_ref
s_end     = 16.0
s_r       = 0.0
s_k       = 0.0
x_r       = 0.0
y_r       = 0.0
th_r      = 0.0
#数据存储
r_x        = []
r_y        = []
xpoint     = []
ypoint     = []
aydata     = []
evdata     = []
exdata     = []
eydata     = []
ethdata    = []
deltapoint = []
kappapoint = []
#仿真循环
step    = 0
while step < 12000 and s_r < s_end:
    #参考速度
    v_r      = min(v_max, a_max*step*T_s)
    a_k      = speed_control_strategy.proportional(v_k, v_r)
    v_k     += a_k*T_s
    v_k      = max(min(v_k,v_max), -v_max)
    # 参考轨迹
    if   s_r < s1:
        kappa_r = 0.0
    elif s_r < s1+s2:
        kappa_r = 1/r_ref
    else:
        kappa_r = 0.0
    #参考点
    th_r    += kappa_r*v_r*T_s
    x_r     += v_r*math.cos(th_r)*T_s
    y_r     += v_r*math.sin(th_r)*T_s
    s_r     += v_r*T_s
    #误差计算
    dx   = x_k-x_r
    dy   = y_k-y_r
    e_s  = s_k-s_r
    e_v  = v_k-v_r
    e_y  = -dx*math.sin(th_r)+dy*math.cos(th_r)
    e_th = math.atan2(math.sin(theta_k-th_r),math.cos(theta_k-th_r))

    delta_c  = steer_control_strategy.stanley(e_y, e_th, v_k)
    delta_c  = max(min(delta_c, delta_max), -delta_max)

    delta_k += (1-math.exp(-T_s/tau_steer))*(delta_c-delta_k) #直接输出的是前轮转角
    delta_k  = max(min(delta_k, delta_max), -delta_max)

    kappa_k    = math.tan(delta_k)/l

    a_y      = pow(v_k,2)*math.tan(delta_k)/l
    theta_k += v_k*math.tan(delta_k)*T_s/l

    x_k     += v_k*math.cos(theta_k)*T_s
    y_k     += v_k*math.sin(theta_k)*T_s
    s_k     += v_k*T_s #走过的路径长度


    r_x.append(x_r)
    r_y.append(y_r)
    xpoint.append(x_k)
    ypoint.append(y_k)
    aydata.append(a_y)
    evdata.append(e_v)
    eydata.append(e_y)
    ethdata.append(e_th)
    deltapoint.append(delta_k)
    kappapoint.append(kappa_k)

    step += 1

    if abs(a_y) >= 0.3*9.81:
        print("step:",step,"v_k:",v_k,"a_y:",a_y)

fig,ax = plt.subplots(2,4,figsize=(15,10))
ax[0,0].plot(evdata)
ax[0,1].plot(aydata)
ax[0,2].plot(eydata)
ax[0,3].plot(deltapoint)
ax[1,0].plot(ethdata)
ax[1,1].plot(kappapoint)
ax[1,2].plot(r_x,r_y,label="reference")
ax[1,2].plot(xpoint,ypoint,label="vehicle")
ax[1,2].axis("equal")
ax[1,2].legend()

ax[0,0].set_title("e_v")
ax[0,1].set_title("a_y")
ax[0,2].set_title("e_y")
ax[0,3].set_title("delta")
ax[1,0].set_title("e_th")
ax[1,1].set_title("kappa")
ax[1,2].set_title("position")
plt.show()