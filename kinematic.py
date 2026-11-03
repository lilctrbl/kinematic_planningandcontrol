import math 
import matplotlib.pyplot as plt
# 坐标系与初值
x_k     = 0
y_k     = 0
a_k     = 0.05
v_k     = 1.0
T_s     = 0.001
theta_k = math.pi/3 #航向角
delta_k = math.pi/180*30  #前轮转角初始量
delta_c = math.pi/180*-60 #前轮转角控制量
# 车辆参数
l       = 0.175 #轴距
#跟踪轨迹
r_ref   = 2.0
s1      = 5.0
s2      = math.pi/2*r_ref
s_end   = 16.0
s_r     = 0.0
x_r     = 0.0
y_r     = 0.0
th_r    = 0.0
#数据存储
r_x        = []
r_y        = []
xpoint     = []
ypoint     = []
aydata     = []
eydata     = []
ethdata    = []
deltapoint = []
kappapoint = []
#仿真循环
step    = 0
while step < 12000 and s_r < s_end:
    v_k     += a_k*T_s
    v_k      = max(min(v_k,1.5), -1.5)
    # 参考轨迹
    if   s_r < s1:
        kappa_r = 0.0
    elif s_r < s1+s2:
        kappa_r = 1/r_ref
    else:
        kappa_r = 0.0
    #参考点
    th_r    += kappa_r*v_k*T_s
    x_r     += v_k*math.cos(th_r)*T_s
    y_r     += v_k*math.sin(th_r)*T_s
    s_r     += v_k*T_s

    delta_k += delta_c*T_s
    delta_k  = max(min(delta_k, math.pi/180*30), math.pi/180*-30)

    kappa_k    = math.tan(delta_k)/l

    a_y      = pow(v_k,2)*math.tan(delta_k)/l
    theta_k += v_k*math.tan(delta_k)*T_s/l

    x_k     += v_k*math.cos(theta_k)*T_s
    y_k     += v_k*math.sin(theta_k)*T_s
    #误差计算
    dx   = x_k-x_r
    dy   = y_k-y_r
    e_y  = -dx*math.sin(th_r)+dy*math.cos(th_r)
    e_th = math.atan2(math.sin(theta_k-th_r),math.cos(theta_k-th_r))

    r_x.append(x_r)
    r_y.append(y_r)
    xpoint.append(x_k)
    ypoint.append(y_k)
    aydata.append(a_y)
    eydata.append(e_y)
    ethdata.append(e_th)
    deltapoint.append(delta_k)
    kappapoint.append(kappa_k)

    step += 1

    if abs(a_y) >= 0.3*9.81:
        print("step:",step,"v_k:",v_k,"a_y:",a_y)

fig,ax = plt.subplots(2,2,figsize=(13,8))
ax[0,0].plot(aydata)
ax[0,1].plot(deltapoint)
ax[1,0].plot(kappapoint)
ax[1,1].plot(r_x,r_y,label="reference")
ax[1,1].plot(xpoint,ypoint,label="vehicle")
ax[1,1].axis("equal")
ax[1,1].legend()

ax[0,0].set_title("a_y")
ax[0,1].set_title("delta")
ax[1,0].set_title("kappa")
ax[1,1].set_title("position")
plt.show()