#自动导航1.0
#导入模块
from _future import print_function
import time
from dronekit import *
from pymavlink import mavuti



#定义全局变量
global safe_fo  #有无障碍
global state_t  #飞行姿态
global speed_t  #飞行速度
global fly_hight_rec #推荐飞行高度
global put_hight_rec #推荐投放高度
global back_home #返航指令 0：原地降落，1：返航到起飞点，2降落到指定地点
global coord_load #指定降落地点
global num #投放点数量
global coord_putdown #投放点坐标
global coord_putdown_now #当前投放坐标点
global coord_temp #目标坐标(临时）




#全局变量默认值
safe_fo = 0
state_t=[0.0,0.0,0.0,0.0,0.0,0.0]
speed_t=[0.0,0.0,0.0]
fly_hight_rec=1.00
put_hight_rec=1.00
back_home=1
coord_load=[0.0,0.0,0.0]
num=0
coord_putdown=[0.0,0.0,0.0]
coord_temp=[0.0,0.0,0.0]
coord_putdown_now=[0.0,0.0,0.0]


#子函数

#姿态修正函数(未完成）
def recorrect(coord):
    pass



#判定有无障碍(未完成)
def barrier():
    pass




#自动导航模块
def auto():
    #确保无人机处于安全高度
    while vehicle.rangefinder.distance<=fly_hight_rec*0.9:
        print("等待无人机飞到指定高度")
        sleep(0.5)
        
    n=0#记录投放次数

    while True:
        i=0
        while i<3:
            coord_putdown_now[i]= coord_putdown[n*3+i]
            i=i+1


