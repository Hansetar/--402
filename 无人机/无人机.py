#自动导航1.0
#导入模块
from _future import print_function
import time
import math
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



#参数文件：


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




#设置速度（完成）
def send_nav_velocity( velocity_x, velocity_y, velocity_z):
    # 生成SET_POSITION_TARGET_LOCAL_NED命令
    msg =Vehicle.message_factory.set_position_target_local_ned_encode(
                    0,       # time_boot_ms (not used)
                    0, 0,    # target system, target component
                    mavutil.mavlink.MAV_FRAME_BODY_NED,   # frame
                    0b0000111111000111, # type_mask (only speeds enabled)
                    0, 0, 0, # x, y, z positions (not used)
                    velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                    0, 0, 0, # x, y, z acceleration (not used)
                    0, 0)    # yaw, yaw_rate (not used) 
    # 发送指令
    Vehicle.send_mavlink(msg)
    Vehicle.flush()





    #设置航向（完成）
def condition_yaw( heading, relative, clock_wise):
    # 使用相对角度或绝对方位
    if relative:
        isRelative = 1
    else:
        isRelative = 0

    # 若使用相对角度，则进行顺时针或逆时针转动
    if clock_wise:
        direction = 1  # "heading"所对应的角度将被加和到当前朝向角
    else:
        direction = -1 # "heading"所对应的角度将被从当前朝向角减去
    if not relative:
        direction = 0

    # 生成CONDITION_YAW命令
    msg = Vehicle.message_factory.command_long_encode(
                    0, 0,       # target system, target component
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
                    0,          # confirmation
                    heading,    # param 1, yaw in degrees
                    0,          # param 2, yaw speed (not used)
                    direction,  # param 3, direction
                    isRelative, # param 4, relative or absolute degrees 
                    0, 0, 0)    # param 5-7, not used
    # 发送指令
    Vehicle.send_mavlink(msg)
    Vehicle.flush()






#姿态修正函数(完成）
def recorrect(coord):

#修正高度
    while True:
        v_z=(coord[2]-Vehicle._location.local_frame.down)/3
        send_nav_velocity(0.0,0.0,v_z)
        if Vehicle._location.local_frame.down >= coord[2] * 0.95:
            send_nav_velocity(0,0,0)
            time.sleep(1.5)
            break
        time.sleep(1)


    #修正航向
    y=coord[1]-Vehicle._location.local_frame.east
    x=coord[0]-Vehicle._location.local_frame.north
    r=math.atan(y/x)
    r=math.degrees(r)
    condition_yaw(r,0,1)


    #修正三向速度
    long_x=0.0
    long_y=0.0
    long_z=0.0
    long_x= (coord[0]-Vehicle.location.local_frame.north) ** 2 
    long_y=(coord[1]-Vehicle._location.local_frame.east ) ** 2
    long_z= (coord[2]-Vehicle.location.local_frame.down )**2
    long_x=long_x ** (1/2)
    long_y=long_y ** (1/2)
    long_z=long_z ** (1/2)
    v_x=0.0
    v_y=0.0
    v_z=0.0
    v_x=long/30
    v_y=long/30
    v_z=long/30
    send_nav_velocity(v_x,v_y,v_z)






#解锁切换GUIDED模式（完成）
def unlock():
        #解锁切换模式
    Vehicle.mode = VehicleMode("GUIDED")
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not Vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    Vehicle.mode = VehicleMode("GUIDED")
    Vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not Vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    while not Vehicle.mode.name=='GUIDED' and not Vehicle.armed and not api.exit:
        print (" Getting ready to take off ...")
        Vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)





#定高起飞（cm）
def takeoff(altitude):

    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not Vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    Vehicle.mode = VehicleMode("GUIDED")
    Vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not Vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    Vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if Vehicle._location._down >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)







#判定有无障碍(完成)
def barrier():
    if safe_fo ==0 :
        pass
    elif safe_fo == 1 :
        while True:
            recorrect(coord_temp)
            if abs(Vehicle.location.local_frame.north-coord_temp[0]) <=0.05:
                if abs(Vehicle.location.local_frame.east-coord_temp[1]) <=0.05:
                    if abs(Vehicle.location.local_frame.down -coord_temp[2]) <=0.05:
                        break
            time.sleep(1)
    elif safe_fo == 2:
        pass



#调用舵机（未完成）
def put(order):
    pass

#返航(完成，不包含降落）
def back(coord):
   while True:
       recorrect(coord)
       barrier()
       if Vehicle._location.local_frame.north <= coord[0]*0.01:
            if Vehicle._location.local_frame.east <= coord[1]*0.01:
                   break
       time.sleep(1)


#降落（完成）
def down():
    print("Setting LAND mode...")
    while True:
        Vehicle.mode = VehicleMode("LAND")
        if Vehicle._location._down <= 0.01:
            break
        time.sleep(1)

#关机（完成）
def shutdown():
    Vehicle.disarmed = False
    print("Setting LAND mode...")
    Vehicle.mode = VehicleMode("LAND")
    time.sleep(1)

    # Close vehicle object before exiting script
    print("Close vehicle object")
    Vehicle.close()


#锁定（完成）
def lock():
    Vehicle.disarmed = False





#自动导航模块(完成）
def auto():
    
    #确保无人机处于安全高度
    while Vehicle.location.local_frame.down <=fly_hight_rec*0.9:
        recorrect
        print("等待无人机飞到指定高度")
        sleep(0.5)
        
    n=0#记录投放次数

    while True:
        i=0
        #更新坐标信息
        while i<3:
            coord_putdown_now[i]= coord_putdown[n*3+i]
            i=i+1
        coord_putdown_now[2]=coord_putdown_now[2]+0.1

        #第一个死循环
        while True:
            recorrect(coord_putdown_now) #姿态修正
            barrier() #判定是否有障碍
            if abs(Vehicle.location.local_frame.north - coord_putdown_now[0]) <= 0.01: #到达目的地
                if  abs(Vehicle.location.local_frame.east  - coord_putdown_now[1]) <= 0.01:
                    break
        #更新修正坐标信息
        coord_putdown_now[2]=put_hight_rec
        while True:
            recorrect(coord_putdown_now)
            if Vehicle.location.local_frame.down <=put_hight_rec:
                break
            if Vehicle.location.local_frame.down  <=0.01:
                break
        print("正在投放第%d号物料",n)
        put(n)#投放物料

        #修正高度
        coord_putdown_now[2]=fly_hight_rec
        print("正在回升高度")
        while Vehicle.location.local_frame.down  <= fly_hight_rec :
            recorrect(coord_putdown_now)

        n=n+1
        if n>= num:
            break
    print("全部投放完毕！")

    #读取返航
    if back_home == 0:
        pass
    elif back_home==1:
        coord=[0.0,0.0,0.1]
        back(coord)
    elif back_home == 2:
        back(coord_load)


    down()

    #反馈降落成功
    print("降落成功")

    

    lock()
    print("无人机已锁定")



        #主程序模块
while True:


    fly_hight_rec=input("请输入推荐飞行高度：")
    put_hight_rec=input("请输入推荐投放高度：")
    num =input("请输入投放点数量：")
    coord=[0.0,0.0,0.0]
    i=0
    coord_putdown[0]=input("请输入1号点位的x:")
    coord_putdown[1]=input("请输入1号点位的y:")
    coord_putdown[2]=input("请输入1号点位的z:")
    while True:
        coord[0]=input("请输入下一个点位的x:")
        coord[1]=input("请输入下一个点位的y:")
        coord[2]=input("请输入下一个点位的z:")

        i=i+1
        if i >= num-1:
            break

    while True:
        takeoff(50)
        auto()
        print("执行完毕")
        com=inport("是否再次运行(y)(小写）：")
        if com == y:
            com2=inport("是否保留之前的参数（y)(小写）：")
            if com2 == y :
                break
            else:
                pass
        else :
            print("关机")
            shutdown()







######