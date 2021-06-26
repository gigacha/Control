#!/usr/bin/env python
# -*-coding: utf-8-*-
import serial
import time 
import struct
from math import radians
from math import degrees
from math import cos
from math import sin
from math import asin
from math import pow
from math import atan2
import matplotlib.pyplot as plt
import csv
from time import sleep
import rospy
from nav_msgs.msg import Odometry
ser = serial.Serial('/dev/ttyUSB2', 115200)
# path_x = [ 0, 0, 0, 10];
# path_y = [ 10, 20, 30, 40 ];  
path_x = [ ]
path_y = [ ]
path_k = [ ]
path_idx = 0
look_ahead = 3
speed_lim = 6
first_chk = True
k = 0.2
i = 0
tmp = []
cte_list = []
cur_speed_list = []


base_lat = 37.3837028
base_lon = 126.653281

def serWrite(speed, steering, cnt, cur_mode, time_interval):
    global i
    cur_mode = 2
    input_speed = speed
    break_val = 0x01
    if cur_mode ==2:
        break_val = 0x01
    elif cur_mode==1:
        speed*=0.2
        break_val = 10
    elif cur_mode==3:
        speed=0
        break_val=0x10
    elif cur_mode==0:
        speed=0
        # break_val=0x60
        if input_speed <70:
            break_val = input_speed - int(0.4 *cur_speed)
            if break_val <= 0:
                break_val = 1
        if input_speed <100:
            break_val = input_speed + 10 - int(0.3 *cur_speed)
            if break_val <= 0:
                break_val = 1
        else:
            break_val = input_speed + 20 - int(0.3*cur_speed)
            if break_val <= 0:
                break_val = 1
        
    print('speed : ', speed);
    print("ser_write", speed, steering, cnt, cur_mode)
    # steering 값 2000 넘길 시 2000으로 설정
    if abs(steering)>2000:
        if steering>0:
            steering = 2000
        else :
            steering =-2000

    i += 1
    tmp.append(i*time_interval)
    _,cte = calc_nearest_index()
    cte_list.append(cte)
    cur_speed_list.append(cur_speed)


    # 기어 기본값 0: 전진, 1:후진
    print("speed ", speed, "steering ", steering, "cnt : ", cnt)
    result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, int(speed),
                steering, break_val, cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
    print("pc : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
      result[9], result[10], result[11], result[12], result[13] )
    ser.write(result)
    
    
def Y_angle(cur_x, cur_y, cur_yaw, target_x, target_y): # By 삼각함수 멋쟁이 예환
    print("Y_angle", cur_x, cur_y, cur_yaw, target_x, target_y)
    yaw=radians(cur_yaw)
    th = cur_yaw - degrees(atan2((target_y - cur_y), (target_x - cur_x)))
    delta = degrees(atan2(2*1.1*sin(radians(th))/look_ahead,1))
    if abs(delta)>180:
        if (delta < 0) :
            delta += 360
        else :
            delta -= 360

    print("Theta : ", delta)
    if abs(delta)>30:
        if delta > 0:
            return 30
        else :
            return -30
    else :
        return int(delta)

def cal_steering(cur_x, cur_y, cur_yaw, look_ahead):
    print("cal_steering", cur_x, cur_y, cur_yaw)    
    global path_idx
    print("path_idx", path_idx)    
    target_x = path_x[path_idx]
    target_y = path_y[path_idx]
    dis = pow(pow(abs(target_x - cur_x),2) + pow(abs(target_y - cur_y),2), 0.5)
    print("distance", dis)
    if dis <= look_ahead:
        print("### HIT target!!! ###")
        path_idx += 1
        return cal_steering(cur_x, cur_y, cur_yaw, look_ahead)
    else : 
         return Y_angle(cur_x, cur_y, cur_yaw, target_x, target_y)

def cal_lookahead(vel):
    # look_ahead = k*vel/36 + 3.0
    look_ahead = 3.3
    return look_ahead

# msg 수신시 호출되는 함수
def getOdoMsg(msg):
    start_time = time.time()
    global first_chk
    global path_idx
    print("======== get a odo msg =========")
    cur_x    = msg.pose.pose.position.x * 0.1
    cur_y    = msg.pose.pose.position.y * 0.1
    cur_yaw  = msg.pose.pose.orientation.x
    cur_mode = msg.pose.pose.orientation.y
    print(cur_yaw)
    
    if(first_chk):
        find_first_pathidx(cur_x, cur_y, cur_yaw)

    if cur_yaw > 90 and cur_yaw < 270:
        cur_yaw = -(cur_yaw-90);
    else :
        if cur_yaw < 90:
            cur_yaw = (90-cur_yaw);
        else :
            cur_yaw = 360-cur_yaw +90     

    cnt=0x00;
    #print('a')
    result = ser.readline() # erp -> pc
    #print(result)   
    if path_idx != len(path_k):
        if abs(path_k[path_idx]) > 0.002 :
            if cur_mode == 2:
                cur_mode = 1

    print("len", len(result)) 
    if len(result) > 17:
        print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
        result[9], result[10], result[11], result[12], result[13], result[14], result[15], result[16], result[17])
        cnt = result[15]
        print(cnt)
        look_ahead = cal_lookahead(result[6])
        steering = cal_steering(cur_x, cur_y, cur_yaw, look_ahead);

        time_interval = time.time() - start_time

        serWrite(int(speed_lim*10), int(steering*71), cnt, cur_mode, time_interval)
        start_time = time.time()
    # cnt가 10일때 패킷이 잘려서 2번에 걸쳐 들어옴.+++++++++++++++++++++++++++++ 0x0a값이 아스키코드 LF(new line)!!!
    elif len(result) == 16:
        print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
        result[9], result[10], result[11], result[12], result[13], result[14], result[15])
        add_result = ser.readline() # erp -> pc
        print(cnt)        
        look_ahead = cal_lookahead(result[6])
        steering = cal_steering(cur_x, cur_y, cur_yaw, look_ahead);

        time_interval = time.time() - start_time

        serWrite(int(speed_lim*10), int(steering*71), 10, cur_mode, time_interval)
        start_time = time.time()
    # serWrite(int(speed_lim*10), int(steering*71), 10, cur_mode)
    print("cur_speed : " , result[7])

def find_first_pathidx(cur_x, cur_y, cur_yaw):
    global path_idx
    global first_chk
    min_idx = 0
    min_dis = 9999999999
    size = len(path_x)
    for i in range(size):
        x = path_x[i]
        y = path_y[i]
        dis = (x-cur_x)**2 + (y-cur_y)**2
        if(min_dis > dis):
            min_dis = dis
            min_idx = i
    print("fin_dis : ", min_dis)
    print("fin_idx : ", min_idx)
    path_idx = min_idx
    first_chk = False
    yaw = Y_angle(cur_x, cur_y, cur_yaw, path_x[min_idx], path_y[min_idx])
    if(abs(yaw)>90):
        path_idx += 1

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def calc_nearest_index():
    dx = [cur_x - icx for icx in path_x]
    dy = [cur_y - icy for icy in path_y]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)
    ind = d.index(mind)
    mind = math.sqrt(mind)

    dxl = path_x[ind] - cur_x
    dyl = path_y[ind] - cur_y

    angle = pi_2_pi(path_yaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def main(): 
    with open('/home/wego/catkin_ws/src/integration/scripts/path.csv', mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for next_r in csv_reader:
            # path_x.append(int(((float(next_r['X']) - float(base_lon)) * 1000000))*0.1)
            # path_y.append(int(((float(next_r['Y']) - float(base_lat)) * 1000000))*0.1)
            path_x.append(float(next_r['X'])*0.1)
            path_y.append(float(next_r['Y'])*0.1)
            path_k.append(float(next_r['K']))
            # print("local_x ",int(((float(next_r['X']) - float(base_lon)) * 1000000))*0.1)
            # print("local_y ",int(((float(next_r['Y']) - float(base_lat)) * 1000000))*0.1)
            # print("local_x ",float(next_r['X']))
            # print("local_y ",float(next_r['Y']))
    plt.figure()

    rospy.init_node('control_node', disable_signals=False)
    rospy.loginfo("-------control node start!-------")
    rospy.Subscriber("/pub_odo", Odometry, getOdoMsg)
    rospy.loginfo("-------start spin-------")
    rospy.spin()
    rospy.loginfo("-------finish spin-------")
    
    plt.plot(tmp, cte_list)
    plt.plot(tmp, cur_speed_list)
    plt.grid(True)
    plt.title("input speed: " + str(speed_lim) + "km/h")

if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()