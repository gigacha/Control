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
import csv
from time import sleep
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyUSB0', 115200)

path_x = [0, 0, 0, 0];
path_y = [10, 20, 30, 40]
path_idx = 0;
look_ahead = 3
speed_lim = 10
k = 0.5
cur_speed_list =[]
break_list = []
i= 0
tmp =[]
cmd_list = []
sum_speed = 0
pre_mode = 0 

def serWrite(speed, steering, cnt, cur_mode, cur_speed):
    global i, sum_speed, pre_mode
    input_speed = speed
    
    break_val = 0x01
    if cur_mode ==2:
        break_val = 0x01
    elif cur_mode==1:
        speed*=0.3
    elif cur_mode==0:
        # break function code----------------
        speed=0
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
        
        

    # steering 값 2000 넘길 시 2000으로 설정
    if abs(steering)>2000:
        if steering>0:
            steering = 2000
        else :
            steering =-2000
    # 기어 기본값 0: 전진, 1:후진
    print('==serWrite==');
    print("speed     : ", speed)
    print("cur_speed : ", cur_speed)
    print("break_val : ", break_val)
    print("steering  : ", steering)
    print("cur_mode  : ", cur_mode)
    print("cnt       : ", cnt)
    result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, int(speed),
                steering, break_val, cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
    print("pc : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
      result[9], result[10], result[11], result[12], result[13] )
    
    i+=1
    tmp.append(i*0.0509144806)
    if speed == 0:
        cmd_list.append(0)
    else:
        cmd_list.append(100)

    if cur_mode == 0:
        sum_speed += 0.0509144806 * cur_speed /36
    
    if pre_mode - cur_mode ==2:
        # brk_idx = tmp[i]
        print(tmp[i-1])

    cur_speed_list.append(cur_speed)
    break_list.append(break_val)

    pre_mode = cur_mode

    ser.write(result)
    
    
def Y_angle(cur_x, cur_y, cur_yaw, target_x, target_y): # By 삼각함수 멋쟁이 예환
    #print("Y_angle", cur_x, cur_y, cur_yaw, target_x, target_y)
    yaw=radians(cur_yaw)
    th = cur_yaw - degrees(atan2((target_y - cur_y), (target_x - cur_x)))
    if abs(th)>180:
        if (th < 0) :
            th += 360
        else :
            th -= 360
    #print("Theta : ", th)
    if abs(th)>30:
        if th > 0:
            return 30
        else :
            return -30
    else :
        return int(th)

def cal_steering(cur_x, cur_y, cur_yaw, look_ahead):
    #print("cal_steering", cur_x, cur_y, cur_yaw)    
    global path_idx
    print("path_idx", path_idx)    
    target_x = path_x[path_idx]
    target_y = path_y[path_idx]
    dis = pow(pow(abs(target_x - cur_x),2) + pow(abs(target_y - cur_y),2), 0.5)
    print("distance", dis)
    if dis <= look_ahead:
        print("### HIT target!!! ###")
        path_idx += 1
        return cal_steering(cur_x, cur_y, cur_yaw)
    else : 
         return Y_angle(cur_x, cur_y, cur_yaw, target_x, target_y)

def cal_lookahead(vel):
    look_ahead = k*vel/36 + 2.2
    return look_ahead

# msg 수신시 호출되는 함수
def control_cmd(cur_x, cur_y, cur_yaw, cur_mode):
    print("======== control_cmd =========")
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
    
    print("len", len(result)) 
    if len(result) > 17:
        print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
        result[9], result[10], result[11], result[12], result[13], result[14], result[15], result[16], result[17])
        cnt = result[15]
        print(cnt)
        look_ahead = cal_lookahead(result[7])
        steering = cal_steering(cur_x, cur_y, cur_yaw, look_ahead);
        cur_speed = result[6]
        serWrite(int(speed_lim*10), int(steering*71), cnt, cur_mode, cur_speed)


    # cnt가 10일때 패킷이 잘려서 2번에 걸쳐 들어옴.+++++++++++++++++++++++++++++ 0x0a값이 아스키코드 LF(new line)!!!
    elif len(result) == 16:
        print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
        result[9], result[10], result[11], result[12], result[13], result[14], result[15])
        add_result = ser.readline() # erp -> pc
        cnt = result[15]
        print(cnt)        
        look_ahead = cal_lookahead(result[7])
        cur_speed = result[6]
        steering = cal_steering(cur_x, cur_y, cur_yaw, look_ahead);
        serWrite(int(speed_lim*10), int(steering*71), 10, cur_mode, cur_speed)
    # serWrite(int(speed_lim*10), int(steering*71), 10, cur_mode)
    # print("cur_speed : " , result[6])


def main(): 
    fig = plt.figure()
    plt.grid(True)
    while(True):
        start_t = time.time()
        while((time.time()-start_t)<5):
            control_cmd(0,0,0,0)
        start_t = time.time()  # 시작 시간 저장
        print('----------Run!----------')
        while((time.time()-start_t)<7):
            control_cmd(0,0,0,2)
        start_t = time.time()
        # cxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx 
        print('----------Break!----------')
        while((time.time()-start_t)<5):
            control_cmd(0,0,0,0)
        break

    print('break distance:', sum_speed)
    plt.plot(tmp, cmd_list)
    plt.plot(tmp, cur_speed_list)
    plt.plot(tmp, break_list)
    plt.title(speed_lim)
    plt.xlabel(sum_speed)
    plt.legend(['mode_change', 'cur_speed', 'break_val'])
    plt.show()



if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()