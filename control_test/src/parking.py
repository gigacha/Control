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

cur_speed_test = 0

path_x = [0, 0, 0, 0];
path_y = [10, 20, 30, 40];
path_idx = 0;
look_ahead = 3
speed_lim =15
k = 0.5
cur_speed_list =[]
break_list = []
i= 0
tmp =[]
cmd_list = []

def serWrite(speed, steering, cnt, cur_mode, cur_speed, back):
    global i
    break_val = 0x01
    if cur_mode ==2:
        break_val = 0x01
    elif cur_mode==1:
        speed*=0.3
    elif cur_mode==0:
        # break function code----------------
        speed=0
        # if cur_speed%2==1:
        #     cur_speed -= 1
        break_val = 100 - int(0.5*cur_speed)
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
    result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, back, int(speed),
                steering, break_val, cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
    print("pc : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
      result[9], result[10], result[11], result[12], result[13] )
    
    i+=1
    tmp.append(i*0.0509144806)
    if speed == 0:
        cmd_list.append(0)
    else:
        cmd_list.append(100)
    cur_speed_list.append(cur_speed)
    break_list.append(break_val)

    ser.write(result)
    
def send_packet(steering, speed, break_val, gear):
    result = ser.readline() # erp -> pc
    steering = (steering*71)
    if abs(steering)>2000:
        if steering>0:
            steering = 2000
        else :
            steering =-2000
    speed *= 10
    if len(result) >= 15: 
        cnt = result[15]
        result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, gear, int(speed),
                    int(steering), int(break_val), cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
        # print(cnt, result, steering, speed)
        ser.write(result)
    sleep(0.1)


# msg 수신시 호출되는 함수
def control_cmd(steering, speed , back, cur_mode):
    print("======== control_cmd =========")
    cnt=0x00;
    #print('a')
    global cur_speed_test
    result = ser.readline() # erp -> pc
    #print(result)   
    print("len", len(result)) 
    if len(result) > 17:
        print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
        result[9], result[10], result[11], result[12], result[13], result[14], result[15], result[16], result[17])
        cnt = result[15]
        print(cnt)
        cur_speed = result[6]
        cur_speed_test = result[6]
        serWrite(int(speed*10), int(steering*71), cnt, cur_mode, cur_speed, back)
    # cnt가 10일때 패킷이 잘려서 2번에 걸쳐 들어옴.+++++++++++++++++++++++++++++ 0x0a값이 아스키코드 LF(new line)!!!
    elif len(result) == 16:
        print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
        result[9], result[10], result[11], result[12], result[13], result[14], result[15])
        add_result = ser.readline() # erp -> pc
        cnt = result[15]
        print(cnt)        
        cur_speed = result[6]
        cur_speed_test = result[6]
        serWrite(int(speed*10), int(steering*71), 10, cur_mode, cur_speed, back)
    # serWrite(int(speed_lim*10), int(steering*71), 10, cur_mode)
    # print("cur_speed : " , result[6])



def main(): 
    fig = plt.figure()
    plt.grid(True)
    global cur_speed_test

    while(True):
        start_t = time.time()# 시작 시간 저장
        while((time.time()-start_t)<2):
            send_packet(0,0, 50, 0)  
        start_t = time.time()# 시작 시간 저장
        while((time.time()-start_t)<5.5):
            send_packet(30, 7, 0, 0)  
        start_t = time.time()
        while((time.time()-start_t)<1.5):
            send_packet(0, 2, 0, 0)
        start_t = time.time()
        while((time.time()-start_t)<5.5):
            send_packet(-30,7, 0, 0)
        start_t = time.time()
        while((time.time()-start_t)<1):
            send_packet(0, 5, 0, 0)
        #left
        '''
        start_t = time.time()# 시작 시간 저장
        while((time.time()-start_t)<5):
            send_packet(0,0, 50, 0)  
        start_t = time.time()# 시작 시간 저장
        while((time.time()-start_t)<3):
            send_packet(-30, 11, 0, 0)  
        start_t = time.time()
        while((time.time()-start_t)<0):
            send_packet(0, 5, 0, 0)
        start_t = time.time()
        while((time.time()-start_t)<2.3):
            send_packet(30, 11, 0, 0)
        start_t = time.time()
        while((time.time()-start_t)<1):
            send_packet(0, 5, 0, 0)
        '''

        # cxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx 
        print('----------Break!----------')
        while((time.time()-start_t)<5):
            control_cmd(0,0,0,0)

        break

    plt.plot(tmp, cmd_list)
    plt.plot(tmp, cur_speed_list)
    plt.plot(tmp, break_list)
    plt.legend(['mode_change', 'cur_speed', 'break_val'])
    plt.show()
    '''
    fig = plt.figure()
    plt.grid(True)
    while(True):
        start_t = time.time()# 시작 시간 저장
        while((time.time()-start_t)<5):
            control_cmd(0,0,0,0)

        start_t = time.time()
        while((time.time()-start_t)<6):
            control_cmd(30,7,0,2)

        start_t = time.time()
        while((time.time()-start_t)<2):
           control_cmd(0,2,0,2)

        start_t = time.time()
        while((time.time()-start_t)<5):
           control_cmd(0,0,0,0)

        start_t = time.time()
        while((time.time()-start_t)<3):
           control_cmd(0,6,2,2)

        start_t = time.time()
        while((time.time()-start_t)<3.5):
            control_cmd(30,7,2,2)  
        print('----------Run!----------')

        start_t = time.time()
        while((time.time()-start_t)<2):
            control_cmd(0,0,0,0)  
        print('----------Run!----------')

        start_t = time.time()
        while((time.time()-start_t)<5):
           control_cmd(0,5,0,2)


        # cxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx 
        print('----------Break!----------')
        while((time.time()-start_t)<5):
            control_cmd(0,0,0,0)

        break

    plt.plot(tmp, cmd_list)
    plt.plot(tmp, cur_speed_list)
    plt.plot(tmp, break_list)
    plt.legend(['mode_change', 'cur_speed', 'break_val'])
    plt.show()
    '''


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()