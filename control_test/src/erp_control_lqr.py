#!/usr/bin/env python
# -*-coding: utf-8-*-
import serial
import time 
import struct
import numpy as np
from math import radians
from math import degrees
from math import cos
from math import sin
from math import asin
from math import pow
from math import atan2
import csv
from time import sleep
import rospy
from nav_msgs.msg import Odometry
ser = serial.Serial('/dev/ttyUSB0', 115200)
# path_x = [ 0, 0, 0, 10];
# path_y = [ 10, 20, 30, 40 ];  
path_x = []
path_y = []
path_yaw = []
path_k =[]
speed_lim = 6

Q = np.eye(4)
R = np.eye(1)
Q[0][0]=130
Q[1][1]=5
Q[2][2]=130
Q[3][3]=5
R[0][0]=1000

e, th_e = 0.0, 0.0

base_lat = 37.3837028
base_lon = 126.653281

def serWrite(speed, steering, cnt, cur_mode):

    break_val = 0x01
    if cur_mode ==2:
        break_val = 0x01
    elif cur_mode==1:
        speed*=0.3
    elif cur_mode==3:
        speed=0
        break_val=0x10
    elif cur_mode==0:
        speed=0
        break_val=0x600
    print('speed : ', speed);
    print("ser_write", speed, steering, cnt, cur_mode)
    # steering 값 2000 넘길 시 2000으로 설정
    if abs(steering)>2000:
        if steering>0:
            steering = 2000
        else :
            steering =-2000
    # 기어 기본값 0: 전진, 1:후진
    print("speed ", speed, "steering ", steering, "cnt : ", cnt)
    result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, int(speed),
                steering, break_val, cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
    print("pc : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
      result[9], result[10], result[11], result[12], result[13] )
    ser.write(result)
    

def solve_DARE(A, B, Q, R):
    """
    solve a discrete time_Algebraic Riccati equation (DARE)
    """
    X = Q
    maxiter = 150
    eps = 0.01

    for i in range(maxiter):
        Xn = A.T @ X @ A - A.T @ X @ B @ \
            la.inv(R + B.T @ X @ B) @ B.T @ X @ A + Q
        if (abs(Xn - X)).max() < eps:
            break
        X = Xn

    return Xn


def dlqr(A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # ref Bertsekas, p.151
    """

    # first, try to solve the ricatti equation
    X = solve_DARE(A, B, Q, R)

    # compute the LQR gain
    K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

    return K

def lqr_steering_control(pe, pth_e, v):
    ind, e = calc_nearest_index(path_x, path_y, path_yaw)

    k = path_k[ind]
    th_e = cur_yaw - path_yaw[ind]

    A = np.zeros((4, 4))
    A[0, 0] = 1.0
    A[0, 1] = dt
    A[1, 2] = max(10,v)
    A[2, 2] = 1.0
    A[2, 3] = dt
    # print(A)

    B = np.zeros((4, 1))
    B[3, 0] = v / L

    K = dlqr(A, B, Q, R)

    x = np.zeros((4, 1))

    x[0, 0] = e
    x[1, 0] = (e - pe) / dt
    x[2, 0] = th_e
    x[3, 0] = (th_e - pth_e) / dt

    ff = math.atan2(L * k, 1)
    fb = math.degrees((-K @ x)[0, 0])

    delta = ff + fb

    return delta, e, th_e

def calc_nearest_index(path_x, path_y, path_yaw):
    dx = [cur_x - icx for icx in path_x]
    dy = [cur_y - icy for icy in path_y]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind)

    mind = math.sqrt(mind)

    dxl = path_x[ind] - cur_x
    dyl = path_y[ind] - cur_y

    #뒤돌아 있는 경우 

    angle = pi_2_pi(path_yaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind

# msg 수신시 호출되는 함수
def getOdoMsg(msg):
    print("======== get a odo msg =========")
    cur_x    = msg.pose.pose.position.x * 0.1
    cur_y    = msg.pose.pose.position.y * 0.1
    cur_yaw  = msg.pose.pose.orientation.x
    cur_mode = msg.pose.pose.orientation.y
    print(cur_yaw)

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
        steering, e, th_e =lqr_steering_control(e, th_e, result[6])
        serWrite(int(speed_lim*10), int(steering*71), cnt, cur_mode)
    # cnt가 10일때 패킷이 잘려서 2번에 걸쳐 들어옴.+++++++++++++++++++++++++++++ 0x0a값이 아스키코드 LF(new line)!!!
    elif len(result) == 16:
        print("erp : ", result[0], result[1], result[2], result[3], result[4], result[5], result[6], result[7], result[8],
        result[9], result[10], result[11], result[12], result[13], result[14], result[15])
        add_result = ser.readline() # erp -> pc
        print(cnt)        
        steering, e, th_e =lqr_steering_control(e, th_e,  result[6])
        serWrite(int(speed_lim*10), int(steering*71), 10, cur_mode)
    # serWrite(int(speed_lim*10), int(steering*71), 10, cur_mode)
    

def main(): 
    with open('/home/wego/catkin_ws/src/integration/scripts/songdo_waypoint_.csv', mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for next_r in csv_reader:
            path_x.append(float(next_r['X'])*0.1)
            path_y.append(float(next_r['Y'])*0.1)
            path_k.append(float(next_r['K'])*10)
            path_yaw.append(float(next_r['yaw']))

            print("local_x ",float(next_r['X'])*0.1)
            print("local_y ",float(next_r['Y'])*0.1)

    rospy.init_node('control_node', disable_signals=False)
    rospy.loginfo("-------control node start!-------")
    rospy.Subscriber("/pub_odo", Odometry, getOdoMsg)
    rospy.loginfo("-------start spin-------")
    rospy.spin()
    rospy.loginfo("-------finish spin-------")
    
 
if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()