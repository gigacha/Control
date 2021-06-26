#!/usr/bin/env python
import pynmea2
import serial #Serial USB
import socket #UDP LAN
import sys
import time
import rospy
import numpy as np
# from master.msg import sync_msg
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
base_lat = 37.23896333333333  #37.2389619983  #37.23896  #37.3847120383
base_lon = 126.77297833333333  #126.772982033  #126.77298  #126.65599282
# base_alt =  0.9 #29.976  #0.9    #45.538  

# def get_xy(lat, lon, alt):
#     e, n, u = pymap3d.geodetic2enu(lat, lon, alt, base_lat, base_lon, base_alt)
#     return e, n

# msg_sync = sync_msg()
# ser = serial.Serial('/dev/pts/15',38400)

pub = rospy.Publisher('/sim_gps', Odometry, queue_size = 1)
rospy.init_node("sim_gps_node",anonymous=True)
rospy.loginfo("-------sim_gps_node start!-------")
rate = rospy.Rate(10)
# ser = serial.Serial('/dev/ttyUSB',38400) #Serial USB
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP LAN

recv_address = ('127.0.0.1', 3052)
# count_num = 0
# recv_address = ('192.168.10.100', 3051)

sock.bind(recv_address)

p = Odometry()

if __name__ == '__main__':
	# f= open("1111.txt",'w')
	# t_s = time.time()sg2.true_course
	# # pre_x = 0.0
	# # pre_y = 0.0
    while True:
        data, sender = sock.recvfrom(1024)  # udp
        lat = 0
        lon = 0
        heading_int = 0
        print('========================================')
        # count = count+1
        # data = ser.readline()  # serial
		# print(str(data[1:6]))
        if str(data[1:6]) == "b'GPRMC'":
            heading = data[49:54]
            heading = str(heading)[2:5]
            heading_int = int(heading)

			# print(data)
			# msg2 = pynmea2.parse(data)
			# heading = msg2.true_course
			# print(heading)

        if str(data[1:6]) == "b'GPGGA'":
            lat_str = str(data[16:25])
            lon_str = str(data[28:38])
            lat_int = 37 + float(lat_str[4:11])/60
            lon_int = 126 + float(lon_str[5:12])/60
            if lat_int != 0 :
                p.pose.pose.position.x = round(lon_int, 9)
                p.pose.pose.position.y = round(lat_int, 9)
        
            
        
        
        if heading_int != 0 :
            if(0 < heading_int and heading_int < 180) :
                heading_int += 180
            else : 
                heading_int -= 180
            p.pose.pose.position.z = heading_int
            
        print('heading: ', p.pose.pose.position.z)
        print('lat :',p.pose.pose.position.y)
        print('lon :',p.pose.pose.position.x)
        
        pub.publish(p)
		# RMC_msg =pynmea2.parse(data.split('\n')[0])
		# GGA_msg =pynmea2.parse(data.split('\n')[1])

		# lat = str(float(GGA_msg.lat[0:2]) + (float(GGA_msg.lat[2:9])/60))
		# lon = str(float(GGA_msg.lon[0:3]) + (float(GGA_msg.lon[3:10])/60))
		# heading = float(RMC_msg.true_course)
		# print(heading)
		# print("lat: {}, lon: {}, heading: {}".format(lat,lon,heading))

		# x, y = get_xy(float(lat), float(lon), GGA_msg.altitude)
		# vector = np.array([x,y])
		# vector_ini = np.array([1,0])
		# theta = (vector @ vector_ini) / ( (x**2 + y**2)**0.5)
		# theta = np.degrees(theta)
		# print("x is %f, y is %f, heading is %f, theta is %f" % (x, y, heading, theta))











		# if data[1:6] == 'GPRMC':
		# 	msg2 = pynmea2.parse(data)
		# 	heading = msg2.true_course
		# 	print(heading)
		
		# if data[1:6] == 'GPGGA':
		# 	msg = pynmea2.parse(data)
		# 	# print(msg)
		# 	lat = str(float(msg.lat[0:2]) + (float(msg.lat[2:9])/60))
		# 	lon = str(float(msg.lon[0:3]) + (float(msg.lon[3:10])/60))
		# 	# print("lat = " + str(lat) + "long = " + str(lon) + "alt"+str(msg.altitude))
		# 	x, y = get_xy(float(lat), float(lon), msg.altitude)
		# 	print("%f,%f" % (x, y))


			
		# print(data)
		# print(type(data))
		# print(msg)

	

	

		# lat = str(float(msg.lat[0:2]) + float(int(msg.lat[2:9])/60) + float(float(msg.lat[4:9])/36000000))
		# lat = str(float(msg.lat[0:2]) + (float(msg.lat[2:9])/60))
		# lon = str(float(msg.lon[0:3]) + (float(msg.lon[3:10])/60))
		#  print("lat = " + str(lat) + "long = " + str(lon) + "alt = " + str(msg.altitude))
		# x, y = get_xy(float(lat), float(lon), msg.altitude)
		# 
		# print("%f,%f" % (x, y))

		# d =((x-pre_x)**2 + (y-pre_y)**2)**0.5
		# print(d)

		# if d > 2:
		# 	f.write(str(x)+"\t"+ str(y)+"\n")
		# 	pre_x = x
		# 	pre_y = y
		# else:
		# 	continue
	

		
		# if (time.time()-t_s) > 0.1:
		# 	f.write(str(x)+"\t"+ str(y)+"\n")
		# 	t_s = time.time()

		# else:
		# 	continue
			
		
		

		