#!/usr/bin/env python
import socket
import threading
import time
import struct
class erp_udp_parser :
    def __init__(self,ip,port,data_type):
        
        self.data_type=data_type
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (ip,port)
        self.sock.bind(recv_address)
        self.data_size=65535 
        self.parsed_data=[]
        thread = threading.Thread(target=self.recv_udp_data)
        thread.daemon = True 
        thread.start() 

    

    def recv_udp_data(self):
        while True :
            raw_data, sender = self.sock.recvfrom(self.data_size)
            self.data_parsing(raw_data)




    def data_parsing(self,raw_data) :
        if self.data_type == 'status' :
            header=raw_data[0:9].decode()
            data_length=struct.unpack('i',raw_data[9:13])

            if header == '#ERPInfo$' and data_length[0] ==32:
                
                unpacked_data=struct.unpack('ffffffff',raw_data[25:57])
                self.parsed_data=list(unpacked_data)
           
            
        elif self.data_type == 'obj' :
            
            header=raw_data[0:12].decode()
     
            if header == '#ERPObjInfo$' :
                unpacked_data=[]
                offset_byte=28
                
                for i in range(20) :
                    start_byte=i*34
                    obj_type=struct.unpack('h',raw_data[start_byte+offset_byte:start_byte+offset_byte+2])
                    obj_info=struct.unpack('8f',raw_data[start_byte+offset_byte+2:start_byte+offset_byte+34])
                    obj_info_list=list(obj_info)
                    obj_info_list.insert(0,obj_type[0])
                    if not(obj_info_list[0] == 0 and obj_info_list[1] == 0 and obj_info_list[2] == 0) :
                        unpacked_data.append(obj_info_list)
                    
                    # print(i,obj_type[0])
                # print(raw_data)
                # print(len(unpacked_data))
                if len(obj_info_list) !=0 :
                    self.parsed_data=unpacked_data
                else :
                    self.parsed_data=[]

                
       
                

    def get_data(self) :
        return self.parsed_data

    def __del__(self):
        self.sock.close()
        print('del')


class erp_udp_sender :
    def __init__(self,ip,port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ip=ip
        self.port=port

        header='#ERPCtrlCmd$'.encode()
        data_length=struct.pack('i',12)
        aux_data=struct.pack('iii',0,0,0)
        self.upper=header+data_length+aux_data
        self.tail='\r\n'.encode()
        
        

    def send_data(self,accel,brake,steering_angle):
        
        packed_accel=struct.pack('f',accel)
        packed_brake=struct.pack('f',brake)
        packed_steering_angle=struct.pack('f',steering_angle)
        lower=packed_accel+packed_brake+packed_steering_angle
        send_data=self.upper+lower+self.tail
        
        self.sock.sendto(send_data,(self.ip,self.port))


        
        

      

