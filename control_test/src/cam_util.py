#!/usr/bin/env python
import cv2
import numpy as np
import os
import glob
import struct

def get_img(UDPSocket_cam, params_cam):

    '''
    receive a camera image
    \n UDPSocket_cam : UDP server socket
    \n params_cam : parameters from cameras 
    '''

    if params_cam["SOCKET_TYPE"] == 'JPG':

        # JPG/UDP type
        UnitBlockSize_cam = params_cam["Block_SIZE"]
        max_len = np.floor(params_cam["WIDTH"]*params_cam["HEIGHT"]/UnitBlockSize_cam/2)-1
        
        TotalBuffer = []
        num_block = 0

        while True:

            bytesAddressPair = UDPSocket_cam.recvfrom(UnitBlockSize_cam)
            UnitBlock = bytesAddressPair[0]
            
            UnitIdx = struct.unpack('i', UnitBlock[3:7])[0]

            UnitTail = UnitBlock[-2:]
            UnitSize = struct.unpack('i', UnitBlock[7:11])[0]
            
            UnitBody = np.frombuffer(UnitBlock[11:(11+UnitSize)], dtype = "uint8")
            
            if num_block == UnitIdx:
                TotalBuffer.append(UnitBody)
                num_block += 1
            else:
                TotalBuffer = []
                num_block = 0

            if UnitTail==b'EI' and len(TotalBuffer)>max_len:

                TotalIMG = cv2.imdecode(np.hstack(TotalBuffer), 1)

                break

    else:

        # RGB/UDP type

        TotalIMG = np.zeros((params_cam["HEIGHT"], params_cam["WIDTH"], 3), dtype = "uint8")
        img_head = np.zeros(int(params_cam["HEIGHT"]/params_cam["UnitBlock_HEIGHT"]),)
        UnitBlockSize_cam = int(params_cam["WIDTH"]*params_cam["UnitBlock_HEIGHT"]*3+8)

        while True:

            bytesAddressPair = UDPSocket_cam.recvfrom(UnitBlockSize_cam)
            UnitBlock = bytesAddressPair[0]

            UnitBlock_array = np.frombuffer(UnitBlock, dtype = "uint8")
            
            UnitHead = int(UnitBlock_array[0])
            UnitBody = UnitBlock_array[4:UnitBlockSize_cam-4].reshape(-1, params_cam["WIDTH"], 3)
            
            TotalIMG[UnitHead*params_cam["UnitBlock_HEIGHT"]:(UnitHead+1)*params_cam["UnitBlock_HEIGHT"],:,:] = UnitBody
            img_head[UnitHead] = 1
            
            if np.mean(img_head)>0.99:

                break

    return TotalIMG

def cam_calib(params_calib, TEST_IMAGE_PATHS):
    '''
    calibrate a camera 
    \n params_cam : parameters from cameras 
    '''
    
    corner_rows = params_calib["CORNER ROWS"]
    corner_cols = params_calib["CORNER COLS"]

    #termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 150, 0.001)

    #prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(CORNER ROWS-1,CORNER COLS-1,0)
    objp = np.zeros((corner_cols*corner_rows,3), np.float32)
    objp[:,:2] = np.mgrid[0:corner_rows,0:corner_cols].T.reshape(-1,2)
    
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    for image_path in TEST_IMAGE_PATHS:

        xyz_img = cv2.imread(image_path)
        h, w = xyz_img.shape[:2]
        xyz_gray = cv2.cvtColor(xyz_img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(xyz_gray, (corner_rows, corner_cols), None)

        if ret == True:
            objpoints.append(objp)

            cv2.cornerSubPix(xyz_gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)

            #draw and display the corners

            if params_calib["VIEW CHECKBOARD CALIB"]:
                cv2.drawChessboardCorners(xyz_img, (corner_rows, corner_cols), corners, ret)
                cv2.imshow(image_path, cv2.resize(xyz_img, (w, h),
                                                    interpolation=cv2.INTER_LINEAR))
                cv2.waitKey(0)
                cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, xyz_gray.shape[::-1],None,None)

    return ret, mtx, dist, rvecs, tvecs


def cam_optimalmatrix(mtx, dist, w, h):
    '''
    get the optimal camera matrix  
    \n mtx : instric camera matrix
    \n dist : distortion coefficient
    \n w,h : width and height of images 
    '''

    newcameramtx, roi =cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    return newcameramtx, roi


def undistort_cam_img(img, mtx, dist, newcameramtx, roi, crop=False):
    '''
    get the optimal camera matrix  
    \n img : camera image array
    \n mtx : instric camera matrix
    \n newcameramtx : optimal calibrated camera matrix
    \n roi : range of valid image
    \n crop : crop the invalid part of the undistorted image 
    '''
    
    # undistort
    undist = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # crop the image
    if crop:
        x,y,w,h = roi
        undist = undist[y:y+h, x:x+w]
    
    return undist