# -*- coding: utf-8 -*-
import numpy as np
import cv2 as cv
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

cap = cv.VideoCapture("test_1.mp4")
hsv = 0
h=[]
s=[]
v=[]
Ycrcb=0
Y=[]
cb=[]
cr=[]
hsv_a = 0
h_a=[]
s_a=[]
v_a=[]
Ycrcb_a=0
Y_a=[]
cb_a=[]
cr_a=[]
hsv_w=0
h_w=[]
s_w=[]
v_w=[]
Ycrcb_w=0
Y_w=[]
cr_w=[]
cb_w=[]

def mouse_callback(event, x, y, flags, param):
    global hsv,h,s,v,Ycrcb,Y,cb,cr,hsv_a,h_a,s_a,v_a,Ycrcb_a,Y_a,cb_a,cr_a
    # 마우스 왼쪽 버튼 누를시 위치에 있는 픽셀값을 읽어와서 HSV로 변환합니다.

    if event == cv.EVENT_LBUTTONDOWN:
        print(img_color[y, x])
        color = img_color[y, x]
        one_pixel = np.uint8([[color]])
        hsv = cv.cvtColor(one_pixel, cv.COLOR_BGR2HSV)
        Ycrcb = cv.cvtColor(one_pixel,cv.COLOR_BGR2YCrCb)
        h.append(hsv[0][0][0])
        s.append(hsv[0][0][1])
        v.append(hsv[0][0][2])
        Y.append(Ycrcb[0][0][0])
        cr.append(Ycrcb[0][0][1])
        cb.append(Ycrcb[0][0][2])
    if event ==cv.EVENT_RBUTTONDOWN:
        print(img_color[y, x])
        color = img_color[y, x]
        one_pixel = np.uint8([[color]])
        hsv_a = cv.cvtColor(one_pixel, cv.COLOR_BGR2HSV)
        Ycrcb_a = cv.cvtColor(one_pixel, cv.COLOR_BGR2YCrCb)
        h_a.append(hsv_a[0][0][0])
        s_a.append(hsv_a[0][0][1])
        v_a.append(hsv_a[0][0][2])
        Y_a.append(Ycrcb_a[0][0][0])
        cr_a.append(Ycrcb_a[0][0][1])
        cb_a.append(Ycrcb_a[0][0][2])
    if event ==cv.EVENT_MOUSEWHEEL:
        print(img_color[y, x])
        color = img_color[y, x]
        one_pixel = np.uint8([[color]])
        hsv_w = cv.cvtColor(one_pixel, cv.COLOR_BGR2HSV)
        Ycrcb_w = cv.cvtColor(one_pixel, cv.COLOR_BGR2YCrCb)
        h_w.append(hsv_w[0][0][0])
        s_w.append(hsv_w[0][0][1])
        v_w.append(hsv_w[0][0][2])
        Y_w.append(Ycrcb_w[0][0][0])
        cr_w.append(Ycrcb_w[0][0][1])
        cb_w.append(Ycrcb_w[0][0][2])

cv.namedWindow('img_color')
cv.setMouseCallback('img_color', mouse_callback)

fig = plt.figure()
fig2 = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax2 = fig2.add_subplot(111, projection='3d')

while True:
    _,img_color= cap.read()

    img_color = cv.resize(img_color, (1240,620), interpolation=cv.INTER_AREA)
    cv.imshow('img_color', img_color)
    if cv.waitKey(0) & 0xFF == ord('w'):
        break
    cv.waitKey(0)

Y.sort()
cr.sort()
cb.sort()
Y_w.sort()
cr_w.sort()
cb_w.sort()
h.sort()
s.sort()
v.sort()
h_w.sort()
s_w.sort()
v_w.sort()
print('Yellow Lane Threshold value:\n', (h[int(len(h)*0.1)]-10),'<H','<',h[int(len(h)*0.9)])
print((s[int(len(h)*0.1)]-10), '<S','<', s[int(len(s)*0.9)])
print((v[int(len(h)*0.1)]-10),'<V', '<',v[int(len(s)*0.9)])

#print('White Lane Threshold value:\n',(Y_w[int(len(Y)*0.1)]-10),'<Y','<',Y_w[int(len(Y_w)*0.9)])
#print((cr_w[int(len(cr_w)*0.1)]-10), '<Cr','<', cr_w[int(len(cr_w)*0.9)])
#print((cb_w[int(len(cb_w)*0.1)]-10), '<Cb','<', cb_w[int(len(cb_w)*0.9)])

ax.scatter(h,s,v,c='Orange',label='Yellow Lane', marker='o', s=15, cmap='Greens')

ax.scatter(h_a, s_a, v_a,c='black',label='Asphalt', marker='o', s=15, cmap='Greens')
ax.scatter(h_w, s_w, v_w,c='blue', label='White Lane',marker='o', s=15, cmap='Greens')
ax.set_xlabel('H channel')
ax.set_ylabel('S channel')
ax.set_zlabel('V channel')
plt.suptitle('COLOR EXTRACTION OF LANE-ASPHALT WITH HSV')

ax2.scatter(Y,cb,cr,c='Orange', marker='o', s=15, cmap='Greens')
ax2.scatter(Y_a, cb_a, cr_a,c='black', marker='o', s=15, cmap='Greens')
ax2.scatter(Y_w, cb_w, cr_w,c='blue', marker='o', s=15, cmap='Greens')
ax2.set_xlabel('Y channel')
ax2.set_ylabel('Cr channel')
ax2.set_zlabel('Cb channel')

plt.suptitle('COLOR EXTRACTION OF LANE-ASPHALT WITH YCrCb')
cv.destroyAllWindows()
plt.show()