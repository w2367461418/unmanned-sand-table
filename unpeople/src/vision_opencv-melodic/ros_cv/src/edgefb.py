#!/usr/bin/env python
#-*-coding:utf-8-*
from __future__ import print_function
 
import roslib
roslib.load_manifest('ros_cv')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import time
class image_converter:
 
  global lines
  def __init__(self):
    self.pitch = 0
    self.yaw = 0
    self.count = 0
    self.last_error = 0
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.pub = rospy.Publisher('/cv/error',String)   #发布话题
    self.sub_imu = rospy.Subscriber('/imu_data',Imu,self.imu_callback)
    self.bridge = CvBridge()
    self.a = cv2.imread('./1_.jpg')
    self.h,self.w =self.a.shape[:2]
    self.pts = np.float32([[115,150],[515,150],[10,280],[605,270]])
    self.pts1 = np.float32([[0,0],[self.w-1,0],[0,self.h-1],[self.w-1,self.h-1]])
    self.m = cv2.getPerspectiveTransform(self.pts,self.pts1)
  #  self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)
 

  def hsv_yellow(self,img): #提取黄色后的 图片
    hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    cv2.imshow('yellow',hsv_img)
    lower_hsv = np.array([15,60,100])
    upper_hsv = np.array([34,255,255])
    mask = cv2.inRange(hsv_img,lowerb=lower_hsv,upperb=upper_hsv)
    return mask
  def hsv_white(self,img): #提取黄色后的 图片
    hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lower_hsv = np.array([0,0,221])
    upper_hsv = np.array([180,30,255])
    mask = cv2.inRange(hsv_img,lowerb=lower_hsv,upperb=upper_hsv)
    return mask
  def rgb_hls_lab(self,img):
    img_lab = cv2.cvtColor(img,cv2.COLOR_BGR2LAB) #提取黄色   b通道
    img_hls = cv2.cvtColor(img,cv2.COLOR_BGR2HLS) #提取白色   l通道
    imghls_h = img_hls[:,:,0]
    imghls_l = img_hls[:,:,1]
    imghls_s = img_hls[:,:,2]

    imglab_l = img_lab[:,:,0]
    imglab_a = img_lab[:,:,1]
    imglab_b = img_lab[:,:,2]

    retval_lab,dst_lab = cv2.threshold(imglab_b,150,255,cv2.THRESH_BINARY)
    retval_hls,dst_hls = cv2.threshold(imghls_l,180,255,cv2.THRESH_BINARY)

    hls_lab = np.zeros_like(dst_lab)
    hls_lab = dst_lab + dst_hls 
    cv2.imshow('b',dst_lab)
    cv2.imshow('l',dst_hls)
#    cv2.imshow('lab',imglab_b)
#    cv2.imshow('hls',imghls_l)
    cv2.imshow('hls_lab',hls_lab)
    return hls_lab
  def roi_mask(self,edges,vertices):
    mask = np.zeros_like(edges)
    mask_color = 255
    cv2.fillPoly(mask, vertices, mask_color)
    masked_img = cv2.bitwise_and(edges, mask)
    return masked_img
  def extract(self,img): #经过canny边缘提取后的图片
    blur = cv2.GaussianBlur(img,(5,5),0,0)
    blur = cv2.GaussianBlur(blur,(5,5),0,0)
    blur = cv2.GaussianBlur(blur,(5,5),0,0)
    gray = cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY)
   # edges = cv2.Canny(ygrad,xgrad,50,150)
    edges = cv2.Canny(gray,50,150)
    cv2.rectangle(edges,(0,0),(639,120),0,-1)
    cv2.imshow('qiege',edges)

    
    #print (xgrad.shape)
    #dst = cv2.addWeighted(threshx,0.5,threshy,0.5,0)
    #ret,thresh = cv2.threshold(dst,120,255,cv2.THRESH_BINARY)
    #cv2.imshow('x',thresh)
    #cv2.imshow('y',ygrad)
    h = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    contours = h[1]
    #contours = [cv2.approxPolyDP(cnt,1,True) for cnt in contours]
    temp = np.ones(edges.shape,np.uint8)*255
    for contour in contours:
        if(len(contour) > 400 and len(contour) < 800):
            cv2.drawContours(temp,contour,-1,(0,255,0),cv2.FILLED)
   # ret = cv2.findContours(temp,cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    count = 0
    for contour in contours:
        count = count + 1
        print (len(contour))
    print ('len',count)
    cv2.imshow('contours',temp)
    
    return  0

  def hough(self,img): #获取图片中的线条    
    rho = 1
    theta = np.pi / 180 
    threshold = 250
    min_line_len = 70
    max_line_gap = 50
    lines = cv2.HoughLinesP(img,rho,theta,threshold,maxLineGap=max_line_gap)
    line_img = np.zeros_like(img)
    print (len(lines))
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(line_img,(x1,y1),(x2,y2),255,1)
	cv2.imshow('line_img',line_img)
	return line_img
  def imu_data_deal(self):
    error = 0
    percent =  (abs)(self.yaw) / 360 
    yaw = abs(self.yaw)   
    if (percent <= 0.125 or percent > 0.875):
        if (percent <= 0.125):
            error = yaw - 0
        elif (percent > 0.875):
            error = yaw - 360 
    elif (percent > 0.125 and percent <= 0.375 ):
        error = yaw - 90
    elif (percent > 0.375 and percent <= 0.625):
        error = yaw - 180
    elif (percent > 0.625 and percent <= 0.875):
        error = yaw - 270
    return error    
#斑马线识别
  def zebra_line(self,img):
    img_cp = np.copy(img)
    
    zebra_img = self.hsv_white(img_cp)
    print (zebra_img.shape)
    zebra_img[0:170,0:639] = 0
#    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#    ret,thresh = cv2.threshold(img,200,255,cv2.THRESH_BINARY)
    image,contours,hierarchy = cv2.findContours(zebra_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    count = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 1800:
            count = count + 1
            x,y,w,h = cv2.boundingRect(contour)
            img_cp = cv2.rectangle(img_cp,(x,y),(x+w,y+h),(255,255,0),2)
        else:
            continue
    zebra = str(count)
    if zebra == 0:
        pass
    else:
        cv2.putText(img_cp,zebra,(400,100),cv2.FONT_HERSHEY_COMPLEX,2.0,(255,255,200),5)
    cv2.imshow('bmx',img_cp)
    return count
  def transform(self,img,transform_param):

    dst = cv2.warpPerspective(img,transform_param,(640,360))
    cv2.imshow('dst',dst)
    return dst
  def counts_select(self,img_ed,img):
    img_cp = np.copy(img_ed)  #副本图像
    img_height = 360
    img_width = 640

    ll_ = np.zeros((img_height,1))
    rr_ = np.full((img_height,1),img_width - 1)
    width = np.full((img_height,1),0)
    
    start = time.time()
    '''
    for i in range(img_height-5):
        row = img_height - i - 1
        for j in range((int)((img_width - 10)/2)):
            line_l = (int)(img_width / 2 - j - 1)
            if(img_cp[row][line_l] + (img_cp[row][line_l - 1]) == 255):
                ll_[row] = line_l - 2
                break
            else:
                continue
        for j in range((int)((img_width - 10)/2)):
            line_r = (int)(img_width/2 + j + 1)
            if(img_cp[row][line_r] +(img_cp[row][line_r + 1]) == 255 ):
                rr_[row] = line_l + 2
                break
            else:
                continue
        if row < 150:
            break
    '''

    end = time.time()
    print ('time',start-end)
    point_deal_l = []
    point_deal_r= []
    for index,i in enumerate(ll_):
        pose = (int(i[0]),int(index))
        point_deal_l.append(pose)
    for index,i in enumerate(rr_):
        pose = (int(i[0]),int(index))
        point_deal_r.append(pose)



    for dian in point_deal_r:
        cv2.circle(img,dian,1,(255,0,0),1)
    for dian in point_deal_l:
        cv2.circle(img,dian,1,(125,0,255),1)
#得到中线
    mid = []
    for i in range(img_height):
        pose = ((int)((ll_[i] + rr_[i]) / 2),i)
        mid.append(pose)
#画出中线
    for dian in mid:
        cv2.circle(img,dian,1,(255,0,255),1)

#画出基准线
    reference_line = []
    actual_line = []
    mid_value = 0
    start_raw = 160
#    print ('pitch:')
#    print (self.pitch) 
    if (self.pitch > 6 or self.pitch < -3):
        start_raw = 230
    else:
        start_raw = 160
    text_pitch = str(self.pitch)
    text_yaw = str(self.yaw)
    cv2.putText(img,text_pitch,(500,100),cv2.FONT_HERSHEY_COMPLEX,2.0,(255,255,200),2)
    cv2.putText(img,text_yaw,(100,100),cv2.FONT_HERSHEY_COMPLEX,2.0,(255,0,200),2)
    text_raw = str(start_raw)
    cv2.putText(img,text_raw,(500,200),cv2.FONT_HERSHEY_COMPLEX,2.0,(255,255,200),2)
    for i  in range(20):  #计算60行的中值 求平均
        mid_value = mid_value + mid[start_raw+i][0]
        pose = (img_width/2,start_raw+i)
        reference_line.append(pose)
    mid_value = (int)(mid_value / 20)

#    k = 0
#    for i  in range(340):
#        k = (mid[718-i-1][0] - mid[718-i][0]) + k

    print (mid_value)
    for i in range(30):
        pose = (mid_value,150+i)
        actual_line.append(pose)

    for dian in reference_line:
        cv2.circle(img,dian,1,(0,255,0),1)
    for dian in actual_line:
        cv2.circle(img,dian,1,(150,255,0),1)
    error = mid_value - (img_width / 2 - 3)
    if (abs)(error - self.last_error) > 350:
        error = self.last_error
   # cv2.imshow("box",img_box)
    self.last_error = error

    text_error = str(error)
    cv2.putText(img,text_error,(200,100),cv2.FONT_HERSHEY_COMPLEX,2.0,(255,255,200),5)
    cv2.imshow("final",img)
    return 0 

  def imu_callback(self,data):
#    try:
     self.pitch = data.orientation.y 
     self.yaw = data.orientation.z 
#    print ('imu')
#    print (self.pitch)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    start = time.time()
    #cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
    #cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
    #cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
    img_cp = np.copy(cv_image)
    
    transform_img = self.transform(cv_image,self.m)
  #  count = self.zebra_line(img_cp)
  #  hsv_img = self.hsv_yellow(cv_image)
  #  edges = self.extract(hsv_img)
  #  hsv_img = self.rgb_hls_lab(transform_img)
    edges = self.extract(transform_img)
  #  cv2.imshow('edges',edges)
  #  self.hough(edges)
  #  roi = np.array([[(0,hsv_img.shape[0]),(75,150),(565,150),(hsv_img.shape[1],hsv_img.shape[0])]])
  #  roi_edges = self.roi_mask(edges,roi)
#    pic = self.hough(roi_edges)
  #  pic = roi_edges
    pic = edges
    #img11,contours,hier = cv2.findContours(pic,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    #error = self.counts_select(pic,transform_img)  #最终得到偏差值
    error = 0
    end = time.time()
    #print ('cost time',end-start)
    count = 0
    if count >= 7:
        str_zerba = 1
    else:
        str_zerba = 0
    str_error = "%d"%(error)   #将数字转换为字符串
    str_msg = str_error + " " + str(str_zerba)
    self.pub.publish(str_msg)     #发布偏差话题
    
#    cv2.imshow("first",cv_image)
    
    cv2.waitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    
    
def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)

