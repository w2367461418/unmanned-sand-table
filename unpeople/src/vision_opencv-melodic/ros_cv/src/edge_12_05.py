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
    self.count = 0
    self.last_error = 0
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.pub = rospy.Publisher('/cv/error',String)   #发布话题
    self.sub_imu = rospy.Subscriber('/imu_data',Imu,self.imu_callback)
    self.bridge = CvBridge()
  #  self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)
 

  def hsv_yellow(self,img): #提取黄色后的 图片
    hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lower_hsv = np.array([11,43,120])
    upper_hsv = np.array([28,255,240])
    mask = cv2.inRange(hsv_img,lowerb=lower_hsv,upperb=upper_hsv)
    return mask
  def hsv_white(self,img): #提取黄色后的 图片
    hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lower_hsv = np.array([0,0,221])
    upper_hsv = np.array([180,30,255])
    mask = cv2.inRange(hsv_img,lowerb=lower_hsv,upperb=upper_hsv)
    return mask
  def roi_mask(self,edges,vertices):
    mask = np.zeros_like(edges)
    mask_color = 255
    cv2.fillPoly(mask, vertices, mask_color)
    masked_img = cv2.bitwise_and(edges, mask)
    return masked_img
  def extract(self,img): #经过canny边缘提取后的图片
    blur_gray = cv2.GaussianBlur(img,(5,5),0,0)
    edges = cv2.Canny(blur_gray,50,150)
    return edges

  def hough(self,img): #获取图片中的线条    
    rho = 1
    theta = np.pi / 180 
    threshold = 10
    min_line_len = 10
    max_line_gap = 50
    lines = cv2.HoughLinesP(img,rho,theta,threshold,maxLineGap=max_line_gap)
    line_img = np.zeros_like(img)
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(line_img,(x1,y1),(x2,y2),255,1)
	cv2.imshow('line_img',line_img)
	return line_img
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
  def counts_select(self,contours,img):
    img_box = np.copy(img)  #副本图像
    cv2.imshow('otigin',img_box)
    img_height = 360
    img_width = 640
    start = time.time()
#对提取出的点进行左右车道线分类
    point_list_l = []
    point_list_r = []
    contour_position = 'l'
    for contour in contours:
        rows,cols = img_box.shape[:2]
        [vx,vy,x_,y_] = cv2.fitLine(contour,cv2.DIST_L2,0.2,0.01,0.01)
   #     print ("k:")
        if (contour.shape[0] < 40):
            continue
        pose_x,pose_y,pose_w,pose_h = cv2.boundingRect(contour)
        img_box = cv2.rectangle(img_box,(pose_x,pose_y),(pose_x+pose_w,pose_y+pose_h),(0,255,0),2) #画出图像轮廓
        if (((pose_x + pose_w + pose_x) / 2 ) < (img_width / 2 + 20) and (vy/vx) < 0):
            contour_position = 'l'
        elif ( ((pose_x + pose_w + pose_x) / 2) > (img_width / 2 - 20) and (vy/vx) > 0):
            contour_position = 'r'
        else:
            continue
        for pose in contour:
            x = pose[0][0]
            y = pose[0][1]
            point = (x,y)
            if y >= (int)(img_height / 5):
               # if x <= img_width / 2:
                if contour_position == 'l':
                    point_list_l.append(point)
                else:
                    point_list_r.append(point)
    npl = np.mat(point_list_l)
    npr = np.mat(point_list_r)

    ll_ = np.zeros((img_height,1))
    rr_ = np.full((img_height,1),img_width - 1)
    width = np.full((img_height,1),0)
    for i in range (img_height):
        l = int(-(i-360) / 5.0)
        r = int((i+2840) / 5.0)
        wid = r - l
        width[i] = wid

#把轮廓上杂乱的点 提取成左右线（一行只有一个点）
    for pose in npl:
        pose = pose.A  #必须要将矩阵转化为数组 
        if (pose.shape[1] != 2):
            continue
        x = pose[0][0]
        y = pose[0][1]
        if (ll_[y] == 0):
            ll_[y] = x
        else:
            if ll_[y] > x:
                ll_[y] = ll_[y]
            else:
                ll_[y] = x
        
    for pose in npr:
        pose = pose.A  #必须要将矩阵转化为数组 
        if (pose.shape[1] != 2):
            continue
        x = pose[0][0]
        y = pose[0][1]
        if (rr_[y] == img_width-1):
            rr_[y] = x
        else:
            if rr_[y] < x:
                rr_[y] = rr_[y]
            else:
                rr_[y] = x

#    cv2.putText(img,count_l,(200,600),cv2.FONT_HERSHEY_COMPLEX,2.0,(255,255,200),5)
#    cv2.putText(img,count_r,(1000,600),cv2.FONT_HERSHEY_COMPLEX,2.0,(255,255,200),5)

#将（170-720）行的点进行保存  
    point_deal_l = []
    point_deal_r= []
    for index,i in enumerate(ll_):
        if(i[0] == 0):
            if (rr_[index] == img_width -1):
                i[0] = (int)(-(index - 360) / 5.0)
            else:
                i[0] = rr_[index] - width[index]
        pose = (int(i[0]),int(index))
        point_deal_l.append(pose)
    for index,i in enumerate(rr_):
        if(i[0] == img_width - 1):
            if (ll_[index] == 0):
                i[0] = (int)((index + 2768) / 5.0)
            else:
                i[0] = ll_[index] + width[index]
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
    if (self.pitch > 3 or self.pitch < -3):
        start_raw = 200
    else:
        start_raw = 160
    text_pitch = str(self.pitch)
    cv2.putText(img,text_pitch,(500,100),cv2.FONT_HERSHEY_COMPLEX,2.0,(255,255,200),2)
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
    error = mid_value - (img_width / 2)
    if (abs)(error - self.last_error) > 350:
        error = self.last_error
    text_error = str(error)
    cv2.putText(img,text_error,(200,100),cv2.FONT_HERSHEY_COMPLEX,2.0,(255,255,200),5)
    cv2.imshow("final",img)
   # cv2.imshow("box",img_box)
    self.last_error = error
    end = time.time()
    print ('cost time',end-start)
    return error


 

  def imu_callback(self,data):
#    try:
     self.pitch = data.orientation.y 
#    print ('imu')
#    print (self.pitch)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
    cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
    cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
    img_cp = np.copy(cv_image)

    count = self.zebra_line(img_cp)
    print ("line count :",count)

    hsv_img = self.hsv_yellow(cv_image)
#    cv2.imshow('yellow',hsv_img)

    edges = self.extract(hsv_img)
#    cv2.imshow('edges',edges)

    roi = np.array([[(0,hsv_img.shape[0]),(75,150),(565,150),(hsv_img.shape[1],hsv_img.shape[0])]])
    roi_edges = self.roi_mask(edges,roi)
#    pic = self.hough(roi_edges)
    pic = roi_edges
    img11,contours,hier = cv2.findContours(pic,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    error = self.counts_select(contours,img_cp)  #最终得到偏差值
    if count >= 7:
        str_zerba = 1
    else:
        str_zerba = 0
    str_error = "%d"%(error)   #将数字转换为字符串
    str_msg = str_error + " " + str(str_zerba)
    self.pub.publish(str_msg)     #发布偏差话题
    
#    cv2.imshow('lv',roi_edges)
    cv2.imshow("first",cv_image) 
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

