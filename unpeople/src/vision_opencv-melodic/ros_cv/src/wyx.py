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
 
class image_converter:
 
  global lines
  def __init__(self):
    self.count = 0
    self.image_pub = rospy.Publisher("image_topic_2",Image)
 
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
 

  def hsv_(self,img): #提取黄色后的 图片
    print ('hello')
    hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    print ('nihao')
    lower_hsv = np.array([11,43,150])
    upper_hsv = np.array([28,255,255])
    mask = cv2.inRange(hsv_img,lowerb=lower_hsv,upperb=upper_hsv)
    print ('38')
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
  def counts_select(self,contours,img):
#对提取出的点进行左右车道线分类
    point_list_l = []
    point_list_r = []
    for contour in contours:
        for pose in contour:
            x = pose[0][0]
            y = pose[0][1]
            point = (x,y)
            if y >= 170:
                if x <= 640:
                    point_list_l.append(point)
                else:
                    point_list_r.append(point)

    npl = np.mat(point_list_l)
    npr = np.mat(point_list_r)

    ll_ = np.zeros((720,1))
    rr_ = np.full((720,1),1279)
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
        if (rr_[y] == 1279):
            rr_[y] = x
        else:
            if rr_[y] < x:
                rr_[y] = rr_[y]
            else:
                rr_[y] = x
#将（170-720）行的点进行保存  
    point_deal_l = []
    point_deal_r= []
    for index,i in enumerate(ll_):
        if i[0] == 0:
            i[0] = (int)(i[0] / 5)
        pose = (int(i[0]),int(index))
        point_deal_l.append(pose)
    for index,i in enumerate(rr_):
        if i[0] == 1279:
            i[0] = (int)((i[0] + 5650) / 5)
        pose = (int(i[0]),int(index))
        point_deal_r.append(pose)



    for dian in point_deal_l:
        cv2.circle(img,dian,1,(0,255,255),1)
    for dian in point_deal_r:
        cv2.circle(img,dian,1,(0,0,255),1)
#得到中线
    mid = []
    for i in range(720):
        pose = ((int)((ll_[i] + rr_[i]) / 2),i)
        mid.append(pose)
#画出中线
    for dian in mid:
        cv2.circle(img,dian,1,(255,0,255),1)

#画出基准线
    reference_line = []
    actual_line = []
    mid_value = 0

    for i  in range(60):
        mid_value = mid_value + mid[250+i][0]
        pose = (640,300+i)
        reference_line.append(pose)
    mid_value = int(mid_value / 60)

    print (mid_value)
    for i in range(60):
        pose = (mid_value,300+i)
        actual_line.append(pose)

    for dian in reference_line:
        cv2.circle(img,dian,1,(0,255,0),1)
    for dian in actual_line:
        cv2.circle(img,dian,1,(150,255,0),1)
    error = mid_value - 640
    text = str(error)
    cv2.putText(img,text,(200,100),cv2.FONT_HERSHEY_COMPLEX,2.0,(255,255,200),5)
    cv2.imshow("final",img)
    return error





  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
 
    (rows,cols,channels) = cv_image.shape 
    str1 = './origin/'
    str2 = './img/'   
    path1 = str1 +  '%d'%(self.count) +'.jpg'
    path2 = str2 +  '%d'%(self.count) +'.jpg'
#    print (cv_image.shape)
    #if cols > 60 and rows > 60 :
     # cv2.circle(cv_image, (50,50), 10, 255)
#    cv2.imwrite(path1,cv_image)
    cv_image = cv2.GaussianBlur(cv_image,(5,5),0)
    img_cp = np.copy(cv_image)
    hsv_img = self.hsv_(cv_image)
#    cv2.imshow('yellow',hsv_img)

    edges = self.extract(hsv_img)
#    cv2.imshow('edges',edges)

    roi = np.array([[(0,hsv_img.shape[0]),(150,150),(1030,150),(hsv_img.shape[1],hsv_img.shape[0])]])
    roi_edges = self.roi_mask(edges,roi)
#    pic = self.hough(roi_edges)
    pic = roi_edges
    img11,contours,hier = cv2.findContours(pic,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    self.counts_select(contours,img_cp)
    cv2.imshow('lv',roi_edges)
#    cv2.imwrite(path2,roi_edges)
    self.count = self.count + 1	
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

