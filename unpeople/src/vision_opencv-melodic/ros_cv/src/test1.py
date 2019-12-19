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
 
  def __init__(self):
    self.count = 0
    self.image_pub = rospy.Publisher("image_topic_2",Image)
 
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
 


#cv2.drawContours(img, contours, -1, (255,0,0), 1)
   # cv2.imshow("findcontour",img)
   # cv2.waitKey()
   # cv2.destroyAllWindows()

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
    print (cv_image.shape)
    #if cols > 60 and rows > 60 :
     # cv2.circle(cv_image, (50,50), 10, 255)
    cv2.imwrite(path1,cv_image)
    hsv_img = self.hsv_(cv_image)
    cv2.imshow('yellow',hsv_img)

    edges = self.extract(hsv_img)
    cv2.imshow('edges',edges)

    roi = np.array([[(0,hsv_img.shape[0]),(350,150),(930,150),(hsv_img.shape[1],hsv_img.shape[0])]])
    roi_edges = self.roi_mask(edges,roi)
    cv2.imshow('lv',roi_edges)
    cv2.imwrite(path2,roi_edges)
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

