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
import heapq
class image_converter:
 
  global lines
  def __init__(self):
    self.pitch = 0
    self.yaw = 0
    self.count = 0
    self.last_error = 0
    self.bmx_flag = 0
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.pub = rospy.Publisher('/cv/error',String)   #发布话题
    self.sub_imu = rospy.Subscriber('/imu_data',Imu,self.imu_callback)
    self.bridge = CvBridge()
    self.a = cv2.imread('./100.png')
    self.h,self.w =self.a.shape[:2]
    self.pts = np.float32([[50,100],[263,100],[0,180],[319,180]])
    self.pts1 = np.float32([[0,0],[self.w-1,0],[0,self.h-1],[self.w-1,self.h-1]])
    self.m = cv2.getPerspectiveTransform(self.pts,self.pts1)
  #  self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)
 
  def rgb_hls_lab(self,img):
    img_lab = cv2.cvtColor(img,cv2.COLOR_BGR2LAB) #提取黄色   b通道
    img_hls = cv2.cvtColor(img,cv2.COLOR_BGR2HLS) #提取白色   l通道
    imghls_h = img_hls[:,:,0]
    imghls_l = img_hls[:,:,1]
    imghls_s = img_hls[:,:,2]
    #gray = cv2.equalizeHist(gray)

    imglab_l = img_lab[:,:,0]
    imglab_a = img_lab[:,:,1]
    imglab_b = img_lab[:,:,2]

    retval_lab,dst_lab = cv2.threshold(imglab_b,170,255,cv2.THRESH_BINARY)
    retval_hls,dst_hls = cv2.threshold(imghls_l,180,255,cv2.THRESH_BINARY)
    
  #  imglab_b = cv2.equalizeHist(imglab_b)
  #  imghls_l = cv2.equalizeHist(imghls_l)
    #hls_lab = np.zeros_like(dst_lab)
   # hls_lab = dst_lab + dst_hls 
   # cv2.imshow('b',dst_lab)
   # cv2.imshow('l',dst_hls)
    #cv2.imshow('b',imglab_b)
    count = np.sum(dst_hls) / 255
    print ('white_count',np.sum(dst_hls) / 255)

    cv2.imshow('l',imghls_l)
    cv2.imshow('l_',dst_hls)
    if count > 17000:
        self.bmx_flag = 1
    else:
        self.bmx_flag = 0
    #cv2.imshow('hls_lab',hls_lab)
    return count  
  def extract(self,img): #找到距离中心线最近的4个轮廓
    img_height = 180 
    img_width = 320

    blur = cv2.GaussianBlur(img,(3,3),0,0)
    gray = cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY)
    gray_cp = gray[59:179,:]
    mid = np.median(gray_cp)
    min_thresh = (int)(mid * 0.66)
    max_thresh = (int)(mid * 1.33)
    #gray = cv2.equalizeHist(gray)
    #edges = cv2.Canny(gray,50,200)
    edges = cv2.Canny(gray,min_thresh,max_thresh)
    temp = np.ones(edges.shape,np.uint8)*255  #白色幕布
    cv2.rectangle(edges,(0,0),(319,60),0,-1)
    cv2.imshow('qiege',edges)

    h = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    contours = h[1]
   # if (len(contours) > 20):
   #     list_ = []
   #     self.bmx_flag = 1
   #     return list_,temp
   # elif (len(contours) < 8):
   #     self.bmx_flag = 0
    

    contours = [contour for contour in contours  if ((abs)(cv2.fitLine(contour,cv2.DIST_L2,0.2,0.01,0.01)[1]/cv2.fitLine(contour,cv2.DIST_L2,0.2,0.01,0.01)[0]) > 0.5 and len(contour) > 55)]
    
    point_listr = []
    point_listr_index = []
    point_listl = []
    point_listl_index = []
    point_list = []
    point_list_index = []
    point_listk = []
    point_listk_index = []

    for contour in contours:
      #  cv2.drawContours(temp,contour,-1,(0,255,0),cv2.FILLED)
        [vx,vy,x_,y_] = cv2.fitLine(contour,cv2.DIST_L2,0.2,0.01,0.01)
        pose_x,pose_y,pose_w,pose_h = cv2.boundingRect(contour)
        distance =  (((pose_x+pose_x+pose_w) / 2 - 180))
        length = len(contour)
        k = pose_y
        #print ('dis',distance)
        point_list_index.append(length)
        point_list.append(contour)
        point_listk_index.append(k)
        point_listk.append(contour)
        if distance >= 0:
            point_listr_index.append(distance)
            point_listr.append(contour)
        else:
            point_listl_index.append(distance)
            point_listl.append(contour)

    find_countl = 0
    find_countr = 0
    find_count = 0
    find_countk = 0
    if len(contours) <= 3:
        find_countl = len(contours)
        find_countr = len(contours)
        find_count = len(contours)
        find_countk = len(contours)
    else:
        find_countr = 2 
        find_countl = 2
        find_count = 3
        find_countk = 1
         
    dis_min = (heapq.nlargest(find_count,point_list_index))
    dis_minl = (heapq.nlargest(find_countl,point_listl_index))
    dis_minr = (heapq.nsmallest(find_countr,point_listr_index))
    dis_mink = (heapq.nsmallest(find_countk,point_listk_index))
    deal_contours = []
    for i in dis_minl:
        index = point_listl_index.index(i)
        deal_contours.append(point_listl[index])
    for i in dis_minr:
        index = point_listr_index.index(i)
        deal_contours.append(point_listr[index])
    for i in dis_min:
        index = point_list_index.index(i)
        deal_contours.append(point_list[index])
    for i in dis_mink:
        index = point_listk_index.index(i)
        deal_contours.append(point_list[index])

   # contours = [contour for contour in contours if (len(contour) > 400 and len(contour) < 800)]   #滤除过小或过大的轮廓线
    count = 0
    for contour in deal_contours:
        cv2.drawContours(temp,contour,-1,(0,255,0),cv2.FILLED)
        count = count + 1
    
    cv2.imshow('contours',temp)
    
    return deal_contours,temp 

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
  def transform(self,img,transform_param):   #透视变换
    dst = cv2.warpPerspective(img,transform_param,(320,180))
    cv2.imshow('dst',dst)
    return dst
  def counts_select(self,contours,img_ed,img):    #中线提取
    img_cp = np.copy(img_ed)  #副本图像
    img_height = 180
    img_width = 320

    point_list_l = []
    point_list_r = []
    contour_position = 'l'
    ll_q = 0
    rr_q = 0
   # print ('start',len(contours))
    for contour in contours:
        rows,cols = img_cp.shape[:2]
        [vx,vy,x_,y_] = cv2.fitLine(contour,cv2.DIST_L2,0.2,0.01,0.01)
        pose_x,pose_y,pose_w,pose_h = cv2.boundingRect(contour)
    #    print ('width,k,mid',pose_w,vy/vx,(pose_x+pose_w+pose_x)/2)
        if (pose_w >= 75 and vy/vx < 0) or (pose_w < 75 and (pose_x + pose_w + pose_x) / 2 <= img_width / 2):
            contour_position = 'l'
            ll_q = ll_q + pose_w
        elif (pose_w >= 75 and vy/vx > 0) or (pose_w <75 and (pose_x + pose_w + pose_x) / 2 >= img_width / 2 ):
            contour_position = 'r'
            rr_q = rr_q + pose_w 
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
   # print ('end')
    print ('ll_q',ll_q)
    print ('rr_q',rr_q)
    npl = np.mat(point_list_l)
    npr = np.mat(point_list_r)

    ll_ = np.zeros((img_height,1))
    rr_ = np.full((img_height,1),img_width - 1)
    width = np.full((img_height,1),0)
    for i in range (img_height):
        l = 40
        r = 280
        wid = r - l
        width[i] = wid

#把轮廓上杂乱的点 提取成左右线（一行只有一个点）
    for pose in npl:
        pose = pose.A  #必须要将矩阵转化为数组 
        if (pose.shape[1] != 2):
            continue
        x = pose[0][0]
        y = pose[0][1]
        if (y > img_height - 120):
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
        if (y > img_height - 120):
            if (rr_[y] == img_width-1):
                rr_[y] = x
            else:
                if rr_[y] < x:
                    rr_[y] = rr_[y]
                else:
                    rr_[y] = x

    point_deal_l = []
    point_deal_r= []
    for index,i in enumerate(ll_):
        if (index <= 60):
            continue
        if (i[0] == 0):
            if(rr_[index] == img_width - 1):
                i[0] = 25
                ll_[index] = 25
            else:
                i[0] = rr_[index] - width[index]
                ll_[index] = rr_[index] - width[index]
        pose = (int(i[0]),int(index))
        point_deal_l.append(pose)
    for index,i in enumerate(rr_):
        if (index <= 60):
            continue
        if (i[0] == img_width - 1):
            if(ll_[index] == 0):
                i[0] = 295
                rr_[index]  = 295
            else:
                i[0] = ll_[index] + width[index]
                rr_[index] = ll_[index] + width[index]
        pose = (int(i[0]),int(index))
        point_deal_r.append(pose)
    [vxl,vyl,xl_,yl_] = cv2.fitLine(np.array(point_deal_l),cv2.DIST_L2,0.2,0.01,0.01)
    [vxr,vyr,xr_,yr_] = cv2.fitLine(np.array(point_deal_r),cv2.DIST_L2,0.2,0.01,0.01)
    kl = vyl/vxl
    kr = vyr/vxr
   
    ''' 
    if (kl*kr > 0):
        if ((kl < 1.5 or kr < 1.5) and kl > 0 and kl > 0):
            if (kl > kr):
                point_deal_l = []
                for index,pose in enumerate(point_deal_r):
                    x = pose[0]
                    y = pose[1]
                    x = x - width[index]   #临时赋值
                    rr_[y] = x
                    pose_ = (x,y)
                    point_deal_l.append(pose_)
            elif (kl < kr):
                point_deal_r = []
                for index,pose in enumerate(point_deal_l):
                    x = pose[0]
                    y = pose[1]
                    ll_[y] = x
                    pose_ = (x,y)
                    point_deal_r.append(pose_)
                point_deal_l = []
                for index,pose in enumerate(point_deal_r):
                    x = pose[0] - width[index]
                    y = pose[1]
                    rr_[y] = x
                    pose_ = (x,y)
                    point_deal_l.append(pose_)
    posel_x,posel_y,posel_w,posel_h = cv2.boundingRect(np.array(point_deal_l))
    poser_x,poser_y,poser_w,poser_h = cv2.boundingRect(np.array(point_deal_r))
    img_box = cv2.rectangle(img,(posel_x,posel_y),(posel_x+posel_w,posel_y+posel_h),(0,255,0),2) #画出图像轮廓
    img_box = cv2.rectangle(img,(poser_x,poser_y),(poser_x+poser_w,poser_y+poser_h),(0,255,0),2) #画出图像轮廓
    '''
    for y,i in enumerate(ll_):
        if index < 60:
            continue
        if((rr_[y] < ll_[y]) or ((abs)(rr_[y] - ll_[y]) < 40)):
            if(rr_q - ll_q > 125):
                ll_[y] = rr_[y] - width[y]
            elif(ll_q - rr_q > 125):
                rr_[y] = ll_[y] + width[y]
    print ('kl',vyl/vxl)
    print ('kr',vyr/vxr)

    for index,i in enumerate(rr_):
        pose = ((int)(i[0]),int(index))
        cv2.circle(img,pose,1,(255,0,0),1)
    for index,i in enumerate(ll_):
        pose = (int(i[0]),int(index))
        cv2.circle(img,pose,1,(255,0,255),1)
#得到中线
    mid = []
    for i in range(img_height):
        pose = ((int)((ll_[i] + rr_[i]) / 2),i)
        mid.append(pose)
#出中线
    for dian in mid:
        cv2.circle(img,dian,1,(255,0,255),1)

#画出基准线
    reference_line = []
    actual_line = []
    mid_value = 0
    start_raw = 80
#    print ('pitch:')
#    print (self.pitch) 
    if (self.pitch > 6 or self.pitch < -3):
        start_raw = 80 
    else:
        start_raw = 80
    text_pitch = str(self.pitch)
    text_yaw = str(self.yaw)
    bmx_flag = str(self.bmx_flag)
    text_raw = str(start_raw)
    cv2.putText(img,text_pitch,(250,50),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,255,200),2)
    cv2.putText(img,bmx_flag,(50,100),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,255,200),2)
    cv2.putText(img,text_yaw,(50,50),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,0,200),2)
    cv2.putText(img,text_raw,(250,100),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,255,200),2)
    for i  in range(10):  #计算60行的中值 求平均
        mid_value = mid_value + mid[start_raw+i][0]
        pose = (img_width/2,start_raw+i)
        reference_line.append(pose)
    mid_value = (int)(mid_value / 10)


    print ('mid_value',mid_value)
    for i in range(15):
        pose = (mid_value,75+i)
        actual_line.append(pose)

    for dian in reference_line:
        cv2.circle(img,dian,1,(0,255,0),1)
    for dian in actual_line:
        cv2.circle(img,dian,1,(150,255,0),1)
    error = mid_value - (img_width / 2) + (ll_q - rr_q) *  0.1
   # if (abs)(error - self.last_error) > 250:
   #     error = self.last_error
    if (self.bmx_flag == 1):
       error = -self.imu_data_deal() * 10 
       #pass
    #self.last_error = error

    text_error = str(error)
    cv2.putText(img,text_error,(100,50),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,255,200),5)
    cv2.imshow("final",img)
    return error 

  def imu_callback(self,data):
     self.pitch = data.orientation.y 
     self.yaw = data.orientation.z 

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    start = time.time()
    img_cp = np.copy(cv_image)
    cv2.imshow('one',cv_image)
    
    transform_img = self.transform(cv_image,self.m)  #透视变换后的图像
    img_hl = self.rgb_hls_lab(transform_img)
    contours,edges = self.extract(transform_img)     #边缘提取后的轮廓和图像
    #contours,edges = self.extract(bgr_img)     #边缘提取后的轮廓和图像
   
    error = self.counts_select(contours,edges,transform_img)    #中线提取 最终得到偏差值
    #error = 0
    end = time.time()
    print ('cost time',end-start)
    count = 0
    if count >= 7:
        str_zerba = 1
    else:
        str_zerba = 0
    str_error = "%d"%(error)   #将数字转换为字符串
    str_msg = str_error + " " + str(str_zerba)
    self.pub.publish(str_msg)     #发布偏差话题
    
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

