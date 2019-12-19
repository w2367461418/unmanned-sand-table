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
from threading import Timer
class image_converter:
 
  global lines
  def __init__(self):
    self.start = 0
    self.pitch = 0
    self.yaw = 0
    self.po_dao = 0
    self.count = 0
    self.last_error = 0
    self.pd_ping_flag  = 0
    self.last_rr =  np.zeros((179,1))
    self.last_ll =  np.zeros((179,1))
    self.bmx_flag = 0
    self.last_img = np.empty([180,320])
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
 
  def clean_flag(self):
    self.bmx_flag = 0
    self.po_dao = 0
    self.pd_ping_flag = 0
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
    print (dst_hls.shape[0])
    print (dst_hls.shape[1])
    count = np.sum(dst_hls) / 255
    print ('white_count',count)

    cv2.imshow('l',imghls_l)
    cv2.imshow('l_',dst_hls)
  #  if count > 17000 and self.bmx_flag == 0:
   #     self.bmx_flag = 1
    #    t = Timer(2.0,self.clean_flag)
    #    t.start()
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
    contours = [contour for contour in contours  if ((abs)(cv2.fitLine(contour,cv2.DIST_L2,0.2,0.01,0.01)[1]/cv2.fitLine(contour,cv2.DIST_L2,0.2,0.01,0.01)[0]) > 0.45 and len(contour) > 100)]
    contours_bmx = [contour for contour in contours if len(contour) > 200]
    print ('wai',len(contours))
    if self.pitch > 25 and len(contours) >= 12 and self.pd_ping_flag == 0:
        self.pd_ping_flag = 1
        t = Timer(2.0,self.clean_flag)
        t.start()
    print ('bmx',len(contours_bmx))
    if (len(contours_bmx) >= 9 and self.bmx_flag == 0 and self.pitch < 8):
        self.bmx_flag = 1
        t = Timer(2.0,self.clean_flag)
        t.start()
    cv2.drawContours(temp,contours,-1,(0,255,0),cv2.FILLED)
    cv2.imshow('contours',temp)
    return contours,temp 

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
    point_listr = []
    point_listr_index = []
    point_listl = []
    point_listl_index = []
    count_l = 0
    count_r = 0
    for contour in contours:
        rows,cols = img_cp.shape[:2]
        [vx,vy,x_,y_] = cv2.fitLine(contour,cv2.DIST_L2,0.2,0.01,0.01)
        pose_x,pose_y,pose_w,pose_h = cv2.boundingRect(contour)
        if (pose_w >= 25 and vy / vx < 0):
            count_l = count_l + 1
        elif (pose_w >= 25 and vy / vx > 0):
            count_r = count_r + 1

        if (pose_w >= 50 and vy/vx < 0) or (pose_w < 50 and (pose_x + pose_w + pose_x) / 2 <= img_width / 2):
            contour_position = 'l'
            ll_q = ll_q + pose_w
            point_listl_index.append(pose_x)
            point_listl.append(contour)

        if (pose_w >= 50 and vy/vx > 0) or (pose_w < 50 and (pose_x + pose_w + pose_x) / 2 >= img_width / 2 ):
            contour_position = 'r'
            rr_q = rr_q + pose_w 
            point_listr_index.append(pose_x)
            point_listr.append(contour)
        else:
            continue

    print ('l,r',count_l,count_r)
    find_countl = 0
    find_countr = 0
    if len(contours) <= 1:
        find_countl = len(contours)
        find_countr = len(contours)
    else:
        find_countr = 1 
        find_countl = 1
    dis_minr = (heapq.nsmallest(find_countr,point_listr_index))
    dis_minl = (heapq.nlargest(find_countl,point_listl_index))
    deal_contoursl = []
    deal_contoursr = []
    for i in dis_minl:
        index = point_listl_index.index(i)
        deal_contoursl.append(point_listl[index])
    for i in dis_minr:
        index = point_listr_index.index(i)
        deal_contoursr.append(point_listr[index])
    if (ll_q - rr_q >= 100 or count_l - count_r >= 2):
        for contour in deal_contoursl:
            for pose in contour:
                x = pose[0][0]
                y = pose[0][1]
                point = (x,y)
                if y >= (int)(img_height / 5):
                    point_list_l.append(point)
    elif (rr_q - ll_q >= 100 or count_r  - count_l >= 2):
        for contour in deal_contoursr:
            #[vxl,vyl,xl_,yl_] = cv2.fitLine(np.array(contour),cv2.DIST_L2,0.2,0.01,0.01)
            for pose in contour:
                x = pose[0][0]
                y = pose[0][1]
                point = (x,y)
                if y >= (int)(img_height / 5):
                    point_list_r.append(point)
    elif (abs)(rr_q - ll_q) < 100 :
        for contour in point_listl:
            for pose in contour:
                x = pose[0][0]
                y = pose[0][1]
                point = (x,y)
                if y >= (int)(img_height / 5):
                    point_list_l.append(point)
        for contour in point_listr:
            for pose in contour:
                x = pose[0][0]
                y = pose[0][1]
                point = (x,y)
                if y >= (int)(img_height / 5):
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
    for y,i in enumerate(ll_):
        if y < 60:
            continue
        if((rr_[y] < ll_[y]) or ((abs)(rr_[y] - ll_[y]) < 40)):
            if(rr_q - ll_q > 125):
                ll_[y] = rr_[y] - width[y]
            elif(ll_q - rr_q > 125 ):
                rr_[y] = ll_[y] + width[y]
        if (kl > 1 and kr > 1):
            ll_[y] = rr_[y] - width[y]
        elif (kr < -1 and kl < -1):
            rr_[y] = ll_[y] + width[y]
        
    print ('kl',vyl/vxl)
    print ('kr',vyr/vxr)
    print (len(ll_),len(self.last_ll))
    for y in range(179):
        if (self.start == 0):
            self.start = 1
            break
        if y < 60:
            continue
        if (abs)(self.last_ll[y] - ll_[y]) > 50 and self.bmx_flag == 0 and self.po_dao == 0  and self.pd_ping_flag == 0: 
            ll_[y] = self.last_ll[y]
        if (abs)(self.last_rr[y] - rr_[y]) > 50 and self.bmx_flag == 0 and self.po_dao == 0 and self.pd_ping_flag == 0:
            rr_[y] = self.last_rr[y]
    if self.bmx_flag == 1 or self.pd_ping_flag == 1 :
        rr_ = np.full((img_height,1),280)
        ll_ = np.full((img_height,1),40)
    for index,i in enumerate(rr_):
        pose = ((int)(i[0]),int(index))
        cv2.circle(img,pose,1,(255,0,0),1)
    for index,i in enumerate(ll_):
        pose = (int(i[0]),int(index))
        cv2.circle(img,pose,1,(255,0,255),1)
    for index,i in enumerate(self.last_rr):
        pose = ((int)(i[0]),int(index))
        cv2.circle(img,pose,1,(100,0,0),1)
    for index,i in enumerate(self.last_ll):
        pose = (int(i[0]),int(index))
        cv2.circle(img,pose,1,(255,0,100),1)
    self.last_rr = rr_.copy()
    self.last_ll = ll_.copy()
#得到中线
    mid = []
    for i in range(img_height):
        pose = ((int)((ll_[i] + rr_[i]) / 2),i)
        mid.append(pose)
#出中线
    for dian in mid:
        cv2.circle(img,dian,1,(100,0,255),1)

#画出基准线
    reference_line = []
    actual_line = []
    mid_value = 0
    start_raw = 80
#    print ('pitch:')
#    print (self.pitch) 
    if ((self.pitch > 8 or self.pitch < -2) and self.po_dao == 0):
        self.po_dao = 1
        start_raw = 80 
        t = Timer(2.0,self.clean_flag)
        t.start()
    self.pitch = float(self.pitch)
    self.yaw = round(self.yaw,2)
    text_pitch = str(self.pitch)
    text_yaw = str(self.yaw)
    bmx_flag = str(self.bmx_flag)
    text_raw = str(start_raw)
    text_pd = str(self.po_dao)
    cv2.putText(img,text_pitch,(250,50),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,255,200),2)
    cv2.putText(img,bmx_flag,(50,100),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,255,200),2)
    cv2.putText(img,text_pd,(100,100),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,255,200),2)
    cv2.putText(img,text_yaw,(0,50),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,0,200),2)
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
    if (self.bmx_flag == 1 or self.pd_ping_flag == 1):
       error = -self.imu_data_deal() * 10 
#    if (abs)(error - self.last_error) > 150:
#        error = self.last_error
       #pass
#    self.last_error = error
    error = float(error)
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
    print (edges.shape)
    print (self.last_img.shape)
    cha = np.subtract(edges,self.last_img)
    cv2.imshow('img_cha',cha)
    #contours,edges = self.extract(bgr_img)     #边缘提取后的轮廓和图像
    self.last_img = np.copy(edges)
    cv2.imshow('img___',self.last_img)
    error = self.counts_select(contours,edges,transform_img)    #中线提取 最终得到偏差值
    #error = 0
    end = time.time()
    print ('cost time',end-start)
    count = 0
    if self.bmx_flag == 1:
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

