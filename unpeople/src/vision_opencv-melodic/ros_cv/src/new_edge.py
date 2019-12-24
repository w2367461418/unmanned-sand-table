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
    self.second_floor = 0
    self.start = 0
    self.pitch = 0
    self.yaw = 0
    self.po_dao = 0
    self.down_po = 0
    self.up_po = 0
    self.count = 0
    self.last_error = 0
    self.pd_ping_flag  = 0
    self.last_rr =  np.zeros((119,1))
    self.last_ll =  np.zeros((119,1))
    self.last_left_up_pose = [0,0]
    self.last_left_down_pose = [0,0]
    self.last_right_up_pose = [0,0]
    self.last_right_down_pose = [0,0]
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
    #self.pd_ping_flag = 0
  def extract(self,img): 
    img_height = 180 
    img_width = 320

    blur = cv2.GaussianBlur(img,(3,3),0,0)
    gray = cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY)
    gray_cp = gray[59:179,:]
    img_origin = img[59:179,:,:]
    cv2.imshow('img_origin',img_origin)
    mid = np.median(gray_cp)


    min_thresh = (int)(mid * 0.66)
    max_thresh = (int)(mid * 1.33)

    gray_ = np.copy(gray)
    edges_ = cv2.Canny(gray_,min_thresh,max_thresh)
    cv2.rectangle(edges_,(0,0),(319,60),0,-1)
   # cv2.imshow('ewai',edges_)
#    edges_ = cv2.bitwise_not(edges_,edges_)
#    cv2.imshow('qiege',edges_)
    h_all = cv2.findContours(edges_,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    contours_all = h_all[1]
    contours_all = [contour for contour in contours_all  if ((abs)(cv2.fitLine(contour,cv2.DIST_L2,0.2,0.01,0.01)[1]/cv2.fitLine(contour,cv2.DIST_L2,0.2,0.01,0.01)[0]) > 0.5 and len(contour) > 100)]
    contours_bmx = [contour for contour in contours_all if len(contour) > 200]
    if self.pitch > 25 and len(contours_all) >= 12 and self.pd_ping_flag == 0:
        self.pd_ping_flag = 1
        self.second_floor = 1
        t = Timer(2.0,self.clean_flag)
        t.start()
    if (self.second_floor == 0 and len(contours_bmx) >= 9 and self.bmx_flag == 0 and self.pitch < 8):
        self.bmx_flag = 1
        t = Timer(2.0,self.clean_flag)
        t.start()
    temp = np.ones(edges_.shape,np.uint8)*255  #白色幕布
    cv2.drawContours(temp,contours_all,-1,(0,255,0),5)
    cv2.line(temp,(0,60),(319,60),0,5)
    cv2.line(temp,(0,179),(319,179),0,5)
    cv2.line(temp,(0,60),(0,179),0,5)
    cv2.line(temp,(319,60),(319,179),0,5)
    cv2.rectangle(temp,(0,0),(319,60),0,1)
    #cv2.imshow("houa",temp)
    #print (temp.shape)
    new_img = temp[60:179,:]
    #cv2.imshow('new',new_img)
    h = cv2.findContours(new_img,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
    h_simple = cv2.findContours(edges_,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    contours_simple = h_simple[1]
    contours_all = h[1]
    area = []
    contour_get = []
    for contour in contours_all: 
        area_d = cv2.contourArea(contour)
        area.append(area_d)
    ret = map(area.index,heapq.nlargest(1,area))
    #dis_max = heapq.nlargest(4,area)
   # print (list(ret))
    for index in ret:
        cv2.drawContours(new_img,contours_all,index,(200,255,0),cv2.FILLED)
        break
    #cv2.imshow('new__',new_img)
    new_temp = np.ones(new_img.shape,np.uint8)*255  #白色幕布
    for index in ret:
        cv2.drawContours(new_temp,contours_all,index,(0,0,0),2)
        break
    '''
    corners = cv2.goodFeaturesToTrack(new_temp,4,0.01,10)
    corners = np.int0(corners)
    for i in corners:
        x,y = i.ravel()
        cv2.circle(new_temp,(x,y),5,(255,100,100),-1)
    cv2.imshow('new_temp',new_temp)
    '''
   # print (new_temp.shape)
    cv2.imshow('new',new_temp)
    return contours_all,new_temp,img_origin  #返回最大的轮廓  和 最新的图像

  def img_deal(self,contours,img,img_origin):  #img[height][width]
    height = img.shape[0] - 3 
    width = img.shape[1] - 2 
    print (height,width)
    left_error_flag = 0
    right_error_flag = 0
#斑马线检测
    if self.bmx_flag == 1 or self.pd_ping_flag == 1:
        rr_ = np.full((height,1),310)
        ll_ = np.full((height,1),40)
    else:
        #获取四个角点坐标
        left_up = 0
        left_up_pose = [0,0]
        left_up_flag = 0
        left_down = 0
        left_down_pose = [0,0]
        left_down_flag = 0
        right_up = width
        right_up_pose = [0,0]
        right_up_flag = 0
        right_down = width
        right_down_pose = [0,0]
        right_down_flag = 0
        row_up = 4 
        while(1):
            if (img[row_up][left_up] == 0 and left_up_flag == 0):
                left_up_pose = [row_up,left_up]
                left_up_flag = 1
            if (img[height-2][left_down] == 0 and left_down_flag == 0):
                left_down_pose = [height-2,left_down]
                left_down_flag = 1
            if (img[row_up][right_up] == 0 and right_up_flag == 0):
                right_up_pose = [row_up,right_up]
                right_up_flag = 1
            if (img[height-2][right_down] == 0 and right_down_flag == 0):
                right_down_pose = [height-2,right_down]
                right_down_flag = 1
            left_up = left_up + 1
            if left_up >= 319:
                left_up = 319
            left_down = left_down + 1
            if left_down >= 319:
                left_down = 319
            right_up = right_up - 1
            if right_up <= 0:
                right_up = 0
            right_down = right_down - 1
            if right_down <= 0:
                right_down = 0
            if left_up >= 319 and right_up <= 0:  #防止程序卡死，所搜行数进行++
                row_up = row_up + 4
                right_up = width
                left_up = 0
            if row_up >= 60:
                left_up_pose = self.last_left_up_pose
                left_down_pose = self.last_left_down_pose
                right_up_pose = self.last_right_up_pose
                right_down_pose = self.last_right_down_pose
                break

            if (left_up_flag == 1 and left_down_flag == 1 and right_up_flag == 1 and right_down_flag == 1):
                break
        self.last_left_up_pose = left_up_pose
        self.last_left_down_pose = left_down_pose
        self.last_right_up_pose = right_up_pose
        self.last_right_down_pose = right_down_pose
        if (left_up_pose[1] - left_down_pose[1] > 15) and (right_up_pose[1] - right_down_pose[1] < 15 and (right_up_pose[1] - left_up_pose[1]) - (right_down_pose[1] - left_down_pose[1]) < 20):
            self.down_po = 1
        else:
            self.down_po = 0
        #进行四个点的纠正
        if (right_up_pose[1] - left_up_pose[1]) -  (right_down_pose[1] - left_down_pose[1]) > 20:
            if (((left_down_pose[1] + right_down_pose[1]) / 2) >= (width / 2)):
                right_up_pose[1] = (int)(left_up_pose[1])
                left_up_pose[1] = 0
                right_down_pose[1] = (int)(left_down_pose[1])
                left_down_pose[1] = 0
            else: 
                left_up_pose[1] = (int)(right_up_pose[1])
                right_up_pose[1] = width
                left_down_pose[1] = (int)(right_down_pose[1])
                right_down_pose[1] = width
        right_dis = (right_up_pose[1] - right_down_pose[1])
        left_dis = (left_up_pose[1] - left_down_pose[1])
        print ('left',left_dis)
        print ('right',right_dis)
        #检测轮廓是否为正确轮廓
        if (right_up_pose[1] - left_up_pose[1] < 40):
            if ((right_down_pose[1] + left_down_pose[1]) / 2 < width / 2):
                left_error_flag = 1
            elif ((right_down_pose[1] + left_down_pose[1]) / 2 > width / 2):
                right_error_flag = 1
            '''
            if (((abs)(right_dis) - (abs)(left_dis) > 15)):
                if (abs)(left_dis) > 10:
                    left_error_flag = 1
            elif (((abs)(left_dis) - (abs)(right_dis) > 15)):
                if (abs)(right_dis) > 10:
                    right_error_flag = 1
            '''
        mid_up_pose = (int((left_up_pose[0] + right_up_pose[0]) / 2),int((left_up_pose[1] + right_up_pose[1]) / 2))
        mid_down_pose = (int((left_down_pose[0] + right_down_pose[0]) / 2),int((left_down_pose[1] + right_down_pose[1]) / 2))
        print ('left_up',left_up_pose)
      #  print (left_down_pose)
        print ('right_up',right_up_pose)
      #  print (right_down_pose)
        print ('mid_up',mid_up_pose)
        mid_line_k =(float)(mid_up_pose[1] - mid_down_pose[1]) / (float)(mid_up_pose[0] - mid_down_pose[0])
        mid_line_b = mid_up_pose[1] - mid_line_k * mid_up_pose[0] 
        mid_ = np.full((height,1),int(width / 2))
        for row in range(height):
            y = row
            x = (int)(mid_line_k * y + mid_line_b)
            mid_[y] = x

        point_list_l = []
        point_list_r = []
        #区分左右点的归属  
        for contour in contours:
            for pose in contour:
                x = pose[0][0]
                y = pose[0][1]
                if y > 115:
                    continue
                point = (x,y)
                if x < mid_[y]: 
                    point_list_l.append(point) 
                else:
                    point_list_r.append(point)
        print ('len',len(point_list_l),len(point_list_r))
        npl = np.mat(point_list_l)
        npr = np.mat(point_list_r)
        #得到左右的边界点
        ll_ = np.zeros((height,1))
        rr_ = np.full((height,1),width - 1)
        width_row = np.full((height,1),0)
        for i in range (height):
            l = 40
            r = 280
            wid = r - l + 30
            width_row[i] = wid

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
            if (rr_[y] == width-1):
                rr_[y] = x
            else:
                if rr_[y] < x:
                    rr_[y] = rr_[y]
                else:
                    rr_[y] = x
    if (left_error_flag == 1):    
        ll_ = np.full((height,1),3)
    elif (right_error_flag == 1):
        rr_ = np.full((height,1),315)
    point_deal_l = []
    point_deal_r= []
    for index,i in enumerate(ll_):
        if (i[0] <= 5):
            if(rr_[index] > width - 5):
                i[0] = 25
                ll_[index] = 25
            else:
                i[0] = rr_[index] - width_row[index]
                ll_[index] = rr_[index] - width_row[index]
        pose = (int(i[0]),int(index))
        point_deal_l.append(pose)
    for index,i in enumerate(rr_):
        if (i[0] >= width - 5):
            if(ll_[index] <= 5):
                i[0] = 295
                rr_[index]  = 295
            else:
                i[0] = ll_[index] + width_row[index]
                rr_[index] = ll_[index] + width_row[index]
        pose = (int(i[0]),int(index))
        point_deal_r.append(pose)
    #与上次左右边界线进行  比较滤除 变化较大的区域
    for y in range(115):
        if (self.start == 0):
            self.start = 1
            break
        if (abs)(self.last_ll[y] - ll_[y]) > 80 and self.bmx_flag == 0 and self.po_dao == 0  and self.pd_ping_flag == 0 and self.down_po == 0: 
            ll_[y] = self.last_ll[y]
        if (abs)(self.last_rr[y] - rr_[y]) > 80 and self.bmx_flag == 0 and self.po_dao == 0 and self.pd_ping_flag == 0 and self.down_po == 0:
            rr_[y] = self.last_rr[y]
    real_mid = np.full((height,1),0)
    for index,i in enumerate(rr_):
        pose = (int(i[0]),int(index))
        cv2.circle(img_origin,pose,1,(255,0,0),1)
    for index,i in enumerate(ll_):
        pose = (int(i[0]),int(index))
        cv2.circle(img_origin,pose,1,(255,0,255),1)
    for index,i in enumerate(ll_):
        real_mid[index] = int(((ll_[index] + (rr_[index])) / 2))
        pose = (real_mid[index],int(index))
        cv2.circle(img_origin,pose,1,(100,125,0),1)
    self.last_rr = rr_.copy()
    self.last_ll = ll_.copy()
#坡到检测
    start_raw = 25
    if ((self.pitch <= 8 and self.pitch >= -4) and self.pd_ping_flag == 1):
        self.pd_ping_flag = 0
    if ((self.pitch > 8 or self.pitch < -2)): #坡到前瞻变换
        start_raw = 40 
    if ((self.pitch > 8 or self.pitch < -2) and self.po_dao == 0 and self.second_floor == 0):
        self.po_dao = 1
        t = Timer(2.0,self.clean_flag)
        t.start()
    reference_line = []
    actual_line = []
    mid_value = 0
    for i  in range(10):  #计算10行的中值 求平均
        mid_value = mid_value + real_mid[start_raw+i]
        pose = (int(width/2),start_raw+i)
        reference_line.append(pose)
    mid_value = (int)(mid_value / 10)
    print ('mid_value',mid_value)
    for i in range(10):
        pose = (mid_value,start_raw+i)
        actual_line.append(pose)

    for dian in reference_line:
        cv2.circle(img_origin,dian,2,(50,50,50),1)
    for dian in actual_line:
        cv2.circle(img_origin,dian,2,(150,255,0),1)
    #区分坡到与平地的偏差计算公式
    if (start_raw == 40):
        error = mid_value - ((int)(width / 2) - 15)
    else:
        error = mid_value - ((int)(width / 2) - 15)
#imu偏差寻线
    if (self.bmx_flag == 1 ):
       error = -self.imu_data_deal() * 10 
    self.pitch = float(self.pitch)
    self.yaw = round(self.yaw,2)
    self.pitch = round(self.pitch,2)
    text_pitch = str(self.pitch)
    text_yaw = str(self.yaw)
    bmx_flag = str(self.bmx_flag)
    text_raw = str(start_raw)
    text_pd = str(self.po_dao)
    cv2.putText(img_origin,text_pitch,(150,50),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,255,200),1)
    cv2.putText(img_origin,bmx_flag,(0,100),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,255,200),1)
    cv2.putText(img_origin,text_pd,(50,100),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,255,200),1)
    cv2.putText(img_origin,text_yaw,(0,50),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,0,200),1)
    cv2.putText(img_origin,text_raw,(150,100),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,255,200),1)
    error = float(error)
    text_error = str(error)
    cv2.putText(img_origin,text_error,(100,20),cv2.FONT_HERSHEY_COMPLEX,1.0,(255,255,200),1)

    cv2.imshow('bian',img_origin)
    return error

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
  def contour_error(self,contour_all,contour_simple,img_ed,img):
    for pose in contour:
        x = pose[0][0]
        y = pose[0][1]
        point = (x,y)
   
    
  def counts_select(self,contours,img_ed,img):    #中线提取
     pass
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
    #img_hl = self.rgb_hls_lab(transform_img)
    contours,new_img,img_origin = self.extract(transform_img)     #边缘提取后的轮廓和图像
    error = self.img_deal(contours,new_img,img_origin)  #img[height][width]
    #contours,edges = self.extract(transform_img)     #边缘提取后的轮廓和图像
    if self.bmx_flag == 1:
        str_zerba = 1
    else:
        str_zerba = 0
    str_error = "%d"%(error)   #将数字转换为字符串
    str_msg = str_error + " " + str(str_zerba)
    self.pub.publish(str_msg)     #发布偏差话题
    '''
#    print (edges.shape)
#    print (self.last_img.shape)
#    cha = np.subtract(edges,self.last_img)
#    cv2.imshow('img_cha',cha)
    #contours,edges = self.extract(bgr_img)     #边缘提取后的轮廓和图像
#    self.last_img = np.copy(edges)
#    cv2.imshow('img___',self.last_img)
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
    ''' 
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

