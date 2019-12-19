#-*-coding:utf-8-*
import cv2
import numpy as np

def extract(img):
	hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV) 
	hsv_img = cv2.medianBlur(hsv_img,5)
	hsv_img = cv2.medianBlur(hsv_img,5)
	hsv_img = cv2.medianBlur(hsv_img,5)
	hsv_img = cv2.medianBlur(hsv_img,5)
	hsv_img = cv2.medianBlur(hsv_img,5)
	hsv_img = cv2.medianBlur(hsv_img,5)
	hsv_img = cv2.medianBlur(hsv_img,5)
	#hsv_img = cv2.blur(hsv_img,(5,5))
	#hsv_img = cv2.GaussianBlur(hsv_img,(5,5),0)
	#hsv_img = cv2.bilateralFilter(hsv_img,9,75,75)
	lower_hsv = np.array([11,43,150])
	upper_hsv = np.array([28,255,255])
	mask = cv2.inRange(hsv_img,lowerb=lower_hsv,upperb=upper_hsv) #获得提取黄色后的图像 
	kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
	mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel,iterations=1)
	mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel,iterations=1)
	mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel,iterations=1)
	mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel,iterations=1)
	mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel,iterations=1)
	mask = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel,iterations=1)
	mask = cv2.medianBlur(mask,5)
	mask = cv2.medianBlur(mask,5)


	#ret, thresh = cv2.threshold(cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY),
	#                            200, 255, cv2.THRESH_BINARY)
	#cv2.imshow('one',mask)
	#cv2.waitKey()
	#cv2.destroyAllWindows()




	 
	#findContour用于找到不规则形状的轮廓
	img11,contours,hier = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	print (len(contours)) 
	count = 0
	for contour in contours:
	    #计算一个简单的边界框
	    x,y,w,h = cv2.boundingRect(contour)
	    #画出边界框
	    cv2.rectangle(img, (x,y), (x+w, y+h),(0,0,255),2)
	    #计算包围目标的最小矩形区域
	    rect = cv2.minAreaRect(contour)
	    box = cv2.boxPoints(rect)
	    box = np.int0(box)
	#    cv2.drawContours(img, [box], 0, (255,0,255), 2)
	#    cv2.minEnclosingCircle函数返回一个二元组，第一个元组为圆心坐标，第二个为半径
	    (x0,y0), radius = cv2.minEnclosingCircle(contour)
	    if radius < 10:
		continue
	    else:
		center = (int(x0), int(y0))
		radius = int(radius)
		img = cv2.circle(img, center, radius, (0,255,255))
	    count = count + 1

	point_list_l = []
	point_list_r = []
	last_y = 0
	for pose in contours[0]:
	    x =  (pose[0][0])
	    y = pose[0][1]
	    if y < last_y:
		break
	    else:
		point = (x,y) 
		point_list_r.append(point)
	#        print (pose)
	    last_y = y
	last_y = 0
	for pose in contours[1]:
	    x =  (pose[0][0])
	    y = pose[0][1]
	    if y < last_y:
		break
	    else:
		point = (x,y) 
		point_list_l.append(point)
	#        print (pose)
	    last_y = y



	print (len(point_list_l))
	print (len(point_list_r))
	for dian in point_list_l:
	    cv2.circle(img,dian,1,(0,0,255),4)
	for dian in point_list_r:
	    cv2.circle(img,dian,1,(0,0,255),4)


	#cv2.drawContours(img, contours, -1, (255,0,0), 1)
	cv2.imshow("findcontour",img)
	cv2.waitKey()
	cv2.destroyAllWindows()
img_ = cv2.pyrDown(cv2.imread("11.jpg", cv2.IMREAD_UNCHANGED))
extract(img_)

