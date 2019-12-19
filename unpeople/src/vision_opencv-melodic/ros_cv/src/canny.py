#-*-coding:utf-8-*
import cv2
import numpy as np
img_origin = cv2.imread("180.jpg")
img_origin = cv2.GaussianBlur(img_origin,(5,5),0)
img_cp = np.copy(img_origin)
def hsv_(img): #提取黄色后的 图片
	hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	lower_hsv = np.array([11,43,150])
	upper_hsv = np.array([28,255,255])
	mask = cv2.inRange(hsv_img,lowerb=lower_hsv,upperb=upper_hsv)
	return mask
def extract(img): #经过canny边缘提取后的图片
	blur_gray = cv2.GaussianBlur(img,(5,5),0,0)
	edges = cv2.Canny(blur_gray,50,150)
	return edges
def roi_mask(edges,vertices):  #进行范围性的滤除
	mask = np.zeros_like(edges)
	mask_color = 255
	cv2.fillPoly(mask, vertices, mask_color)
	masked_img = cv2.bitwise_and(edges, mask)
	return masked_img
def hough(img): #获取图片中的线条
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
def counts_select(contours,img):
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
	rr_ = np.zeros((720,1))
	#把轮廓上杂乱的点 提取成左右线（一行只有一个点）
	for pose in npl:
		pose = pose.A  #必须要将矩阵转化为数组 
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
		x = pose[0][0]
		y = pose[0][1]
		if (rr_[y] == 0):
			rr_[y] = x
		else:
			if ll_[y] < x:
				ll_[y] = ll_[y]
			else:
				ll_[y] = x
	#将（170-720）行的点进行保存  
	point_deal_l = []
	point_deal_r= []
	for index,i in enumerate(ll_):
		pose = (int(i[0]),int(index))
		point_deal_l.append(pose)
	for index,i in enumerate(rr_):
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
	
	for i  in range(20):
		mid_value = mid_value + mid[250+i][0]
		pose = (640,250+i)
		reference_line.append(pose)
	mid_value = int(mid_value / 20)

	print (mid_value)
	for i in range(20):
		pose = (mid_value,250+i)
		actual_line.append(pose)

	for dian in reference_line:
	    cv2.circle(img,dian,1,(0,255,0),1)
	for dian in actual_line:
	    cv2.circle(img,dian,1,(150,255,0),1)
	error = mid_value - 640
	text = str(error)
	cv2.putText(img,text,(200,100),cv2.FONT_HERSHEY_COMPLEX,2.0,(255,255,200),5)
	return error
hsv_img = hsv_(img_origin)
edges = extract(hsv_img)

roi = np.array([[(0,hsv_img.shape[0]),(350,100),(930,100),(hsv_img.shape[1],hsv_img.shape[0])]])
roi_edges = roi_mask(edges,roi)
cv2.imshow('lv',roi_edges)
pic = hough(roi_edges)

img11,contours,hier = cv2.findContours(pic,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
counts_select(contours,img_cp)

cv2.waitKey()
cv2.destroyAllWindows()

