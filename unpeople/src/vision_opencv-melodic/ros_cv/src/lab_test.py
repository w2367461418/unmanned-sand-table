#-*-coding: utf-8 -*-
import cv2
import numpy as np


img = cv2.imread('./5.jpg')
def rgb_hls_lab(img):
  img_lab = cv2.cvtColor(img,cv2.COLOR_BGR2LAB) #提取黄色   b通道
  img_hls = cv2.cvtColor(img,cv2.COLOR_BGR2HLS) #提取白色   l通道
  imghls_h = img_hls[:,:,0]
  imghls_l = img_hls[:,:,1]
  imghls_s = img_hls[:,:,2]

  imglab_l = img_lab[:,:,0]
  imglab_a = img_lab[:,:,1]
  imglab_b = img_lab[:,:,2]

  print (imglab_b)
  retval_lab,dst_lab = cv2.threshold(imglab_b,160,255,cv2.THRESH_BINARY)
  retval_hls,dst_hls = cv2.threshold(imghls_l,220,255,cv2.THRESH_BINARY)

  hls_lab = np.zeros_like(dst_lab)
  hls_lab = dst_lab + dst_hls 
  #cv2.imshow('hl',hls_lab)
  #cv2.imshow('ls',dst_lab)
  #cv2.imshow('ls',dst_hls)
  #cv2.imshow('origin1',imglab_b)
  cv2.imshow('hls_lab',hls_lab)
rgb_hls_lab(img)
cv2.waitKey()
cv2.destroyAllWindows()
