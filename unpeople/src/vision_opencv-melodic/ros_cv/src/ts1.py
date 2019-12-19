import cv2
import numpy as np
a = cv2.imread('./1_.jpg')
b = cv2.imread('./2_.jpg')
h,w = a.shape[:2]
print (h)
print (w)
cv2.imshow('tupian',a)

#pts = np.float32([[140,150],[480,150],[50,280],[550,270]])
pts = np.float32([[140,150],[475,150],[50,280],[546,270]])
pts1 = np.float32([[0,0],[w-1,0],[0,h-1],[w-1,h-1]])
m = cv2.getPerspectiveTransform(pts,pts1)
print (m)

dst = cv2.warpPerspective(b,m,(640,360))
cv2.imshow('dst',dst)


cv2.waitKey()
cv2.destroyAllWindows()

