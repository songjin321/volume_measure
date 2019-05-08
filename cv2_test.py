import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from path import Path
data_path = Path('/home/neousys/Project/volume_measurement/DataFromMobile/dinoSparseRing')
img_file = data_path/'IMAGE/1.png'
img = cv2.imread(img_file,0)
ret,thresh = cv2.threshold(img,127,255,0)
contours,hierarchy = cv2.findContours(thresh, 1, 2)
cnt = contours[0]
print('Hello world')