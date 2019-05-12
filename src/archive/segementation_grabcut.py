import numpy as np
import sys
import cv2 as cv
from matplotlib import pyplot as plt
from path import Path
from skimage import measure

data_path = Path('/home/neousys/Project/volume_measure/DataFromVINS/dinoSparseRing')
image_path = data_path/'IMAGE'
camera_path = data_path/'POSES'
img_num = len(image_path.listdir())
img_width = 480
img_heigth = 640

# segmentation using grabcut algorithm
img_segmentation_mask = np.zeros((img_num, img_heigth, img_width), np.uint8)
img_segmentation = np.zeros((img_num, img_heigth, img_width, 3), np.uint8)
images = np.zeros((img_num, img_heigth, img_width, 3), np.uint8)
# img_file loaded is
for i,img_file in enumerate(image_path.files('*.png')):
    img = cv.imread(img_file)
    images[i] = img
    mask = np.zeros(img.shape[:2],np.uint8)
    bgdModel = np.zeros((1,65),np.float64)
    fgdModel = np.zeros((1,65),np.float64)
    rect = (160,160,320,320)
    cv.grabCut(img,mask,rect,bgdModel,fgdModel,5,cv.GC_INIT_WITH_RECT)
    # assign foreground to 1, background to 0
    mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
    img = img*mask2[:,:,np.newaxis]
    img_segmentation_mask[i] = mask2
    img_segmentation[i] = img
    print("segemet image %d/%d" % (i, img_num))

# imshow 20 segmentated image
fig=plt.figure(figsize=(8, 8))
columns = 4
rows = 5
for i in range(1, columns*rows +1):
    fig.add_subplot(rows, columns, i)
    plt.imshow(img_segmentation[i-1])
plt.show()

# save img_segmentation_mask
np.save(data_path/'segmentation_mask.npy', img_segmentation_mask)
