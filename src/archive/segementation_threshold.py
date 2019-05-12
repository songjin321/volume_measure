import numpy as np
import sys
from matplotlib import pyplot as plt
from path import Path
from skimage import measure
from skimage import io
import re

data_path = Path('/home/neousys/Project/volume_measurement/DataFromMobile/dinoSparseRing')
image_path = data_path/'IMAGE'
camera_path = data_path/'POSES'
img_num = len(image_path.listdir())
img_width = 640
img_heigth = 480
T = 20

# segmentation using grabcut algorithm
img_segmentation_mask = np.zeros((img_num, img_heigth, img_width), np.uint8)
img_segmentation = np.zeros((img_num, img_heigth, img_width, 3), np.uint8)
# img_file loaded is
for i,img_file in enumerate(image_path.files('*.png')):
    img = io.imread(img_file)
    index = re.findall(r"\d+", img_file)
    print(img.shape)
    # assign foreground to 1, background to 0
    mask2 = np.where((img[:,:,0]>T) & (img[:,:,1] > T) & (img[:,:,2] > T) ,1,0).astype('uint8')
    img = img*mask2[:,:,np.newaxis]
    img_segmentation_mask[int(index[0])-1] = mask2
    img_segmentation[int(index[0])-1] = img
    print("segemet image %d/%d" % (i, img_num))

# imshow 16 segmentated image

fig=plt.figure(figsize=(8, 8))
columns = 4
rows = 4
for i in range(1, columns*rows +1):
    fig.add_subplot(rows, columns, i)
    plt.imshow(img_segmentation[i-1])
plt.show()

# save img_segmentation_mask
np.save(data_path/'segmentation_mask.npy', img_segmentation_mask)
print("Hello")