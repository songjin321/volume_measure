import numpy as np
import sys
from matplotlib import pyplot as plt
from path import Path
from skimage import measure
from skimage import io
import re

data_path = Path('/home/neousys/Project/volume_measurement/DataFromMobile/detexture')
segmentation_path = data_path/'mask'

img_num = len(segmentation_path.listdir())
img_segmentation_mask = np.load(segmentation_path/'0.npy')
img_width = img_segmentation_mask.shape[1]
img_heigth = img_segmentation_mask.shape[0]
T = 20

# load segmentaion_mask
img_segmentation_mask = np.zeros((img_num, img_heigth, img_width), np.uint8)

# img_file loaded is
for i,img_file in enumerate(segmentation_path.files('*.npy')):
    index = re.findall(r"\d+", img_file)
    img_segmentation_mask[int(index[0])] = np.load(img_file)

# imshow 16 segmentated image

fig=plt.figure(figsize=(8, 8))
columns = 4
rows = 4
for i in range(1, columns*rows +1):
    fig.add_subplot(rows, columns, i)
    plt.imshow(img_segmentation_mask[i-1])
plt.show()

# save img_segmentation_mask
np.save(data_path/'segmentation_mask.npy', img_segmentation_mask)
print("Hello")