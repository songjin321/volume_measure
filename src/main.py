
import numpy as np
import sys
import cv2 as cv
from matplotlib import pyplot as plt
from path import Path
from skimage import measure

data_path = Path('/home/neousys/Project/volume_measure/DataFromVINS/box')
image_path = data_path/'IMAGE'
camera_path = data_path/'POSES'
img_num = len(image_path.listdir())
img_width = 480
img_heigth = 640

# segmentation using grabcut algorithm
img_segmentation_mask = np.zeros((img_num, img_heigth, img_width), np.uint8)
img_segmentation = np.zeros((img_num, img_heigth, img_width, 3), np.uint8)
images = np.zeros((img_num, img_heigth, img_width, 3), np.uint8)
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
    plt.imshow(images[i-1])
# plt.show()

# save it to file
# np.save(data_path/'segmentation_mask.npy', img_segmentation_mask)

img_segmentation_mask = np.load(data_path/'segmentation_mask.npy')

# load camera params
# camera intrinsics
K = np.zeros((3,3))
K[0,0] = 526.958
K[0,2] = 244.473
K[1,1] = 527.179
K[1,2] = 313.844
K[2,2] = 1.0
# read camera extrinsics from txt file
cam_params = np.zeros((img_num, 3, 4))
for i,cam_file in enumerate(camera_path.files('*.txt')):
    read_arrary = np.genfromtxt(cam_file).astype(np.float32)
    t = read_arrary[1:4].reshape(3,1)
    R = read_arrary[4:].reshape(3,3)
    P = K @ np.hstack((R, t))
    cam_params[i] = P

##### visula hull reconstruction
# create voxel grid
voxel_size = 0.005
dimension_lim = 0.5
voxels_number = int(pow(dimension_lim*2/voxel_size, 3))
voxels = np.zeros((voxels_number, 4))
x=y=z=np.arange(-dimension_lim, dimension_lim, voxel_size)
X,Y,Z = np.meshgrid(x,y,z)
voxels = np.vstack((X.flatten(), Y.flatten(), Z.flatten(), np.zeros(voxels_number))).T

# project voxel to silhouette
object_point_3D = voxels.T
object_point_3D[3,:] = np.ones((1, voxels_number))
for i in range(img_num):
    print("process image %d/%d" % (i, img_num))
    
    # project voxel point to 2D image plane
    P = cam_params[i]
    point2D = P @ object_point_3D
    point_cam = np.floor(point2D/point2D[2,:]) # 3 x voxels_number
    
    # insure projected voxels are inside image     # 将不符合要求的赋值为（1,1）
    point_cam = np.where((point_cam[0,:] > img_width)|(point_cam[1,:] > img_heigth)|(point_cam<0),1,point_cam).astype(np.uint8)

    # increase counter of each voxel for object pixel
    cur_silhouette = img_segmentation_mask[i, :, :]

    img_val = cur_silhouette[point_cam[0,:], point_cam[1,:]]
    voxels[:, 3] = voxels[:, 3] + img_val

# disply voxel using march cube algorithm
error_amount = 5
maxv = np.amax(voxels[:,3])
iso_value = maxv-np.round(((maxv)/100)*error_amount)-0.5
verts, faces, normals, values = measure.marching_cubes_lewiner(voxels, iso_value)

# TODO: calculate object volume from isosurface

print("Hello world!")