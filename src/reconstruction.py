## using visual hull algorithm to 3d reconstuct
import numpy as np
from matplotlib import pyplot as plt
from path import Path
from skimage import measure
from mpl_toolkits.mplot3d import Axes3D
from stl import mesh
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

data_path = Path('/home/neousys/Project/volume_measurement/DataFromMobile/dinoSparseRing')
image_path = data_path/'IMAGE'
cam_file = data_path/'pose.txt'
mask_path = data_path/'MASK'
img_num = len(image_path.listdir())
img_width = 640
img_heigth = 480

# load img_segmentation_mask
img_segmentation_mask = np.load(data_path/'segmentation_mask.npy')

# load camera intrisic and extrinsic parameters from txt file
read_arrary = np.genfromtxt(cam_file).astype(np.float32)
K = read_arrary[:,:9].reshape(-1,3,3)
R = read_arrary[:,9:18].reshape(-1,3,3)
t = read_arrary[:,18:].reshape(-1,3,1)

cam_params = K @ np.concatenate((R, t), axis=2)

##### visula hull reconstruction #######
# create voxel grid
voxel_size = 0.001
dimension_lim = 0.1
dim_size = int(dimension_lim*2/voxel_size)
voxels_number = int(pow(dimension_lim*2/voxel_size, 3))
voxels = np.zeros((voxels_number, 4))
'''
x=np.arange(-0.07, 0.02, voxel_size)
y=np.arange(-0.02, 0.07, voxel_size)
z=np.arange(0.02, -0.07,  -voxel_size)
dim_size=int(0.09/voxel_size+1)
voxels_number = int(pow(0.09/voxel_size+1, 3))
X,Z,Y = np.meshgrid(x,z,y)
'''
x=y=z=np.arange(-dimension_lim, dimension_lim, voxel_size)
Y,Z,X = np.meshgrid(x,y,z)
voxels = np.vstack((X.flatten(), Y.flatten(), Z.flatten(), np.zeros(voxels_number))).T

# project voxel to silhouette
object_point_3D = np.zeros(voxels.T.shape)
object_point_3D[:] = voxels.T
object_point_3D[3,:] = np.ones((1, voxels_number))
for i in range(img_num):
    print("process image %d/%d" % (i, img_num))
    
    # project voxel point to 2D image plane
    P = cam_params[i]
    point2D = P @ object_point_3D
    point_cam = np.floor(point2D/point2D[2,:]) # 3 x voxels_number
    
    # insure projected voxels are inside image     # 将不符合要求的赋值为（1,1）
    point_cam = np.where((point_cam[0,:] > (img_width-1))|(point_cam[1,:] > (img_heigth-1))|(point_cam<0),1,point_cam).astype(np.uint32)

    # increase counter of each voxel for object pixel
    cur_silhouette = img_segmentation_mask[i, :, :]

    img_val = cur_silhouette[point_cam[1,:], point_cam[0,:]]
    voxels[:, 3] = voxels[:, 3] + img_val

## calculate object isosurface using march cube algorithm
error_amount = 5
maxv = np.amax(voxels[:,3])
iso_value = maxv-np.round(((maxv)/100)*error_amount)-0.5
# convert voxels list to voxel 3d  z x y 
voxels_3d = voxels[:,-1].reshape(dim_size, dim_size, dim_size,order='F')
verts, faces, normals, values = measure.marching_cubes_lewiner(voxels_3d, iso_value, spacing=(1, 1, 1))

'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_trisurf(verts[:, 0], verts[:,1], faces, verts[:, 2], lw=1)
plt.show()
'''

# Create the mesh
cube = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
for i, f in enumerate(faces):
    for j in range(3):
        cube.vectors[i][j] = verts[f[j],:]

# Write the mesh to file "cube.stl"
cube.save('cube.stl')

# calculate object volume from verts, metric:mm
verts_xy = verts[:,:2].astype(np.int32)[:,np.newaxis,:]
minirect = cv2.minAreaRect(verts_xy)
minbox = cv2.boxPoints(minirect)
zmax = np.amax(verts[:,2])
zmin = np.amin(verts[:,2])
# using Shoelace formula calculate area
xy_area = 0.0
for i in range(2):
    xy_area = xy_area + (minbox[i][0]*minbox[i+1][1]-minbox[i+1][0]*minbox[i][1])
volume = (zmax-zmin)*np.fabs(xy_area)/2.0
print("volume = %f cm^3" % (volume/1000.0))
print("Hello World!")