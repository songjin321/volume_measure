## using visual hull algorithm to 3d reconstuct
import numpy as np
from matplotlib import pyplot as plt
from plotly.offline import plot
import plotly.graph_objs as go
from path import Path
from skimage import measure
from mpl_toolkits.mplot3d import Axes3D
from stl import mesh
from multiprocessing import pool
import time
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
# config logging module
import logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

data_path = Path('/home/neousys/Project/volume_measurement/DataFromMobile/detexture')
image_path = data_path/'IMAGE'
cam_file = data_path/'pose.txt'

# load img_segmentation_mask
img_segmentation_mask = np.load(data_path/'segmentation_mask.npy')
img_num = img_segmentation_mask.shape[0]
img_heigth = img_segmentation_mask.shape[1]
img_width = img_segmentation_mask.shape[2]
# load camera intrisic and extrinsic parameters from txt file
read_arrary = np.genfromtxt(cam_file).astype(np.float32)
K = read_arrary[:,:9].reshape(-1,3,3)
R = read_arrary[:,9:18].reshape(-1,3,3)
t = read_arrary[:,18:].reshape(-1,3,1)
# add gaussian noise to translate part
t_noise =  np.random.normal(0,0.005,(img_num,3,1))
#t = t+t_noise
cam_params = K @ np.concatenate((R, t), axis=2)

##### visula hull reconstruction #######
# create voxel grid
voxel_size = 0.002
dimension_lim = 0.25
dim_size = int(dimension_lim*2/voxel_size)
voxels_number = int(pow(dimension_lim*2/voxel_size, 3))
x=y=z=np.arange(-dimension_lim, dimension_lim, voxel_size)
Y,Z,X = np.meshgrid(x,y,z)
object_point_3D = np.vstack((X.flatten(), Y.flatten(), Z.flatten(), np.ones(voxels_number)))
# store the parallel processing result
img_val = np.ones((img_num, voxels_number)).astype(np.uint8)

# for each silhouette, project entire voxel to it
def process_image(img_index):
    # project voxel point to 2D image plane
    P = cam_params[img_index]
    point2D = P @ object_point_3D
    point_cam = np.floor(point2D/point2D[2,:]) # 3 x voxels_number
    
    # insure projected voxels are inside image     # 将不符合要求的赋值为（1,1）
    point_cam = np.where((point_cam[0,:] > (img_width-1))|(point_cam[1,:] > (img_heigth-1))|(point_cam<0),1,point_cam).astype(np.uint32)

    # increase counter of each voxel for object pixel
    cur_silhouette = img_segmentation_mask[img_index, :, :]

    img_val[img_index, :] = cur_silhouette[point_cam[1,:], point_cam[0,:]]
    
    print("finish image %d " % (img_index))
#with pool.Pool(6) as p:
#       p.map(process_image, range(img_num))
for i in range(img_num):
        process_image(i)
voxels_value_flatten = np.sum(img_val, axis=0)
## calculate object isosurface using march cube algorithm
error_amount = 5
maxv = np.amax(voxels_value_flatten)
iso_value = maxv-np.round(((maxv)/100)*error_amount)-0.5
# convert voxels list to voxel 3d  z x y 
voxels_3d = voxels_value_flatten.reshape(dim_size, dim_size, dim_size,order='F')

verts, faces, normals, values = measure.marching_cubes_lewiner(voxels_3d, iso_value, spacing=(1,1,1))

# Create the mesh
cube = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
for i, f in enumerate(faces):
    for j in range(3):
        cube.vectors[i][j] = verts[f[j],:]

# Write the mesh to file "cube.stl"
timestr = time.strftime("%Y%m%d-%H%M%S")
cube.save(timestr+'-cube.stl')

# save mesh and camera pose to file for plotting
np.save('plot.npy', {'cam_R':R, 'cam_t':t, 'voxel_value':voxels_value_flatten,'object_point':object_point_3D, 'verts':verts, 'faces':faces} )

# calculate object volume from verts, metric:mm
verts_xy = verts[:,:2][:,np.newaxis,:]
minirect = cv2.minAreaRect(verts_xy)
minbox = cv2.boxPoints(minirect)
zmax = np.amax(verts[:,2])
zmin = np.amin(verts[:,2])
# calculate area using Shoelace formulation
xy_area = 0.0
for i in range(3):
    xy_area = xy_area + (minbox[i][0]*minbox[i+1][1]-minbox[i+1][0]*minbox[i][1])
xy_area = xy_area + (minbox[3][0]*minbox[0][1]-minbox[0][0]*minbox[3][1]) 
scale = voxel_size*1000.0    
volume = (zmax-zmin)*np.fabs(xy_area)/2.0 * scale * scale * scale
print("volume = %f cm^3" % (volume/1000.0))
print("Hello World!")