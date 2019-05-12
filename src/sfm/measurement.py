import numpy as np
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from open3d import *

pc_file_path="data/shoe_tag/reconstruction_sequential/robust_aligned.ply"
GROUND_Z = 20
OBJ_AREA = [[-100, 500],    #x
            [-600, -50],    #y
            [-20, 600]]    #z
OBJ_AREA_CONSTRAINT=False
def display_inlier_outlier(cloud, ind):
    inlier_cloud = select_down_sample(cloud, ind)
    outlier_cloud = select_down_sample(cloud, ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    draw_geometries([inlier_cloud, outlier_cloud])

def draw_obj_box(pcd,minbox,zmax):
    points = [[p[0], p[1], 0] for p in minbox[:]]
    
    points1 = [[p[0], p[1], zmax] for p in minbox[:]]
    points.extend(points1)
    lines = [[0, 1], [1, 2], [2, 3], [0, 3],
             [4, 5], [5, 6], [6, 7], [4, 7],
             [0, 4], [1, 5], [2, 6], [3, 7]]
    colors = [[1, 0, 0] for i in range(len(lines))]
    line_set = LineSet()
    line_set.points = Vector3dVector(points)
    line_set.lines = Vector2iVector(lines)
    line_set.colors = Vector3dVector(colors)
    draw_geometries([pcd, line_set])

def p_in_area(p, area):
    if not OBJ_AREA_CONSTRAINT: return True
    if p[0] > area[0][0] and p[0] < area[0][1] and p[1] > area[1][0] and p[1] < area[1][1] and p[2] > area[2][0] and p[2] < area[2][1]:
        return True
    return False
def main(arg):
    
    # 读取点云robust_aligned.ply，去除离群点，并计算体积
    #### 读取点云,去除离群点 #####
    pcd = read_point_cloud(
        arg[1])
    print(pcd)

    index_rm_cam = []
    for i in range(len(pcd.points)):
        p = pcd.points[i]
        c = pcd.colors[i]
        # not camera point
        if c[0] == 1 and c[1] == 1 and c[2] == 1 and p_in_area(p, OBJ_AREA):
            index_rm_cam.append(i)

    #paint
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    if OBJ_AREA_CONSTRAINT:
        obj_area = [[OBJ_AREA[0][0], OBJ_AREA[1][0]],
                    [OBJ_AREA[0][1], OBJ_AREA[1][0]],
                    [OBJ_AREA[0][1], OBJ_AREA[1][1]],
                    [OBJ_AREA[0][0], OBJ_AREA[1][1]]]
        draw_obj_box(pcd, obj_area, OBJ_AREA[2][1])

    pcd_rm_cam = select_down_sample(pcd, index_rm_cam)

    #pcd.paint_uniform_color([1, 0.706, 0])
    #draw_geometries([pcd])

    print("Radius oulier removal")
    cl_r, ind = radius_outlier_removal(pcd_rm_cam,
                                    nb_points=2, radius=25)
    #display_inlier_outlier(pcd_rm_cam, ind)


    print("Statistical oulier removal")
    cl_s, ind = statistical_outlier_removal(cl_r,
                                        nb_neighbors=3, std_ratio=2)
    #display_inlier_outlier(cl_r, ind)

    print("Radius oulier removal 2")
    cl_r, ind = radius_outlier_removal(cl_s,
                                    nb_points=2, radius=25)
    #display_inlier_outlier(cl_s, ind)

    # verts表示点云，N代表点的个数，3表示点的x，y，z坐标, 单位mm。将点云赋值给verts
    cl_obj_with_ground = cl_r
    #verts = np.zeros((N, 3))

    verts = []
    index_rm_ground = []
    for i in range(len(cl_obj_with_ground.points)):
        p = cl_obj_with_ground.points[i]
        if p[2] > GROUND_Z:  # not camera point
            index_rm_ground.append(i)
            verts.append(p)
    #display_inlier_outlier(cl_obj_with_ground, index_rm_ground)

    pcd_obj = select_down_sample(cl_obj_with_ground, index_rm_ground)
    #draw_geometries([pcd_obj])

    verts = np.array(verts,np.int32)

    # calculate object volume from verts
    verts_xy = verts[:,:2][:,np.newaxis,:]
    minirect = cv2.minAreaRect(verts_xy)
    minbox = cv2.boxPoints(minirect)


    zmax = np.amax(verts[:, 2])

    draw_obj_box(pcd, minbox, zmax)

    #zmin = np.amin(verts[:,2])
    zmin = 0
    # calculate area using Shoelace formulation
    xy_area = 0.0
    for i in range(3):
        xy_area = xy_area + (minbox[i][0]*minbox[i+1][1]-minbox[i+1][0]*minbox[i][1])
    xy_area = xy_area + (minbox[3][0]*minbox[0][1]-minbox[0][0]*minbox[3][1])   
    volume = (zmax-zmin)*np.fabs(xy_area)/2.0

    print("volume = %f cm^3" % (volume/1000.0))

if __name__ == "__main__":

    main(sys.argv)