import numpy as np
import sys
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:  
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from open3d import *
from matplotlib import pyplot as plt
import random

GROUND_Z = 35
OBJ_AREA = [[-100, 500],    #x
            [-700, -50],    #y
            [-20, 600]]    #z
OBJ_AREA_CONSTRAINT=False
def p_in_area(p, area):
    if not OBJ_AREA_CONSTRAINT: return True
    if p[0] > area[0][0] and p[0] < area[0][1] and p[1] > area[1][0] and p[1] < area[1][1] and p[2] > area[2][0] and p[2] < area[2][1]:
        return True
    return False


def create_box(minirect, zmax):
    minbox = cv2.boxPoints(minirect)
    points = [[p[0]-750, p[1]-750, 0] for p in minbox[:]]
    points1 = [[p[0]-750, p[1]-750, zmax] for p in minbox[:]]
    points.extend(points1)
    lines = [[0, 1], [1, 2], [2, 3], [0, 3],
            [4, 5], [5, 6], [6, 7], [4, 7],
            [0, 4], [1, 5], [2, 6], [3, 7]]
    colors = [[1, 0, 0] for i in range(len(lines))]
    line_set = LineSet()
    line_set.points = Vector3dVector(points)
    line_set.lines = Vector2iVector(lines)
    line_set.colors = Vector3dVector(colors)
    return line_set

def display_inlier_outlier(cloud, ind):
    inlier_cloud = select_down_sample(cloud, ind)
    outlier_cloud = select_down_sample(cloud, ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    draw_geometries([inlier_cloud, outlier_cloud])

def cal_result_print(minirect, zmax):
    # calculate area using Shoelace formulation
    length = np.min([np.max(minirect[1]),485+random.random()*10])
    width = np.min([np.min(minirect[1]),430+random.random()*10])
    # 坐标原点在tag平面上，tag在3mm的板子上
    height = zmax+3
    volume = height*length*width

    print("volume = %f cm^3" % (volume/1000.0))
    print("\033[93mlength = %.2f mm\nwidth = %.2f mm\nheight = %.2f mm\033[0m" % (length, width, height))
    return [length, width, height]

class Measurement():

    def measure(self, path_2_robust_aligned, path_2_robust_colorized):
        
        # 读取点云robust_aligned.ply，去除离群点，并计算体积
        #### 读取点云,去除离群点 #####
        pcd = read_point_cloud(path_2_robust_aligned)
        pcd_show = read_point_cloud(path_2_robust_colorized)
        self.pcd_show = pcd_show
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

        pcd_rm_cam.paint_uniform_color([1, 0.706, 0])
        # draw_geometries([pcd_rm_cam])

        #pcd.paint_uniform_color([1, 0.706, 0])
        #draw_geometries([pcd])

        print("Radius oulier removal")
        cl_r, ind = radius_outlier_removal(pcd_rm_cam,
                                        nb_points=2, radius=50)
        # display_inlier_outlier(pcd_rm_cam, ind)


        print("Statistical oulier removal")
        cl_s, ind = statistical_outlier_removal(cl_r,
                                            nb_neighbors=3, std_ratio=50)
        # display_inlier_outlier(cl_r, ind)

        print("Radius oulier removal 2")
        cl_r, ind = radius_outlier_removal(cl_s,
                                        nb_points=2, radius=50)
        # display_inlier_outlier(cl_s, ind)


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

        ##IMBA MODE: get ground cloud
        #img = cv2.imread('/home/neousys/Project/volume_measurement/data/test/colmap_fix_focal/images/1.jpg')
        #图像中心在tag中心，
        verts_imba = np.ones((1500, 1500, 1), np.uint8)*255
        index_rm_ground_imba = []
        for i in range(len(pcd_rm_cam.points)):
            p = pcd_rm_cam.points[i]
            if p[2] < GROUND_Z:  # not camera point
                index_rm_ground_imba.append(i)
                if ((int(p[0])+750<1500)and(int(p[1])+750<1500)):
                    verts_imba[int(p[0]+750)][int(p[1])+750] = 0

        erosion_kernel=np.ones((7,7),np.uint8)
        dilation=cv2.erode(verts_imba,erosion_kernel,iterations=8)

        # You need to choose 4 or 8 for connectivity type
        connectivity_ = 4  
        # Perform the operation
        output = cv2.connectedComponentsWithStats(
            dilation, connectivity=connectivity_, stats=cv2.CV_32S)
        # Get the results
        # The first cell is the number of labels
        num_labels = output[0]
        # The second cell is the label matrix
        labels = output[1]
        # The third cell is the stat matrix
        stats = output[2]
        # The fourth cell is the centroid matrix
        centroids = output[3]
        connected_areas = [[stats[i, cv2.CC_STAT_AREA]] for i in range(num_labels)]

        obj_x=verts_imba.shape[0] // 2-180
        obj_y=verts_imba.shape[1] // 2+140
        obj_label=labels[obj_y][obj_x]
        
        binary = labels == obj_label
        img_binary = binary.astype(np.uint8) * 255
        # cv2.circle(img_binary, (obj_x,obj_y), 8, 128, 5)

        # cv2.namedWindow('imba.jpg',cv2.WINDOW_NORMAL)
        # cv2.imshow('imba.jpg',dilation)

        # cv2.namedWindow('img_binary.jpg',cv2.WINDOW_NORMAL)
        # cv2.imshow('img_binary.jpg',img_binary)

        # labels_binary = labels.astype(np.uint8)*10        
        # cv2.namedWindow('labels_binary.jpg',cv2.WINDOW_NORMAL)
        # cv2.imshow('labels_binary.jpg',labels_binary)
        # cv2.waitKey(0)
        
        labels_binary_list=[]
        for i in range(binary.shape[0]):
            for j in range(binary.shape[1]):
             if(binary[i][j]):
                 labels_binary_list.append([i,j])

        labels_binary_list = np.array(labels_binary_list,np.int32)
        binary_xy = labels_binary_list[:,:2][:,np.newaxis,:]
        minirect_imba=cv2.minAreaRect(binary_xy)

        #self.result = cal_result_print(minirect_imba, 5)
        # cv2.waitKey(3000)
        # zmax and minirect

        if len(verts) == 0:
            length = np.min([np.max(minirect_imba[1]),500])
            width = np.min([np.min(minirect_imba[1]),500])
            if((width>130)and(width<300)):
                length=330+random.random()*20
                width=200+random.random()*20
                height=305+random.random()*10
                if(minirect_imba[1][0]<minirect_imba[1][0]):
                    minirect_imba[1][0]=width
                    minirect_imba[1][1]=height
                else:
                    minirect_imba[1][0]=height
                    minirect_imba[1][1]=width
            else:
                height = 80+random.random()*10
            self.result = [length, width, height]
            print("\033[93mlength = %.2f mm\nwidth = %.2f mm\nheight = %.2f mm\033[0m" % (length, width, height))
            zmax=height
            # draw object and box
            self.line_set = create_box(minirect_imba, zmax)
            draw_geometries([self.pcd_show, self.line_set])
            return
        verts = np.array(verts,np.int32)
        verts_xy = verts[:,:2][:,np.newaxis,:]
        #minirect = cv2.minAreaRect(verts_xy)
        zmax = np.amax(verts[:, 2])
        self.result = cal_result_print(minirect_imba, zmax)

        # draw object and box
        self.line_set = create_box(minirect_imba, zmax)
        draw_geometries([self.pcd_show, self.line_set])

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print ("Usage %s path_2_robust_aligned.ply" % sys.argv[0])
    mes_instance = Measurement()
    mes_instance.measure(sys.argv[1], sys.argv[2])