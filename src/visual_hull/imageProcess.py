#! /usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np
import sys
from skimage.feature import greycomatrix,greycoprops
from skimage import io
from skimage.util.shape import view_as_blocks
from skimage.transform import rescale
from skimage import img_as_ubyte
import apriltag
import cv2
import heapq
import ctypes
from path import Path
import os
import multiprocessing

_MULTIPLUE_THREAD_=True
#k1,k2,p1,p2,k3
distortion_v8=[-0.0433822460837706,0.0879819223500301,0,0,0]
intrinsicMatrix_v8 = [[2935.16991949835, 0, 1985.95161820373],
                    [0, 3020.91951392312, 1708.91563869920],
                    [0, 0, 1]]
show_img = False
show_outlier = False
patch_size = 32
sample_num = 15
osu_threshold_factor=0
tag_size = 0.162
tag_gap = 0.5 + tag_size

tag_pose= {
    400: [-tag_gap/2, tag_gap/2, 0],
    401: [tag_gap/2, tag_gap/2, 0],
    402: [-tag_gap/2, -tag_gap/2, 0],
    403: [tag_gap/2, -tag_gap/2, 0]
}

GLCM_distance = [patch_size / 4]
GLCM_angle = [0, np.pi / 4, np.pi / 2, np.pi * 3 / 4]

def pool_f(in_bolck):
    in_bolck = in_bolck.reshape(patch_size, patch_size)
    g = greycomatrix(in_bolck, GLCM_distance, GLCM_angle,
                        levels=8, symmetric=True, normed=True)
    contrast = greycoprops(g, 'contrast')
    #homogeneity = greycoprops(g, 'homogeneity')
    return np.average(contrast)

def deblur(image):
    dst = np.zeros_like(image)
    dst_deblur = np.zeros_like(image)

    median = cv2.medianBlur(image, 3)
    cv2.normalize(median, dst, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U) 
    #kernel = np.array([[0, -1.5, 0], [-1.5, 7, -1.5], [0, -1.5, 0]], np.float32) #锐化
    kernel = np.array([[0, -1.5, 0], [-1.5, 7, -1.5],
                       [0, -1.5, 0]], np.float32)  # 锐化
    dst_deblur = cv2.filter2D(dst, -1, kernel=kernel)
    #showimg(dst_deblur)
    #        #计算灰度直方图
    #     grayHist = cv2.calcHist([median], [0], None, [256], [0, 256])
    #     grayHist1 = cv2.calcHist([dst], [0], None, [256], [0, 256])
    #     #画出直方图
    #     x_range = range(256)
    #     plt.plot(x_range, grayHist, 'r', linewidth=1.5, c='black')
    #     plt.plot(x_range, grayHist1, 'r', linewidth=1.5, c='b')
    #     #设置坐标轴的范围
    #     y_maxValue = np.max(grayHist)
    #     plt.axis([0, 255, 0, y_maxValue]) #画图范围
    #     plt.xlabel("gray Level")
    #     plt.ylabel("number of pixels")
    #     plt.show()
    return dst_deblur

def showimg(image):
    plt.imshow(image)
    plt.show()

def findSecondLargeIndexWithMask(num_list,mask):
    two_index = 0
    one_index=0
    one=num_list[0]
    two = num_list[0]
    get_num=0
    for i in range(len(num_list)):
        if mask[i]:
            if get_num==0:
                one = num_list[i]
                one_index = i
                get_num = get_num+1
            elif get_num == 1:
                two = num_list[i]
                two_index = i
                if two > one:
                    one, two = two, one
                    one_index, two_index = two_index, one_index
                get_num = get_num + 1
                start_i=i+1;
                break
    if get_num < 2: return 0,0, False
    for i in range(start_i, len(num_list)):
        if mask[i]:
            if num_list[i] > one:
                two_index=one_index
                two=one
                one = num_list[i]
                one_index=i
            elif num_list[i]>two:
                    two = num_list[i]
                    two_index = i
            else:
                pass
    return one_index,two_index, True
 
def detectionCameraPose(cam_i,tags, intrinsicMatrix,distortion):
    P_corner_bias = np.array([[-tag_size/2, tag_size/2, 0], [tag_size/2, tag_size/2, 0], [tag_size/2, -tag_size/2, 0], [-tag_size/2, -tag_size/2, 0]])
    P_corner = []
    p_corner = []
    for tag in tags:
        P_tag = tag_pose.get(tag.tag_id)
        P_corner_=np.array([P_tag, P_tag, P_tag, P_tag])+P_corner_bias
        P_corner.extend(P_corner_)
        p_corner.extend(tag.corners)
    P_corner = np.array(P_corner)
    p_corner = np.array(p_corner)

    #P_corner = P_corner.reshape(16, 1, 3)
    #p_corner = p_corner.reshape(16, 1, 2)
    # P_corner = P_corner*1000
    # img_Point = np.zeros((1000, 1000),np.uint8)
    # i = 0
    # cv2.namedWindow("img_Point",cv2.WINDOW_NORMAL)
    # for P in P_corner:
    #     i=i+1
    #     cv2.circle(img_Point, (int(P[0]+500), int(-P[1]+500)), 8, 255, 5)
    #     cv2.putText(img_Point, str(
    #         i), (int(P[0]+500), int(-P[1]+500)), cv2.FONT_HERSHEY_DUPLEX, 3, 255)
    #     cv2.imshow('img_Point', img_Point)
    #     cv2.waitKey(500)
    # img_Point = np.zeros((3000, 4000), np.uint8)
    # for P in p_corner:
    #     i = i+1
    #     cv2.circle(img_Point, (int(P[0]), int(P[1])), 8, 255, 5)
    #     cv2.putText(img_Point, str(
    #         i), (int(P[0]), int(P[1])), cv2.FONT_HERSHEY_DUPLEX, 3, 255)
    #     cv2.imshow('img_Point', img_Point)
    #     cv2.waitKey(500)

    output_file = 'points.txt'
    if cam_i == 0:
        f = open(output_file, 'w')
    else:
        f = open(output_file, 'a')
    for i in range(16):
        f.write('%f %f %f %f %f\n'
                % (P_corner[i][0], P_corner[i][1], P_corner[i][2], p_corner[i][0], p_corner[i][1]))
    f.close()


    ret, rvec, tvec = cv2.solvePnP(
        P_corner, p_corner, intrinsicMatrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    rotation_matrix = np.zeros(shape=(3, 3))
    cv2.Rodrigues(rvec, rotation_matrix)
    # new_P_corner = np.ones((16, 4))
    # new_P_corner[:,:3] = P_corner

    # T = np.eye(4)
    # T[:3,:3] = rotation_matrix
    # T[:3, 3] = tvec.flatten()
    # T = np.linalg.inv(T)
    
    #rotation_matrix = T[:3,:3]
    #tvec = T[:3, 3]
    #new_p_corner = intrinsicMatrix@T[:3, :]@new_P_corner.T
    return ret, rotation_matrix, tvec

def write_pose(output_path, index, intrinsic, R, t):
    output_file = output_path / 'pose.txt'
    f=[]
    if index == 0:
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        f = open(output_file, 'w')
    else:
        f = open(output_file, 'a')
    f.write('%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n' \
    % (intrinsic[0, 0], intrinsic[0, 1], intrinsic[0, 2], \
        intrinsic[1, 0], intrinsic[1, 1], intrinsic[1, 2], \
        intrinsic[2, 0], intrinsic[2, 1], intrinsic[2, 2], \
        R[0, 0], R[0, 1], R[0, 2], \
        R[1, 0], R[1, 1], R[1, 2], \
        R[2, 0], R[2, 1], R[2, 2], \
        t[0],t[1],t[2]))
    f.close()

def write_mask(output_path, index, mask_array):
    output_img_path = output_path+"/img/"
    if index == 0:
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        if not os.path.exists(output_img_path):
            os.makedirs(output_img_path)
        #remove all npy file
        #for i_npy, npy_file in enumerate(output_path.files('*.npy')):
        #    os.remove(npy_file)
    np.save(output_path+"/"+str(index)+".npy", mask_array)
    cv2.imwrite(output_img_path+str(index)+".png", mask_array)
def main(argv=None):
    if argv is None:
        argv = sys.argv
    #img_path="./image/"
    #img_path = argv[1]
    img_path = Path("./image/")
    pose_path = Path("./pose")
    mask_path = Path("./mask")
    
    distortion = np.array(distortion_v8)
    intrinsicMatrix = np.array(intrinsicMatrix_v8)
    i_img_succeed = 0
    
    all_files = [os.path.basename(img_path_)for i_img, img_path_ in enumerate(img_path.files('*.jpg'))]
    all_files.sort(key=lambda x: int(x[:-4]))
    img_num = len(all_files)

    for i_img, img_file_name in enumerate(all_files):
        img_file = img_path+img_file_name
        image_src = np.array(io.imread(img_file, True)*255, np.uint8)
        
        print("segemet image: %s.. %d/%d" %
              (os.path.basename(img_file), i_img+1, img_num))
        image_src = deblur(image_src)
        options = apriltag.DetectorOptions(families='tag36h11',
                                        border=1,
                                        nthreads=8,
                                        quad_decimate=1.0,
                                        quad_blur=1,
                                        refine_edges=True,
                                        refine_decode=False,
                                        refine_pose=False,
                                        debug=False,
                                        quad_contours=True)
        detector = apriltag.Detector(options)
        tags = detector.detect(image_src)
        #get camera pose
        if len(tags) < 4:
            print("Tags detection failed, drop image", i_img)
            continue

        ret, rvec, tvec = detectionCameraPose(
            i_img, tags, intrinsicMatrix, distortion)

        #get tag mark area
        tag_mask = np.zeros(image_src.shape, np.uint8)
        tag_corners = [tags[0].center, tags[1].center, tags[3].center, tags[2].center]
        area_center_src = (tags[0].center + tags[1].center + \
            tags[3].center + tags[2].center) / 4
        area = np.array([tag_corners], dtype=np.int32)
        #get roi
        area_rect=cv2.boundingRect(area)
        x = area_rect[0]
        y = area_rect[1]
        dx = area_rect[2]
        dy = area_rect[3]
        image_ROI = image_src[y:y + dy, x:x + dx]
        area_center_roi = area_center_src - [x, y]
        # for tag in tags:
        #     cv2.circle(image_src, (int(tag.corners[3][0]), int(tag.corners[3][1])),
        #         radius=5, color=255, thickness=40)

        #cv2.circle(
        #    image_ROI, (int(area_center_roi[0]), int(area_center_roi[1])), radius=5, color=255, thickness=20)
        
        #showimg(image_ROI)

        shape0_res = patch_size-image_ROI.shape[0] % patch_size
        shape1_res = patch_size-image_ROI.shape[1] % patch_size

        # add 4 255 row
        image_add = np.zeros(
            (image_ROI.shape[0]+shape0_res, image_ROI.shape[1]+shape1_res), np.uint8)
        image_add[:-shape0_res, :-shape1_res] = image_ROI

        # for 8x8 patch calculate grecomatrix
        image = image_add//32
        image_block = view_as_blocks(image, block_shape=(patch_size, patch_size))
        M = np.shape(image_block)[0]
        N = np.shape(image_block)[1]

        #print("enter")
        pool = []
        if _MULTIPLUE_THREAD_:
            cores = multiprocessing.cpu_count()
            pool = multiprocessing.pool.Pool(processes=cores)
        result_contrast = np.zeros(shape=(M, N))

        # Single thresh
        # for i in range(M):
        #     for j in range(N):
        #         result_contrast[i][j]=pool_f(i,j)
        for i in range(M):
            if _MULTIPLUE_THREAD_:
                result_contrast[i] = np.array(
                    pool.map(pool_f, list(image_block[i]))).flatten()
            else:
                result_contrast[i] = np.array([pool_f(block)
                                           for block in list(image_block[i])]).flatten()
        if _MULTIPLUE_THREAD_:
            pool.close()
            pool.join()
        #print("finish")
        result_contrast *= (255.0/result_contrast.max())
        img_contrast = result_contrast.astype(np.uint8)

        #showimg(img_contrast)

        # how to set this threshold
        from skimage.filters import threshold_minimum
        from skimage.filters import threshold_otsu
        data = img_contrast.ravel()
        data = data[data != 0]
        img_gaussianblur = cv2.GaussianBlur(data, (5, 5), 0)
        otsu_thresh = threshold_otsu(data)
        #otsu_thresh, binary = cv2.threshold(
            #img_gaussianblur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        binary = img_contrast < otsu_thresh*3/5
        #binary = cv2.adaptiveThreshold(
        #    img_contrast, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 51, 5)
        tag_mask=tag_mask<128



        img_binary=binary.astype(np.uint8)*255
        # find object reagion by finding max countour
        c_x = img_binary.shape[0] // 2
        c_y = img_binary.shape[1] // 2

        # You need to choose 4 or 8 for connectivity type
        connectivity_ = 8  
        # Perform the operation
        output = cv2.connectedComponentsWithStats(
            img_binary, connectivity=connectivity_, stats=cv2.CV_32S)
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

        #find out second largest area whose centroids in center reagion
        area_center_downsampled = area_center_roi / patch_size
        area_center_distance = np.array([[np.linalg.norm(centroids[i]-area_center_downsampled)]
                                for i in range(num_labels)])
        area_is_center = area_center_distance < 20 * patch_size / 32
        #rule out outsider by condidering if reagion point inside measurement area
        area_downsampled = (area-[x, y]) // patch_size
        
        #insider_point = np.zeros(num_labels)
        index_counter = np.zeros(num_labels)
        
        index_interval = np.array(connected_areas) // sample_num
        for ly in range(labels.shape[0]):
            for lx in range(labels.shape[1]):
                label = labels[ly][lx]
                if area_is_center[label] == False: continue
                if index_counter[label] == index_interval[label]:
                    index_counter[label] = 0;
                    dist = cv2.pointPolygonTest(
                        area_downsampled, (lx, ly), True)
                    if dist < -15:
                        if show_outlier:
                            cv2.circle(img_contrast, (lx, ly), radius=3, color=255, thickness=1)
                            showimg(img_contrast)
                        area_is_center[label] = False
                        break
                index_counter[label] = index_counter[label]+1
 
        #nlargest=heapq.nlargest(2, range(len(connected_areas)), key=connected_areas.__getitem__)
        largest_label, object_label, findsucceed = findSecondLargeIndexWithMask(
            connected_areas, area_is_center)

        #TODO: check aera
        if largest_label == object_label:
            print("Segmentation failed, drop image", i_img)
            continue
        #showimg(labels)

        binary = labels == object_label

        binary = binary.repeat(patch_size, axis=0)
        binary = binary.repeat(patch_size, axis=1)
        #binary = binary & tag_mask
        # delete final 4 row
        binary = binary[: - shape0_res,: - shape1_res]
        img_binary = binary.astype(np.uint8) * 255
        #median = cv2.medianBlur(img_binary, 33)
        median = img_binary
        binary=median>128

        result_img = np.zeros(image_ROI.shape,np.uint8)
        result_img[binary] = image_ROI[binary]
        
        mask_src = np.zeros(image_src.shape, np.uint8)
        mask_src[y:y + dy, x:x + dx] = binary.astype(int)*255
        # # open and close
        # from skimage.morphology import opening, closing, erosion, dilation
        # from skimage.morphology import square
        # selem = square(30)
        # selem_open = square(15)

        # # close
        # closed = closing(binary, selem)

        # # open
        # opened = opening(closed, selem_open)

        # # close
        # closed = closing(opened, selem)
        
        write_pose(pose_path, i_img_succeed, intrinsicMatrix, rvec, tvec)
        write_mask(mask_path, i_img_succeed, mask_src)
        i_img_succeed = i_img_succeed+1
        if show_img:
            #image_ROI = image_add[y:y+dy, x:x+dx]
            final_img = np.ones(image_src.shape, np.uint8)
            final_img[y:y + dy, x:x + dx] = result_img

            plt.figure(figsize=(30, 90))
            plt.subplot(2, 2, 1)
            plt.imshow(img_contrast, cmap=plt.cm.gray)
            plt.title('contrast')

            plt.subplot(2, 2, 2)
            plt.imshow(labels, cmap=plt.cm.gray)
            plt.title('labels')

            plt.subplot(2, 2, 3)
            image_src = cv2.cvtColor(image_src, cv2.COLOR_BGR2RGB)
            plt.imshow(image_src, cmap=plt.cm.gray)
            plt.title('Original')

            plt.subplot(2, 2, 4)
            final_img = cv2.cvtColor(final_img, cv2.COLOR_BGR2RGB)
            plt.imshow(final_img, cmap=plt.cm.gray)
            plt.title('Result')
            plt.show()
            stoppoint=0
            stoppoint = stoppoint+1

if __name__ == '__main__':
     main()
