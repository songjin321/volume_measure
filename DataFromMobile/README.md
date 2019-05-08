# 文件格式约定
dinoSpareRing

    ---IMAGE
        --- 0.png
        --- ....
        --- img_num.png

    --- image_time.txt   image timestamp, one line for each image
    
    --- imudataWithTimestamp.txt    imu data with timestamp

    --- pose.txt                    camera parameters. There is one line for each image.  The format for each line is:
	"k11 k12 k13 k21 k22 k23 k31 k32 k33 r11 r12 r13 r21 r22 r23 r31 r32 r33 t1 t2 t3"
	The projection matrix for that image is given by K*[R t]
	The image origin is top-left, with x increasing horizontally, y vertically

    --- segmentation_mask.npy  A numpy array with shape [img_num, img_heigth, img_width], you can save a numpy arrary wiht np.save() function. the foreground(interested object) should be assigned to 1,the background should be assigned to 0.