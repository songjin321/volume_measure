# 基于移动设备的物体体积快速测量
利用市面常见手机的摄像头和IMU在无标志物的情况下快速测量物体的体积

## 安装
将改进的VINS-mobile在xcode上编译安装，相关安装流程[参考](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile)

## 运行步骤

1. 首先在手机端打开改进的VINS-Mobile程序，放于物体正上方进行初始化

2. 然后将手机围绕物体转一圈，点击界面上的CAP按钮捕捉大致20张图片

3. 将捕捉的图片和VINS估计的相机内外参数拷贝到PC上。在PC上运行三维重构脚本reconstruction.py，得到物体体积

## TODO

1. 利用生成的物体表面计算物体的体积

2. 提高测量精度和速度