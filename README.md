# 基于移动设备的物体体积快速测量
利用市面常见手机的摄像头和一个打印的标志物快速测量物体的体积

## 主方案
利用sfm重建整个场景，通过tag修正尺度和坐标系，最后利用点云滤波滤除去外点并计算体积
### 安装
1. 安装openMVG [pull requests1239](https://github.com/openMVG/openMVG/pull/1239)
2. 安装opencv-contrib-python，open3D, watchdogs
3. 修改sfm_pipeline中的sensor_width_database路径，并在sensor_width_camera_database.txt中加入一行Mi Note 3;3.78.
### 使用说明
1 打印data/aruco_marker_board.pdf文件，并围绕物体各个角度拍摄约30张图片，如shoe_tag/image所示

2 运行三维重建脚本
```bash
python src/sfm/sfm_pipeline.py data/shoe_tag/image data/shoe_tag
```
最后生成的robust_aligned.ply为尺度修正的，坐标系对齐到tag board上的点云。

3 对上一步生成的点云进行离群点处理并计算体积
```bash
python src/sfm/measurement.py data/shoe_tag/reconstruction_sequential/robust_aligned.ply
```

4（optional）结合手机上的web界面,将手机采集的数据保存在online_data文件夹中，然后在这里进行在线实时重建
- 监测到新建一个文件夹ms*/image,开始测量
- 当image中文件图片数量达到20张开始重建，然后新加5张图片，重新进行一次测量

```bash
cd data/online_data
python3 ../../src/tcc/html/tcc-backend/uploadServer.py

cd src/tcc/html/tcc-frontend
HTTPS=true npm start

在手机浏览器上打开chrome，连上同一个局域网，输入电脑ip地址加3000，使用https，https:192.168.1.201:3000。按一下connect server，连上服务器，刷新一下表示重新测量

开启在线三维重建,记得将相机内参写到相对应的位置,每次测量显示后按ESC退出显示，不然会阻塞
python3 src/sfm/sfm_online.py data/online_data  

```


## 针对无纹理物体的备选方案

利用灰度共生矩阵对目标物进行分割，利用tag和pnp方法估计相机位姿，利用visual Hull算法对物体进行重建
### 使用说明
1 用visual_hull/imageProcess.py获取mask和相机pose
2 用reconstruction.py进行重建