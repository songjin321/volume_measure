# 基于移动设备的物体体积快速测量
利用市面常见手机的摄像头和IMU在无标志物的情况下快速测量物体的体积

## 主方案

### 安装
1. 安装openMVG [pull requests1239](https://github.com/openMVG/openMVG/pull/1239)
2. 安装opencv-contrib-python，open3D
3. 修改sfm_pipeline中的sensor_width_database路径，并在sensor_width_camera_database.txt中加入一行Mi Note 3;3.78.
### 使用说明
1 打印data/aruco_marker_board.pdf文件，并围绕物体各个角度拍摄约30张图片，如shoe_tag/image所示
2 运行三维重建脚本
```bash
python src/sfm/sfm_pipeline.py data/shoe_tag/image data/shoe_tag
```
最后生成的robust_aligned.ply为尺度修正的，坐标系对齐到tag board上的点云。
3 对上一步生成的点云进行离群点处理并计算体积
python src/sfm/measurement.py data/shoe_tag/reconstruction_sequential/robust_aligned.ply

## 针对无纹理物体的备选方案
