# 点云+图像融合后续调试

## 1. 外参和内参标定更新

相机和雷达外参标定后需更新至/config/single_calib_result.txt, 格式请参考该文件，直接改数字即可。
内参可以到launch里改

## 2. image消息读取和缓存设计

picture_filter node 读取<sensor_msgs::msg::Image>,话题名“/image_raw”(参考[livox_ros2_driver/lidar_camera_fusion/src/fusion_node.cpp](https://github.com/hellokea771/livox_ros2_driver/blob/master/lidar_camera_fusion/src/fusion_node.cpp))，按相机输出图像消息频率**100HZ**估算（实际上image_raw频率小于相机-ros驱动显示的fps），缓存长度**200**,存储**约2s**内图像消息，并参考small-point-lio输出点云的时间戳找出最近时间的图像消息发送，并把**时间戳和对应的点云话题对齐**

## 3. CloudToLidarNode 的TF缓存策略

能够读取的tf应与接收到的点云的时间戳相差不超过tf_timeout,可以在launch文件里调整，默认为0.2s

## 4. 可以读取的图像格式为CV_8UC3（rgb8或bgr8均可）/CV_8UC1（mono8），输出的融合点云在"livox_frame"下


# 注意事项

## 1. 启用重力对齐但尚未完成时，small-point-lio会跳过发布tf和点云，影响CloudToLidarNode的工作，在此期间请忽略该节点输出的sensor_msgs::msg::PointCloud2 out_msg。

## 2. build前检查：
image话题“/image_raw",如果相机图像话题改名字，请至/launch/small_point_lio_fusion.launch.py处传入参数或至src/picture_filter.cpp更改"image_in"默认传入参数。
订阅的雷达点云话题是small-point-lio发布的odom系中的/cloud_registered，odom>lidar_frame的转换会查询small-point-lio的tf树，最后用联合标定的参数转换到相机坐标系，发布的融合点云的坐标系名为"livox_frame"，即雷达坐标系或small-point-lio中的lidar_frame。如small-point-lio最终发布的点云话题更改名字，请在build前也至/launch/small_point_lio_fusion.launch.py传入新参数或至src/picture_filter.cpp和src/cloud_to_lidar_colorize.cpp中修改默认传入参数。

## 3.终端使用 ros2 launch small-point-lio small_point_lio_fusion.launch.py run_lio:=false 可以仅连接相机和雷达测数据融合。
需要把cloud_to_lidar_colorize.cpp中的内容换成backup.txt中内容，直接读取“/livox/lidar”的点云话题，同理，src/picture_filter.cpp中订阅的点云话题名也要更改，最后需要检查launch为点云融合与图像筛选节点传入的雷达话题名是否也更改了。

## 4.src/picture_filter.cpp中图像和点云的匹配时间戳相差阈值设置为max_dt=0.3,可以在launch里更改。
