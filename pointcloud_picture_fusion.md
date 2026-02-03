# 点云+图像融合后续调试

## 1. 外参标定更新

相机和雷达外参标定后需更新至/config/single_calib_result.txt, 格式请参考该文件。

## 2. image消息读取和缓存设计

picture_filter node 读取<sensor_msgs::msg::Image>,话题名“/image_raw”(参考[livox_ros2_driver/lidar_camera_fusion/src/fusion_node.cpp](https://github.com/hellokea771/livox_ros2_driver/blob/master/lidar_camera_fusion/src/fusion_node.cpp))，按相机输出频率**100HZ**估算，缓存长度**200**,存储**约2s**内图像消息，并参考small-point-lio输出点云的时间戳找出最近时间的图像消息发送，并把**时间戳和对应的点云话题对齐**

## 3. CloudToLidarNode 的TF缓存策略

能够读取的tf应与接收到的点云的时间戳相差不超过tf_timeout,可以在launch文件里调整，默认为0.2s

# 注意事项

启用重力对齐但尚未完成时，small-point-lio会跳过发布tf和点云，影响CloudToLidarNode的工作，在此期间请忽略该节点输出的sensor_msgs::msg::PointCloud2 out_msg。
