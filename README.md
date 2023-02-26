# AT128_ground_remove_JCP
基于论文<<Fast Ground Segmentation for 3D LiDAR Point Cloud Based on Jump-Convolution-Process>>
原论文github链接为：https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC

重构了一下代码，以适配禾赛128线激光雷达
数据采用禾赛128线激光雷达录制的rosbag，分割地面，在ubuntu18.04 ROS-Melodic下可直接编译运行。
rosbag百度云下载地址：https://pan.baidu.com/s/19pUUNX0xZJi4spdRDnJXPA?pwd=6zus   提取码：6zus
下载后将zip包解压缩为AT128.bag

第一个终端：
source devel/setup.bash
rosrun at128_jpc at128_jpc

第二个终端:
rosbag play AT128.bag

第三个终端:
rviz

