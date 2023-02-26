# AT128_ground_remove_JCP
基于地面分割论文**<<Fast Ground Segmentation for 3D LiDAR Point Cloud Based on Jump-Convolution-Process>>** 重构了一下代码，以适配禾赛128线激光雷达，论文 github 链接为：https://github.com/wangx1996/Fast-Ground-Segmentation-Based-on-JPC

数据采用禾赛 AT128 激光雷达录制的 rosbag，在ubuntu18.04 ROS-Melodic下可直接编译运行。

rosbag百度云下载地址：https://pan.baidu.com/s/19pUUNX0xZJi4spdRDnJXPA?pwd=6zus   提取码：6zus，下载后将 zip 包解压缩为 AT128.bag


**运行：**

```shell
$ roscore
# 将项目 AT128_Ground_Remove_JCP 放入工作目录 src 下，返回工作目录空间
$ source devel/setup.bash
$ rosrun at128_jpc at128_jpc
```



**播放数据包：**

```shell
$ rosbag play AT128.bag
```



**可视化：**

```shell
$ rviz
```

