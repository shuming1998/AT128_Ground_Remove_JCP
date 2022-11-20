#define PCL_NO_PRECOMPILE //从PCL-1.7开始，您需要定义PCL_NO_PRECOMPILE，然后才能包含任何PCL头文件来包含模板化算
#ifndef MAKE_POINTXYZITR_H
#define MAKE_POINTXYZITR_H

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>
#include <unordered_set>
#include <vector>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <queue>
#include <unordered_map>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace pcl {

struct PointXYZITR {
  PCL_ADD_POINT4D;
  uint8_t intensity;
  double timestamp;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZITR,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
        double, timestamp, timestamp)(uint16_t, ring, ring))

#endif
