#include "at128_jpc/make_pointxyzitr.h"
#ifndef AT128_JPC_H
#define AT128_JPC_H

#define PI 3.1415926
#define RAD 57.295780

typedef pcl::PointCloud<pcl::PointXYZITR> PointType;
typedef pcl::PointXYZITR Point;

extern int radial_index;
const std::string pts_topic = "/hesai/pandar_points";
const int horizon_pts_num = 1281, scan_num = 128, neighbor_num = 25, s = 5;
const float th_g = 0.2, sigma = 10., delta_R = 2., th_d = 1., max_range = 70,
            min_range = 2;
const int radial_num = int((max_range - min_range) / delta_R);
const Eigen::Vector3d lidar_pos{5.4, -0.2, 3.2};

struct Index {
  int x = 0;
  int y = 0;
};

class ProjectionJpc {
public:
  ProjectionJpc();
  ~ProjectionJpc() {}
  void ptsCopy(const sensor_msgs::PointCloud2::ConstPtr &origin_pts);
  void setParams();
  void memAllocation();
  void mapMake();
  void RECM();
  void JPC();
  void ptsPub();
  void runProject(const sensor_msgs::PointCloud2ConstPtr &pts_msg);
  void pts_trans(PointType::Ptr untrans_pts);

private:
  std_msgs::Header pts_header;

  Point nan_pt;
  PointType::Ptr pts_in;
  PointType::Ptr pts_full;
  PointType::Ptr pts_ground;
  PointType::Ptr pts_obstacle;

  ros::NodeHandle nh;
  ros::Subscriber pts_sub;
  ros::Publisher pub_pts_in;
  ros::Publisher pub_pts_full;
  ros::Publisher pub_pts_ground;
  ros::Publisher pub_pts_obstacle;

  std::vector<float> region_minz;
  std::vector<int> cloud_index;
  std::vector<std::pair<int, int>> neighborIterator;

  cv::Mat range_image;
  cv::Mat region_image;

  Eigen::AngleAxisd rollAngle;
  Eigen::AngleAxisd pitchAngle;
  Eigen::AngleAxisd yawAngle;
  Eigen::Matrix3d rotation_matrix;
  Eigen::Matrix4d trans_matrix;
};
#endif
