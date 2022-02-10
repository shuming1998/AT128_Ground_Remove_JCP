#include"at128_jpc/at128_jpc.h"
using namespace std;
ProjectionJpc::ProjectionJpc() : nh("~")
{
    pts_sub = nh.subscribe<sensor_msgs::PointCloud2>(pts_topic, 10, &ProjectionJpc::runProject, this);

    pub_pts_in = nh.advertise<sensor_msgs::PointCloud2> ("/pts_in", 10);
    pub_pts_full = nh.advertise<sensor_msgs::PointCloud2> ("/pts_full", 10);
    pub_pts_ground = nh.advertise<sensor_msgs::PointCloud2> ("/pts_ground", 10);
    pub_pts_obstacle = nh.advertise<sensor_msgs::PointCloud2> ("/pts_obstacle", 10);
    
    nan_pt.x = std::numeric_limits<float>::quiet_NaN(); 
    nan_pt.y = std::numeric_limits<float>::quiet_NaN();
    nan_pt.z = std::numeric_limits<float>::quiet_NaN();
    //nan_pt.intensity = -1;
    nan_pt.ring = 0;
    
    memAllocation();
    setParams();
}

void ProjectionJpc::setParams()
{
    pts_full->clear();
    pts_ground->clear();
    pts_obstacle->clear();
    pts_in->clear();
    
    range_image = cv::Mat::zeros(scan_num, horizon_pts_num, CV_8UC3);
	region_image = cv::Mat::zeros(scan_num, horizon_pts_num, CV_8UC1);
	
	fill(pts_full->points.begin(), pts_full->points.end(), nan_pt);
}

void ProjectionJpc::memAllocation()
{
    pts_in.reset(new PointType());
    pts_full.reset(new PointType());
    pts_ground.reset(new PointType());
    pts_obstacle.reset(new PointType());
    
    region_minz.assign(radial_num * horizon_pts_num, 100);
	cloud_index.assign(scan_num * horizon_pts_num, -1);
	
	neighborIterator.reserve(neighbor_num);
    for(int i = -2; i <= 2; ++i)
    {
        for(int j = -2; j <= 2; ++j)
        {
            neighborIterator.push_back(make_pair(i, j));
        }
    }
    neighborIterator.erase(neighborIterator.begin() + 12);
    
    pts_full->points.resize(scan_num * horizon_pts_num);
}

void ProjectionJpc::mapMake()
{
    float range(0);
    int row_i(0), col_i(0), index(0), radial_index(0), region_index(0);
    for(const auto pt : pts_in->points)
    {
        //把容器中的点按照索引重新赋值，非空点会替换掉之前用来填充的NAN点。
        if(pt.ring != 0)
        {   
            range = (float)sqrt(pt.x * pt.x + pt.y * pt.y);
            if(range < min_range || range > max_range)
            {
                continue;
            }
            row_i = pt.ring;
            if(atan2(pt.y, pt.x) >= 0)
                col_i = ((128.1/2) - atan2(pt.y, pt.x) * RAD) / 0.1;
            else
                col_i = (-atan2(pt.y, pt.x) * RAD + (128.1 / 2)) / 0.1;
            if(row_i < 0 || row_i >= scan_num)
            {
                continue;
            }
            if(col_i < 0 || col_i >= horizon_pts_num)
            {
                continue;
            }
            //范围外的点和nan点对应的range_image为(0,0,0)
		    range_image.at<cv::Vec3b>(row_i, col_i) = cv::Vec3b(0,255,0);
            index = col_i * scan_num + row_i;

            radial_index = (int)((range-min_range)/delta_R);//该点所在的环数
            region_index = col_i * radial_num + radial_index;//radial_num为每条轴线总环数
            //区域最小z值列表按照1281条线排序，每条线分为radial_num个线段区间,后续判断每条线上不同线段区间里的最小z值的时候，直接循环即可。
		    region_minz[region_index] = min(region_minz[region_index], pt.z);
		    region_image.at<uchar>(row_i, col_i) = radial_index;//region_中存储该点所在环数
		    cloud_index[index] = index;//范围外和nan点的cloud_index为-1     
		    pts_full->points[index] = pt;     
         }      
    }
    //cv::namedWindow("map",CV_WINDOW_NORMAL);
    //cv::imshow("map",range_image);
    //cv::waitKey(1);
}

void ProjectionJpc::RECM()
{
    //遍历region_minz数组,将(m-1,n)和(m,n)联系起来
    for(int i = 0; i < region_minz.size(); ++i)
    {
        if(i % radial_num == 0)
        {
            continue;
        }
        else
        {
            region_minz[i] = min(region_minz[i], region_minz[i-1] + (float)delta_R * tan(sigma));
        }
    }
    //遍历点，如果z值大于对应块内的minz,就判定为障碍点
    for(int row = 0; row < scan_num; ++row)
    {
        for(int col = 0; col < horizon_pts_num; ++col)
        {
            int index_i = col * scan_num + row;
            //忽略nan点和范围外的点
            if(cloud_index[index_i] == -1)
            {
                continue;
            }
            else
            {
                int region_i_index = region_image.at<uchar>(row, col);
                float region_i_height = region_minz[col * radial_num + region_i_index];
                if(pts_full->points[index_i].z >= (region_i_height + th_g))
                {
                    range_image.at<cv::Vec3b>(row, col) = cv::Vec3b(0,0,255);
                }
            }
        }
    }
}

void ProjectionJpc::JPC()
{
    vector<cv::Mat> channels;
    cv::split(range_image, channels);
    cv::Mat core = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    cv::dilate(channels[2],channels[2],core);
    cv::merge(channels, range_image);
    
    queue<Index> pts_need_rejudge;
    for(int row = 0; row < scan_num; ++row)
    {
        for(int col = 0; col < horizon_pts_num; ++col)
        {
            if(range_image.at<cv::Vec3b>(row, col) == cv::Vec3b(0,255,255) && cloud_index[col * scan_num + row] != -1)
            {
                Index pt_judge;
                pt_judge.x = col;
                pt_judge.y = row;
                pts_need_rejudge.push(pt_judge);
                range_image.at<cv::Vec3b>(row, col) = cv::Vec3b(255,0,0);
            }
        }
    }
    while(!pts_need_rejudge.empty())
    {
        Index pt_judge = pts_need_rejudge.front();
        pts_need_rejudge.pop();
        
        Eigen::VectorXf D(24);
        int pt_id = pt_judge.x * scan_num + pt_judge.y; //pt_judge.x = col、 pt_judge.y = row
		int mask[24];
		float sumD(0), diff(0);
		Point p1, p2;
		for(int i = 0; i < 24; ++i)
		{
		    int neighbor_xi = neighborIterator[i].first + pt_judge.x;  //col
		    int neighbor_yi = neighborIterator[i].second + pt_judge.y; //row
		    int neighbori_id = neighbor_xi * scan_num + neighbor_yi;
		    
		    if(neighbor_xi < 0 || neighbor_xi >= horizon_pts_num || neighbor_yi < 0 || neighbor_yi >= scan_num || cloud_index[neighbori_id] == -1)
		    {
		        D(i) == 0;
		        sumD += D(i);
		        mask[i] = -1;
		        continue;
		    }
		    p1 = pts_full->points[pt_id];
		    p2 = pts_full->points[neighbori_id];
		    diff = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
            if(diff > th_d)
            {
                D(i) = 0;
                sumD += D(i);
            }
            else
            {
                D(i) = exp(-s * diff);
                sumD += D(i);
            }
			if(range_image.at<cv::Vec3b>(neighbor_yi, neighbor_xi) == cv::Vec3b(255,0,0)) //range内低置信度的点
			{
				mask[i] = 2;
			}
			else if(range_image.at<cv::Vec3b>(neighbor_yi, neighbor_xi) == cv::Vec3b(0,255,0)) //接地点
			{
				mask[i] = 1;
			}
			else if(range_image.at<cv::Vec3b>(neighbor_yi, neighbor_xi) == cv::Vec3b(0,0,255))  //障碍点和range之外的低置信度的点
			{
				mask[i] = 0;
			}
		}
		Eigen::VectorXf W(24);
		W = D / sumD;
		float score_r(0), score_g(0);
		for(int i=0; i<D.size(); ++i)
		{
			if(mask[i] == 0)
			{
				score_r += W(i);
			}
			else if(mask[i] == 1)
			{
				score_g += W(i);
			}
		}
		if(score_r > score_g)
		{
			range_image.at<cv::Vec3b>(pt_judge.y, pt_judge.x) = cv::Vec3b(0,0,255);
		}
		else
		{
			range_image.at<cv::Vec3b>(pt_judge.y, pt_judge.x) = cv::Vec3b(0,255,0);
		}
    }
}

void ProjectionJpc::ptsPub()
{
    for(int row = 0; row < scan_num; ++row)
    {
        for(int col = 0; col < horizon_pts_num; ++col)
        {
            int index = col * scan_num + row;
            if(cloud_index[index] == -1)
            {
                continue;
            }
            if(range_image.at<cv::Vec3b>(row, col) == cv::Vec3b(0, 255, 0))
            {
                pts_ground->push_back(pts_full->points[index]);
            }
            if(range_image.at<cv::Vec3b>(row, col) == cv::Vec3b(0, 0, 255))
            {
                pts_obstacle->push_back(pts_full->points[index]);
            }            
        }
    }    
    sensor_msgs::PointCloud2 ros_pts_in;
    sensor_msgs::PointCloud2 ros_pts_full;
    sensor_msgs::PointCloud2 ros_pts_ground;
    sensor_msgs::PointCloud2 ros_pts_obstacle;

    //if (pub_pts_in.getNumSubscribers()) 
    //{
    pcl::toROSMsg(*pts_in, ros_pts_in);
    ros_pts_in.header.stamp = pts_header.stamp;
    ros_pts_in.header.frame_id = "map";
    pub_pts_in.publish(ros_pts_in);
    //}

    pcl::toROSMsg(*pts_full, ros_pts_full);
    ros_pts_full.header.stamp = pts_header.stamp;
    ros_pts_full.header.frame_id = "map";
    pub_pts_full.publish(ros_pts_full);    
    
    //if (pub_pts_ground.getNumSubscribers()) 
    //{
    pcl::toROSMsg(*pts_ground, ros_pts_ground);
    ros_pts_ground.header.stamp = pts_header.stamp;
    ros_pts_ground.header.frame_id = "map";
    pub_pts_ground.publish(ros_pts_ground);
    //}
           
    //if (pub_pts_obstacle.getNumSubscribers()) 
    //{
    pcl::toROSMsg(*pts_obstacle, ros_pts_obstacle);
    ros_pts_obstacle.header.stamp = pts_header.stamp;
    ros_pts_obstacle.header.frame_id = "map";
    pub_pts_obstacle.publish(ros_pts_obstacle);
    //}
}

//点云格式转换
void ProjectionJpc::ptsCopy(const sensor_msgs::PointCloud2::ConstPtr& origin_pts)
{
    pts_header = origin_pts->header;
    pcl::fromROSMsg(*origin_pts, *pts_in);
}

//点云位姿转换
void ProjectionJpc::pts_trans(PointType::Ptr untrans_pts)
{
    rollAngle = (Eigen::AngleAxisd(0.8/RAD,Eigen::Vector3d::UnitX()));
    pitchAngle = (Eigen::AngleAxisd(5.2/RAD,Eigen::Vector3d::UnitY()));
    yawAngle = (Eigen::AngleAxisd(0.2/RAD,Eigen::Vector3d::UnitZ()));
    rotation_matrix = yawAngle*pitchAngle*rollAngle;
    trans_matrix.block<3,1>(0,3) = lidar_pos;
    trans_matrix.block<3,3>(0,0) = rotation_matrix;
    pcl::transformPointCloud(*untrans_pts,*untrans_pts,trans_matrix);
}

void ProjectionJpc::runProject(const sensor_msgs::PointCloud2ConstPtr& pts_msg)
{
    double timestart = ros::Time::now().toSec();
    ptsCopy(pts_msg);
    mapMake();
    RECM();
    JPC();
    ptsPub();
    setParams();
    memAllocation();
    double timeend = ros::Time::now().toSec();
    
    std::cout << "time = " << timeend - timestart << endl;
}

int main (int argc, char **argv)
{
	ros::init (argc, argv, "at128_jpc");
    ProjectionJpc AT128;
    ros::spin();
    return 0;
}
