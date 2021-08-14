#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件

std::string file_directory;
std::string file_name;
std::string pcd_file;

std::string map_topic_name;

const std::string pcd_format = ".pcd";

nav_msgs::OccupancyGrid map_topic_msg;

double thre_z_min = 0.3;
double thre_z_max = 2.0;
int flag_pass_through = 0;

double grid_x = 0.1;
double grid_y = 0.1;
double grid_z = 0.1;

double map_resolution = 0.05;

double thre_radius = 0.1;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);

void PassThroughFilter(const double& thre_low, const double& thre_high, const bool& flag_in);

void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_cloud, const double &radius, const int &thre_count);

void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid& msg);

int main(int argc, char** argv)
{
   ros::init(argc, argv, "pcl_filters");
   ros::NodeHandle nh;
   ros::NodeHandle private_nh("~");

   ros::Rate loop_rate(1.0);

   private_nh.param("file_directory", file_directory, std::string("/home/ubuntu/"));
   ROS_INFO("*** file_directory = %s ***\n", file_directory.c_str());


   private_nh.param("file_name", file_name, std::string("good2"));
   ROS_INFO("*** file_name = %s ***\n", file_name.c_str());

   pcd_file = file_directory + file_name + pcd_format;
   ROS_INFO("*** pcd_file = %s ***\n", pcd_file.c_str());

   private_nh.param("thre_z_min", thre_z_min, 0.5);
   private_nh.param("thre_z_max", thre_z_max, 2.0);
   private_nh.param("flag_pass_through", flag_pass_through, 0);
   private_nh.param("grid_x", grid_x, 0.1);
   private_nh.param("grid_y", grid_y, 0.1);
   private_nh.param("grid_z", grid_z, 0.1);
   private_nh.param("thre_radius", thre_radius, 0.5);
   private_nh.param("map_resolution", map_resolution, 0.05);
   private_nh.param("map_topic_name", map_topic_name, std::string("map"));

   ros::Publisher map_topic_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1);

   if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *pcd_cloud) == -1)
   {
     PCL_ERROR ("Couldn't read file: %s \n", pcd_file.c_str());
     return (-1);
   }

   std::cout << "初始点云数据点数：" << pcd_cloud->points.size() << std::endl;

   PassThroughFilter(thre_z_min, thre_z_max, bool(flag_pass_through));

//   RadiusOutlierFilter(cloud_after_PassThrough, 0.1, 10);
//   SetMapTopicMsg(cloud_after_Radius, map_topic_msg);

   SetMapTopicMsg(cloud_after_PassThrough, map_topic_msg);

   while(ros::ok())
   {
     map_topic_pub.publish(map_topic_msg);

     loop_rate.sleep();

     ros::spinOnce();
   }

   return 0;
}

void PassThroughFilter(const double &thre_low, const double &thre_high, const bool &flag_in)
{
    /*方法一：直通滤波器对点云进行处理。*/
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    passthrough.setInputCloud(pcd_cloud);//输入点云
    passthrough.setFilterFieldName("z");//对z轴进行操作
    passthrough.setFilterLimits(thre_low, thre_high);//设置直通滤波器操作范围
    passthrough.setFilterLimitsNegative(flag_in);//true表示保留范围外，false表示保留范围内
    passthrough.filter(*cloud_after_PassThrough);//执行滤波，过滤结果保存在 cloud_after_PassThrough
    std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough->points.size() << std::endl;
}

void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd_cloud0, const double &radius, const int &thre_count)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;  //创建滤波器

    radiusoutlier.setInputCloud(pcd_cloud0);    //设置输入点云
    radiusoutlier.setRadiusSearch(radius);     //设置radius为100的范围内找临近点
    radiusoutlier.setMinNeighborsInRadius(thre_count); //设置查询点的邻域点集数小于2的删除

    radiusoutlier.filter(*cloud_after_Radius);
    std::cout << "半径滤波后点云数据点数：" << cloud_after_Radius->points.size() << std::endl;
}

void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid& msg)
{
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = map_resolution;

  double x_min, x_max, y_min, y_max;
  double z_max_grey_rate = 0.05;
  double z_min_grey_rate = 0.95;
  double k_line = (z_max_grey_rate - z_min_grey_rate) / (thre_z_max - thre_z_min);
  double b_line = (thre_z_max * z_min_grey_rate - thre_z_min * z_max_grey_rate) / (thre_z_max - thre_z_min);

  if(cloud->points.empty())
  {
    ROS_WARN("pcd is empty!\n");

    return;
  }

  for(int i = 0; i < cloud->points.size() - 1; i++)
  {
    if(i == 0)
    {
      x_min = x_max = cloud->points[i].x;
      y_min = y_max = cloud->points[i].y;
    }

    double x = cloud->points[i].x;
    double y = cloud->points[i].y;

    if(x < x_min) x_min = x;
    if(x > x_max) x_max = x;

    if(y < y_min) y_min = y;
    if(y > y_max) y_max = y;
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = int((x_max - x_min) / map_resolution);
  msg.info.height = int((y_max - y_min) / map_resolution);

  msg.data.resize(msg.info.width * msg.info.height);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  ROS_INFO("data size = %d\n", msg.data.size());

  for(int iter = 0; iter < cloud->points.size(); iter++)
  {
    int i = int((cloud->points[iter].x - x_min) / map_resolution);
    if(i < 0 || i >= msg.info.width) continue;

    int j = int((cloud->points[iter].y - y_min) / map_resolution);
    if(j < 0 || j >= msg.info.height - 1) continue;

    msg.data[i + j * msg.info.width] = 100;
//    msg.data[i + j * msg.info.width] = int(255 * (cloud->points[iter].z * k_line + b_line)) % 255;
  }
}


