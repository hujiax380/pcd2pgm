#include <ros/ros.h>
 
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
 
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
 
#include <pcl/point_types.h>
 
 
std::string file_directory;
std::string file_name;
std::string pcd_file;
 
std::string map_topic_name;
 
const std::string pcd_format = ".pcd";
 
nav_msgs::OccupancyGrid map_topic_msg;
 
double map_resolution = 0.05;
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);
 
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid& msg);
 
int main(int argc, char** argv)
{
   ros::init(argc, argv, "pcl_filters");
   ros::NodeHandle nh;
   ros::NodeHandle private_nh("~");
 
   ros::Rate loop_rate(1.0);
 
   private_nh.param("file_directory", file_directory, std::string("/home/ubuntu/"));//此处需要修改为自己pcd文件的路径
   ROS_INFO("*** file_directory = %s ***\n", file_directory.c_str());
 
 
   private_nh.param("file_name", file_name, std::string("pcd_name"));//此处"pcd_name"需要修改为自己的pcd文件名，无需.pcd
   ROS_INFO("*** file_name = %s ***\n", file_name.c_str());
 
   pcd_file = file_directory + file_name + pcd_format;
   ROS_INFO("*** pcd_file = %s ***\n", pcd_file.c_str());
 
   private_nh.param("map_resolution", map_resolution, 0.05);
   private_nh.param("map_topic_name", map_topic_name, std::string("map"));
 
   ros::Publisher map_topic_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1);
 
   if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *pcd_cloud) == -1)
   {
     PCL_ERROR ("Couldn't read file: %s \n", pcd_file.c_str());
     return (-1);
   }
 
   std::cout << "输入点云点数：" << pcd_cloud->points.size() << std::endl;
 
   SetMapTopicMsg(pcd_cloud, map_topic_msg);
 
   while(ros::ok())
   {
     map_topic_pub.publish(map_topic_msg);
 
     loop_rate.sleep();
 
     ros::spinOnce();
   }
 
   return 0;
}
 
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid& msg)
{
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
 
  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = map_resolution;
 
  double x_min, x_max, y_min, y_max; //这里是投影到xy平面，如果要投到xz/yz，这里以及后面的xy对应的数据改为你想投影的平面
 
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
 
  }
}
