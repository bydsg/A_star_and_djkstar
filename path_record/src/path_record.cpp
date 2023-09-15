#include "path_record.hpp"
#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>
#include <fstream>
#include <ros/package.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sys/time.h>
#include <time.h>
#include <cmath>
#include <iostream>
#include<sstream>
#include<string>
#include <unistd.h>
using namespace std;

path_record::path_record(/* args */)
{
}

path_record::~path_record()
{
}


void path_record::record_path(const nav_msgs::Odometry::ConstPtr odometry_msg){
  char p;
  p='c';
   if (this->count == 0)
   {
    double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      std::string roadMap_path = "/home/by/code/A_star_by_ws/data/1.txt";//地图录制位置
      FILE *fp_s;
      fp_s = fopen("/home/by/code/A_star_by_ws/data/1.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf", p,odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      start_x =  odometry_msg->pose.pose.position.x;
      start_y =  odometry_msg->pose.pose.position.y;
      this->count++;
   }else{
    cout<<"qq"<<endl;
    sample_distance = sample_distance+sqrt(pow((odometry_msg->pose.pose.position.x - start_x), 2) + pow((odometry_msg->pose.pose.position.y- start_y), 2) );
    start_x = odometry_msg->pose.pose.position.x;
    start_y = odometry_msg->pose.pose.position.y;
      
    if ( sample_distance >= S0)
    {
      double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      FILE *fp_s;
      fp_s = fopen("/home/by/code/A_star_by_ws/data/1.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf",p, odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      sample_distance = 0;
      this->count++;      
    } 
   }  
}

void path_record::record_path_GPS(const geometry_msgs::PoseStamped::ConstPtr odometry_msg){
  cout<<"qqqqqqqqqqqqqq"<<endl;
  char p;
  p='c';
   if (this->count == 0)
   {
    double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      std::string roadMap_path = "/home/lwh/Lpp_files/data/road_path6.txt";//地图录制位置
      FILE *fp_s;
      fp_s = fopen("/home/lwh/Lpp_files/data/road_path6.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf", p,odometry_msg->pose.position.x, odometry_msg->pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      start_x =  odometry_msg->pose.position.x;
      start_y =  odometry_msg->pose.position.y;
      this->count++;
   }else{
    sample_distance = sample_distance+sqrt(pow((odometry_msg->pose.position.x - start_x), 2) + pow((odometry_msg->pose.position.y- start_y), 2) );
    start_x = odometry_msg->pose.position.x;
    start_y = odometry_msg->pose.position.y;
       ROS_INFO_STREAM (" /////////////////////// sample_distance   == "<< sample_distance   ); 
       ROS_INFO_STREAM (" ///////////////////////  start_x    == "<<  start_x  ); 
        ROS_INFO_STREAM (" ///////////////////////  odometry_msg->pose.pose.position.x     == "<<  odometry_msg->pose.position.x   ); 
    if ( sample_distance >= S0)
    {
      double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      FILE *fp_s;
      fp_s = fopen("/home/lwh/Lpp_files/data/road_path6.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf",p, odometry_msg->pose.position.x, odometry_msg->pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      sample_distance = 0;
      this->count++;      
    } 
   }  
}




void path_record::record_path2(const nav_msgs::Odometry::ConstPtr odometry_msg){
     cout<<"/////start_path_record////////"<<endl;
        ros::Rate loop_rate(10);
      char p;
     p='c';
     double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      FILE *fp_s;
      fp_s = fopen("/home/lwh/Lpp_files/data/road_path3.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf", p, odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      loop_rate.sleep();
     
}


void path_record::nodeStart(int argc, char **argv){
    ros::init(argc, argv, "path_record");
    ros::NodeHandle nc;
     count = 0;
    // 订阅相关节点
     // ros::Subscriber sub_odom = nc.subscribe("/aft_mapped_to_init", 1, &path_record::record_path, this);//录制轨迹话题雷达
      ros::Subscriber sub_odom = nc.subscribe("/odom", 1, &path_record::record_path, this);//录制轨迹话题小车里程计
    //  ros::Subscriber sub_globle_path = nc.subscribe("globle_path_by_two", 1, &path_record::globlepathGetCallBack, this); // 接受local规划路径
    // ros::Subscriber sub_odom = nc.subscribe("/rear_post", 1, &path_record::record_path_GPS, this);//录制轨迹话题GPS
      ros::spin();
}

int main(int argc, char *argv[])
{
    
    path_record node;
    node.nodeStart(argc, argv);
    return(0);
}
