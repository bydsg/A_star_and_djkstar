#ifndef _RVIZ_H_
#define _RVIZ_H_
#include "headfile.h"

void  rviz_road6(ros::Publisher marker_pub, vector<vector<Point>> point_vec){
         visualization_msgs::Marker points;
         points.header.frame_id =  "map";
         points.header.stamp = ros::Time::now();
         points.ns = "points_and_lines";
         points.action =  visualization_msgs::Marker::ADD;
         points.pose.orientation.w =  1.0;
         points.id = 0;
         points.type = visualization_msgs::Marker::POINTS;
         // POINTS markers use x and y scale for width/height respectively
         points.scale.x =0.5;
         points.scale.y =0.5;
         // Points are green
         points.color.g = 0;
         points.color.r = 0;
         points.color.b = 1;
         points.color.a = 1.0;
         // Create the vertices for the points 
         for(int i=0;i<point_vec.size();i++){
            for (size_t j = 0; j < point_vec[i].size(); j++){
                geometry_msgs::Point p1;
                p1.y =  point_vec[i][j].y;
                p1.x =  point_vec[i][j].x; 
                p1.z = 0;
                points.points.push_back(p1);   
            }
         }
       
          marker_pub.publish(points);
     }
void  rviz_road5(ros::Publisher marker_pub, vector<vector<Point>> point_vec){
         visualization_msgs::Marker points;
         points.header.frame_id =  "map";
         points.header.stamp = ros::Time::now();
         points.ns = "points_and_lines";
         points.action =  visualization_msgs::Marker::ADD;
         points.pose.orientation.w =  1.0;
         points.id = 0;
         points.type = visualization_msgs::Marker::POINTS;
         // POINTS markers use x and y scale for width/height respectively
         points.scale.x =0.05;
         points.scale.y =0.05;
         // Points are green
         points.color.g = 1.0f;
         points.color.r = 0;
         points.color.b = 0;
         points.color.a = 1.0;
         // Create the vertices for the points 
         for(int i=0;i<point_vec.size();i++){
            for (size_t j = 0; j < point_vec[i].size(); j++){
                geometry_msgs::Point p1;
                p1.y =  point_vec[i][j].y;
                p1.x =  point_vec[i][j].x; 
                p1.z = 0;
                points.points.push_back(p1);   
            }
         }
       
          marker_pub.publish(points);
     }
void  rviz_road3(ros::Publisher marker_pub, vector<Point> rem_point_vec){
         visualization_msgs::Marker points;
         points.header.frame_id =  "map";
         points.header.stamp = ros::Time::now();
         points.ns = "points_and_lines";
         points.action =  visualization_msgs::Marker::ADD;
         points.pose.orientation.w =  1.0;
         points.id = 0;
         points.type = visualization_msgs::Marker::POINTS;
         // POINTS markers use x and y scale for width/height respectively
         points.scale.x = 0.05;
         points.scale.y = 0.05;
         // Points are green
         points.color.r = 1.0f;
         points.color.b = 1.0f;
         points.color.a = 0.1f;
         // Create the vertices for the points 
         for(int i=0;i<map_msg_jubu_vec.size();i++){
  
            for (size_t j = 0; j < map_msg_jubu_vec[i].rem_point_vec.size(); j++){
                    geometry_msgs::Point p1;   
                         p1.x =  map_msg_jubu_vec[i].rem_point_vec[j].x*map_msg_jubu_vec[i].resolution+map_msg_jubu_vec[i].orign_x;
                         p1.y =  map_msg_jubu_vec[i].rem_point_vec[j].y*map_msg_jubu_vec[i].resolution+map_msg_jubu_vec[i].orign_y;
                         p1.z = 0;
                         points.points.push_back(p1);  
          }
         }
        
          marker_pub.publish(points);
     }
void  rviz_road2(ros::Publisher marker_pub, vector<vector<Point>> point_vec){
          visualization_msgs::Marker points;
         points.header.frame_id =  "map";
         points.header.stamp = ros::Time::now();
         points.ns = "points_and_lines";
         points.action =  visualization_msgs::Marker::ADD;
         points.pose.orientation.w =  1.0;
         points.id = 0;
         points.type = visualization_msgs::Marker::POINTS;
         // POINTS markers use x and y scale for width/height respectively
         points.scale.x = 0.05;
         points.scale.y =0.05;
         // Points are green
         points.color.g = 0.5f;
         points.color.r = 0.5f;
         points.color.b = 1.0f;
         points.color.a = 1.0;
         // Create the vertices for the points 
         for(int i=0;i<point_vec.size();i++){
            for (size_t j = 0; j < point_vec[i].size(); j++){
                geometry_msgs::Point p1;
                p1.x =  point_vec[i][j].y;
                p1.y =  point_vec[i][j].x; 
                p1.z = 0;
                points.points.push_back(p1);
            }
         }
     
          marker_pub.publish(points);
     }
void  rviz_road4(ros::Publisher marker_pub, vector<vector<Point>> point_vec){
         visualization_msgs::Marker points;
         points.header.frame_id =  "map";
         points.header.stamp = ros::Time::now();
         points.ns = "points_and_lines";
         points.action =  visualization_msgs::Marker::ADD;
         points.pose.orientation.w =  1.0;
         points.id = 0;
         points.type = visualization_msgs::Marker::POINTS;
         // POINTS markers use x and y scale for width/height respectively
         points.scale.x = 0.05;
         points.scale.y = 0.05;
         // Points are green
         points.color.g = 0.0f;
         points.color.r = 1;
         points.color.b = 0;
         points.color.a = 1.0;
         // Create the vertices for the points 
          for(int i=0;i<point_vec.size();i++){
            for (size_t j = 0; j < point_vec[i].size(); j++){
                geometry_msgs::Point p1;
                p1.y =  point_vec[i][j].y;
                p1.x =  point_vec[i][j].x; 
                p1.z = 0;
                points.points.push_back(p1);   
            }
         }
          marker_pub.publish(points);
     }
void  all_door_rviz(ros::Publisher marker_pub, vector<Box_2d> vel_obs_info)
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "odom";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.scale.x = 0.2;
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;
    geometry_msgs::Point p;
    for (uint32_t i = 0; i < vel_obs_info.size(); i++)
    {
        for (uint32_t j = 0; j < vel_obs_info[i].Box2d_corner.size(); j++)
        {
            double y = vel_obs_info[i].Box2d_corner[j].y;
            double x = vel_obs_info[i].Box2d_corner[j].x;
            float z = 0;
            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = z;
            line_list.points.push_back(p);
        }
    }
    marker_pub.publish(line_list);
}
#endif