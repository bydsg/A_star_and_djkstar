#ifndef _HEADFILE_H_
#define _HEADFILE_H_
using namespace std;

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>//引用Ｅｉｇｅｎ库
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <fstream>
#include <algorithm> 

#include "Astar2.h"
#include "Box2d.h"
#include "rviz.h"
#include "BSpine.h"
#include "by_djstl/Path.h"
#include "by_djstl/PathPoint.h"


//加载txt中路径
std::vector<Road> load_direction_Road(const std::string fileName)
{ 
    std::vector<Road> roads;
    bool record_points = false;
    Road road;
    std::string line;
    std::string line3;
    std::ifstream fs;
    fs.open(fileName, std::ios::in);
    while (getline(fs, line)) {
      
        if (line.length() > 0) {
            std::stringstream ss(line);
            std::stringstream sss(line);
            std::string line2;
            int id;
            ss>>line2;
            if (line2 == "road")
            {   
                ss>>id;
                road.id = id;
                continue;
            }
        
            if (line2 == "point")
            {
                record_points = true;
                continue;
            }
            if (line2 != "pre" && record_points && line2!= "lf")
            {
             Point  point ;
             point.l=2;
             point.r=2;  
             ss >> point.x >> point.y>>point.theta;
             road.road_points.push_back(point); 
             continue;
            }

            if (line2 == "lf")
            {
                pair<float, float> lf;
                ss >> lf.first >> lf.second;
                road.lf = lf;
                road.ifmap=1;
                continue;
            }
            if (line2 == "rb")
            {
                pair<float, float> rb;
                ss >> rb.first >> rb.second;
                road.rb = rb;
                continue;
            }

            if (line2 == "pre")
            {
                int count = 0;
                while(getline(sss,line3,' ')){ 
                   count++;
                   if (count>=2)
                   {
                       std::stringstream ssss(line3);
                       int id;
                       ssss>>id;
                       road.pre.push_back(id);
                   }
                }
                record_points = false;
                continue;
            }
 
            if (line2 == "beh")
            {    
                int count = 0;
                while(getline(sss,line3,' ')){ 
                   count++;
                   if (count>=2)
                   {
                       std::stringstream ssss(line3);
                       int id;
                       ssss>>id;
                       road.beh.push_back(id);
                   }
                }
                
                record_points = false;
                roads.push_back(road); 
                road = {0};               
                continue;  
            }     

        }   
    }
    
    fs.close();
    return roads;
}


#endif