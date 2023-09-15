#include "include/headfile.h"
using namespace std;

int u0_gloab=0;
int e0_gloab=0;
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
class globle_planning
{
public:
    ros::Publisher marker_pub;
    ros::Publisher pub_globle_path;
    ros::Publisher globle_path_marker_pub;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_end_odom;
    vector<Road> roads;
    vector<Road> roads_new;
    Point vehicle_pose;
    pair<float, float> start;
    pair<float, float> end;

    void nodeStart(int argc, char **argv);
    void odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg);
    void end_odom_callback(const geometry_msgs::PoseStamped::ConstPtr msg);
    void start_odom_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void low_pass_filter_road(vector<Road> &roads, int num, float max_dis, int road_num);
};
void globle_planning::low_pass_filter_road(vector<Road> &roads,int num,float max_dis,int road_num){
    vector<Point> road_points_new;
    Point p_last;
    p_last.x = roads[road_num].road_points[0].x;
    p_last.y = roads[road_num].road_points[0].y;
    //滤掉大于max_dis m的飘的点
    int count_dis=0;
    for(int i=0;i<roads[road_num].road_points.size();i++){
        Point p;
      
        p.x = roads[road_num].road_points[i].x;
        p.y = roads[road_num].road_points[i].y;
        float dis=sqrt(pow(p.x-p_last.x,2)+pow(p.y-p_last.y,2));
        cout<<dis<<endl;
        if(dis<max_dis){
            road_points_new.push_back(p);
            p_last.x = p.x;
            p_last.y = p.y;
            count_dis=0;
        }else{
            if(count_dis<5){
             count_dis++;  
            }else{
             p_last.x = p.x;
             p_last.y = p.y;
            }           
        }
    }

    int count=0;
    float x_sum=0,y_sum=0;
    vector<Point> road_points_new_new;
    for(int i=2;i<road_points_new.size()-2;i++){//前两个点不绿
        if(count<num){
            x_sum += road_points_new[i].x;
            y_sum += road_points_new[i].y;
            count++;
        }else{
            Point p;
            p.x = x_sum / (float)count;
            p.y = y_sum / (float)count;
            y_sum = 0;
            x_sum = 0;
            road_points_new_new.push_back(p);
            count = 0;
        } 
    }
   
    Road road;
    for (int i = 0; i < road_points_new_new.size(); i++)
    {
        Point p;
        p.x = road_points_new_new[i].x;
        p.y = road_points_new_new[i].y;
        road.road_points.push_back(p);
    }
    int id;
    std::vector<int> pre;
    std::vector<int> beh;
    for(int i=0;i<roads[road_num].beh.size();i++){
        int beh =roads[road_num].beh[i];
        road.beh.push_back(beh);
    }  
    for(int i=0;i<roads[road_num].pre.size();i++){
        int pre =roads[road_num].pre[i];
        road.pre.push_back(pre);
    }
    road.id=roads[road_num].id;

    std::vector<Road>::iterator it = roads.begin() + road_num;
    *it = road;
}

void globle_planning::odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg){
// 得到车辆的定位信息和速度信息
    ros::Rate loop_rate(10);
    rviz_road(this->marker_pub, this->roads);  

    double raw, pitch, theta;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(raw, pitch, theta);
    this->vehicle_pose.theta = theta;
    this->vehicle_pose.x = odometry_msg->pose.pose.position.x;
    this->vehicle_pose.y = odometry_msg->pose.pose.position.y;
    loop_rate.sleep();
}


void globle_planning::start_odom_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  u0_gloab=0;
  double x=msg->pose.pose.position.x;
  double y=msg->pose.pose.position.y;

  start.first=x;
  start.second=y;
  cout<<"绿色箭头所指("<<x<<", "<<y<<")"<<endl;

  for (size_t i = 0; i < this->roads.size(); i++)
  {
    if(roads[i].ifmap){
        if(x>roads[i].lf.first && x<roads[i].rb.first && y<roads[i].lf.second && y>roads[i].rb.second){
            u0_gloab =1+i;
            cout<<"起点在map "<<u0_gloab<<endl;
        }
    }
  }
}
void globle_planning::end_odom_callback(const geometry_msgs::PoseStamped::ConstPtr msg ){
    ros::Rate loop_rate(10);
    Point end_point;
    end_point.x = msg->pose.position.x;
    end_point.y = msg->pose.position.y;
    double raw, pitch, theta;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation, q);
    tf::Matrix3x3(q).getRPY(raw, pitch, theta);
    end_point.theta = theta;


    //加载结构地图
    int dist[maxSize], path[maxSize], v, E;
    Graphlnk<int, int>  G; // 声明图对象
    G.inputGraph(this->roads);     // 创建图
    //G.outputGraph();
    int u0 = 10;
    int e0 = 10;
    int e_nearest = 0;
     /////获取车辆当前位置对应的路径id
    double min_distance1 =100000;
    int u1=0;
    if(u0_gloab!=0){
         u0=u0_gloab;
    }else{
        for (size_t i = 0; i < this->roads.size(); i++)
            {
                for (size_t j = 0; j < this->roads[i].road_points.size(); j++)
                {
                    double distance = std::sqrt(std::pow(start.first-this->roads[i].road_points[j].x, 2) + std::pow(start.second-this->roads[i].road_points[j].y, 2));
                    if(distance <= min_distance1){
                        u0 = 1+i;
                        min_distance1 = distance;
                        u1=j;
                    }
                }
            }
    }
    e0_gloab=0;
    for (size_t i = 0; i < this->roads.size(); i++)
    {
        if (roads[i].ifmap)
        {
            if (end_point.x > roads[i].lf.first && end_point.x < roads[i].rb.first && end_point.y < roads[i].lf.second && end_point.y > roads[i].rb.second)
            {
                e0_gloab = 1 + i;
                cout<<"终点在map "<<e0_gloab<<endl;
            }
        }
    }
    

    if(e0_gloab!=0){
         e0 =e0_gloab;
    }else{
        double min_distance2 =10000000000;
        for (size_t i = 0; i < this->roads.size(); i++)
        {
            for (size_t j = 0; j < this->roads[i].road_points.size(); j++)
            {
                double distance =(end_point.x-this->roads[i].road_points[j].x)*(end_point.x-this->roads[i].road_points[j].x)+ (end_point.y-this->roads[i].road_points[j].y)*(end_point.y-this->roads[i].road_points[j].y);
                //double distance = std::sqrt(std::pow(end_point.x-this->roads[i].road_points[j].x, 2) + std::pow(end_point.y-this->roads[i].road_points[j].y, 2));
                if(distance <= min_distance2){
                    e0 = 1+i;
                    e_nearest = j;
                    min_distance2 = distance;
                }
            }
        }  
    }
  
    v = G.getVertexPos(u0); // 取得起始顶点的位置
    E = G.getVertexPos(e0); // 取得起始end的位置
    Dijkstra(G, v, dist, path); // 调用Dijkstra函数
    std::vector<int> road_id;
    road_id = printShortestPath(G, v, E, dist, path); // 输出到各个顶点的最短路径



    int e_id = e0 - 1;                                // 终点道路所在的下标
   
    by_djstl::Path globle_path;
    for (size_t i = 0; i < road_id.size(); i++)
      {
          by_djstl::PathPoint p;
          p.road_id=road_id[i];
          globle_path.points.push_back(p);
      }

    this->pub_globle_path.publish(globle_path);
    loop_rate.sleep();
}
void globle_planning::nodeStart(int argc, char **argv)
{
    ros::init(argc, argv, "globle_planning");
    ros::NodeHandle nc;

    string roadMap_path = "/home/by/code/A_star_by_ws/data/1.txt"; // 地图录制位置///home/lwh/Lpp_files/data/road_path8.txt
    this->roads = load_direction_Road(roadMap_path);

    //   low_pass_filter_road(roads,3,8,0);
    //    B_spline_optimization_road(roads,0,3);

      this->pub_globle_path = nc.advertise<by_djstl::Path>("globle_path", 10);                             // 发布全局规划路径
      this->marker_pub = nc.advertise<visualization_msgs::Marker>("road_rviz", 1);                         // 在rviz上显示录制的路径
      this->globle_path_marker_pub = nc.advertise<visualization_msgs::Marker>("road_rviz_globle_path", 1); // 显示规划的全局路径

      // 订阅相关节点
      this->sub_odom = nc.subscribe("/odom", 1, &globle_planning::odometryGetCallBack, this); // 控制节点
      this->sub_end_odom = nc.subscribe("/move_base_simple/goal", 1, &globle_planning::end_odom_callback, this); // 订阅终点位置
      ros::Subscriber sub_start_odom=nc.subscribe("/initialpose",1,&globle_planning::start_odom_callback,this);
      ros::spin();
}


int main(int argc, char  *argv[])
{
    globle_planning node;
    node.nodeStart(argc, argv);
    return(0);
}
