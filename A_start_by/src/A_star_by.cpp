#include "include/headfile.h"




class A_star{
public:
     ros::Publisher marker_pub5; // 展示平滑处理后的曲线
     ros::Publisher marker_pub3; // 展示自己的地图
     ros::Publisher marker_pub2; // 展示A星搜索的路径
     ros::Publisher marker_pub4; ////展示平滑处理后的曲线
     ros::Publisher marker_pub_all_door;//发布所有门
     ros::Publisher marker_pub6;

     vector<vector<int>> map;
     vector<vector<int>> map_final;
     std::vector<Box_2d> door;

     vector<Point> rem_point_vec;
     vector<Point> path_points;
     vector<vector<Point>> path_points_vec;

     struct path_points_vec_str{
          vector<vector<Point>> path_points_vec;
          vector<int> id;
     };

     path_points_vec_str astar_road;
     path_points_vec_str djstl_road;

     vector<Road> roads;
     vector<int> djstl_vec;

     int pz = 5;
     bool init_flag = 0;

     pair<int, int> target;            // 终点
     pair<int, int> start;

     pair<float, float> target_f; // 终点
     pair<float, float> start_f;

     pair<float, float> end;               //粉色按钮
     pair<float, float> origin;            // 绿色按钮

     Point end_point;
     Point vehicle_pose;

     void Node_Start(int argc, char **argv);
     void initdoor();
     void initMap(int map_width, int map_height);
     void GetMapCallBack(const nav_msgs::OccupancyGrid msg);
     void end_odom_callback(const geometry_msgs::PoseStamped::ConstPtr msg);
     void start_odom_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
     void get_end_point_msgs(pair<float,float> end,MAP_MSG map_msg_)
     {

          end_point.x =end.first;
          end_point.y = end.second;

          float min_dis = 1000;
          int x_index = 0,y_index=0;

           for (int i = 0; i < map_msg_.width; i++)
           {
                for (int j = 0; j < map_msg_.height; j++)
                {
                     if ((pow((i * map_msg_.resolution) + map_msg_.orign_x - end_point.x, 2) + pow((j * map_msg_.resolution) + map_msg_.orign_y - end_point.y, 2)) < min_dis)
                     {
                          x_index = j;
                          y_index = i;
                          min_dis = pow((i * map_msg_.resolution) + map_msg_.orign_x- end_point.x, 2) + pow((j * map_msg_.resolution) + map_msg_.orign_y - end_point.y, 2);
                     }
                }
          }
          target = {x_index, y_index};
     }
     void get_start_point_msgs(pair<float,float> end,MAP_MSG map_msg_)
     {

          end_point.x =end.first;
          end_point.y = end.second;

          float min_dis = 1000;
          int x_index = 0,y_index=0;

           for (int i = 0; i < map_msg_.width; i++)
           {
                for (int j = 0; j < map_msg_.height; j++)
                {
                     if ((pow((i * map_msg_.resolution) + map_msg_.orign_x - end_point.x, 2) + pow((j * map_msg_.resolution) + map_msg_.orign_y - end_point.y, 2)) < min_dis)
                     {
                          x_index = j;
                          y_index = i;
                          min_dis = pow((i * map_msg_.resolution) + map_msg_.orign_x- end_point.x, 2) + pow((j * map_msg_.resolution) + map_msg_.orign_y - end_point.y, 2);
                     }
                }
          }
          start = {x_index, y_index};
     }
     void odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg);
     void globlepathGetCallBack( by_djstl::Path msg);
     void A_star_make(int map_id);
};
void A_star::A_star_make(int map_id){//参数是第几个地图从0开始数的0123
     path_points.clear();
     get_end_point_msgs(target_f, map_msg_jubu_vec[map_id]);
     get_start_point_msgs(start_f, map_msg_jubu_vec[map_id]);
     AStar star(map_msg_jubu_vec[map_id].map);// 构造函数读取地图
     star.PrintAStarPath(start, target); // A星算法
     for (int i = 0; i < final_path.size(); i += 10)
     { // 将路径放入path以便进行平滑处理
          Point path;
          path.y =  final_path[i].first * map_msg_jubu_vec[map_id].resolution +map_msg_jubu_vec[map_id].orign_y + map_msg_jubu_vec[map_id].resolution / 2;
          path.x  = final_path[i].second * map_msg_jubu_vec[map_id].resolution+ map_msg_jubu_vec[map_id].orign_x+ map_msg_jubu_vec[map_id].resolution / 2;
          path_points.push_back(path);
     }
     path_points_vec.push_back(path_points);

     for(int i=0;i<opnlist_point.size();i++){
          opnlist_point[i].x = opnlist_point[i].x * map_msg_jubu_vec[map_id].resolution + map_msg_jubu_vec[map_id].orign_y + map_msg_jubu_vec[map_id].resolution / 2;
          opnlist_point[i].y = opnlist_point[i].y * map_msg_jubu_vec[map_id].resolution + map_msg_jubu_vec[map_id].orign_x + map_msg_jubu_vec[map_id].resolution / 2;
     }
     opnlist_point_vec.push_back(opnlist_point);
     astar_road.id.push_back(map_msg_jubu_vec[map_id].id);
     astar_road.path_points_vec.push_back(path_points);
}
void A_star::initdoor(){

     door.clear();//可视化用
     door_msg_vec.clear();//算法用
     Box_2d door1_beh({19.9, 16.4}, 1.57, 3, 0.5, 0);
     Box_2d door2_pre({-3.2, 3.5}, 0.0, 3, 0.5, 0);
     Box_2d door2_beh({0.55, -3.3}, 0.0, 3, 0.5, 0);
     Box_2d door3_pre({12.0, -11.7}, 1.57, 3, 0.5, 0);

     door.push_back(door2_pre);
     door.push_back(door2_beh);
     door.push_back(door1_beh);
     door.push_back(door3_pre);

     door_msg1.map_id=1;
     door_msg1.num=1;
     door_msg1.ifpre=0;
     door_msg1.ifbeh=1;
     door_msg1.door_beh={19.9, 16.4};

     door_msg2.map_id = 3;
     door_msg2.num = 2;
     door_msg2.ifpre = 1;
     door_msg2.ifbeh = 1;
     door_msg2.door_beh = {0.55, -3.3};
     door_msg2.door_pre = {-3.2, 3.5};

     door_msg3.map_id = 5;
     door_msg3.num = 3;
     door_msg3.ifpre = 1;
     door_msg3.ifbeh = 0;
     door_msg3.door_pre = {12.0, -11.7};

     door_msg_vec.push_back(door_msg1);
     door_msg_vec.push_back(door_msg2);
     door_msg_vec.push_back(door_msg3);
}
void A_star::globlepathGetCallBack( by_djstl::Path msg){
     djstl_vec.clear();
   
     opnlist_point_vec.clear();
     opnlist_point.clear();
     path_points.clear();
     path_points_vec.clear();
     cout<<"接到djstla的全局路径规划数据";
     for(int i=0;i<msg.points.size();i++){
          djstl_vec.push_back(msg.points[i].road_id);
          cout << djstl_vec[i] << endl;
     }
/////////////////////////////////////////////////////////////////////////////////////////////////////////栅格地图处理
     for(int j=0;j<map_msg_jubu_vec.size();j++){
          if(djstl_vec[0] == map_msg_jubu_vec[j].id ){//地图在头
               cout<<"map"<<map_msg_jubu_vec[j].id<<"在头"<<endl;
               start_f.first = origin.first;//地图在头，起点几位绿点所点
               start_f.second = origin.second;
               if(djstl_vec.size() == 1){target_f.first=end.first; target_f.second=end.second;} //如果只有一条路就，那就在一个地图内A星
               else{ 
                    for(int i=0;i<door_msg_vec.size();i++){
                         if(door_msg_vec[i].map_id == djstl_vec[0]){
                              if(djstl_vec[0]> djstl_vec[1]){
                                   target_f.first=  door_msg_vec[i].door_pre.first;   target_f.second=  door_msg_vec[i].door_pre.second; //去前门
                              }else{
                                   target_f.first=  door_msg_vec[i].door_beh.first;   target_f.second=  door_msg_vec[i].door_beh.second; //去后门
                              }
                         }
                    }
               }
               A_star_make(j);
          }

          if(djstl_vec.back() == map_msg_jubu_vec[j].id ){//地图在尾
               cout<<"map"<<map_msg_jubu_vec[j].id<<"在尾"<<endl;
               target_f.first = end.first;                   
               target_f.second = end.second;
               if(djstl_vec.size() == 1){start_f.first=origin.first; start_f.second=origin.second;} //如果只有一条路就，那就在一个地图内A星
               else{ 
                    for(int i=0;i<door_msg_vec.size();i++){
                         if(door_msg_vec[i].map_id == djstl_vec.back()){
                              if(djstl_vec.back()> djstl_vec[djstl_vec.size()-2]){
                                   start_f.first=  door_msg_vec[i].door_pre.first;   start_f.second=  door_msg_vec[i].door_pre.second; //去前门
                              }else{
                                   start_f.first=  door_msg_vec[i].door_beh.first;   start_f.second=  door_msg_vec[i].door_beh.second; //去后门
                              }
                         }
                    }
               }
               A_star_make(j);
          }
          if(djstl_vec.size()>1 ){
               for(int i=1;i<djstl_vec.size()-1;i++){
                    if(djstl_vec[i] == map_msg_jubu_vec[j].id){
                         for(int p=0;p<door_msg_vec.size();p++){
                              if(door_msg_vec[p].map_id == djstl_vec[i]){//找到门p
                                   cout<<"地图"<<map_msg_jubu_vec[j].id<<"在中间"<<endl;
                                   if(djstl_vec[i]>djstl_vec[i-1]){
                                        start_f.first=   door_msg_vec[p].door_pre.first;   start_f.second=   door_msg_vec[p].door_pre.second;  //去前门
                                        target_f.first=  door_msg_vec[p].door_beh.first;   target_f.second=  door_msg_vec[p].door_beh.second;///去后门
                                   }else{
                                        start_f.first=   door_msg_vec[p].door_beh.first;   start_f.second=   door_msg_vec[p].door_beh.second; //去后门
                                        target_f.first=  door_msg_vec[p].door_pre.first;   target_f.second=  door_msg_vec[p].door_pre.second; //去前门
                                   }
                                   A_star_make(j);
                              }    
                         }
                    }
               }
          }
     } 
     ros::Rate rate(10);
     rate.sleep();
     rviz_road5(marker_pub5, path_points_vec); // 没有经过平滑处理的路径
     rviz_road2(marker_pub2, opnlist_point_vec);   // 探索路径显示
     rviz_road3(marker_pub3, rem_point_vec);   // 膨胀地图显示
     path_points_vec = B_spline_optimization(path_points_vec); // B样条平滑处理
     rviz_road4(marker_pub4, path_points_vec); // 经过平滑处理的路径
///////////////////////////////////////////////////////////////////////////////////////////////////////////道路地图处理
 djstl_road.path_points_vec.clear();
 djstl_road.id.clear();
          for(int i=0;i<roads.size();i++){
               if(i+1 == djstl_vec[0] && !roads[i].ifmap){  //////////////////////////////////////////////////////道路在头
                    cout<<"road"<<djstl_vec[0]<<"在头"<<endl;
                    vector<Point> path;
                    float min_distance1 = 10000;
                    int u1;
                    float min_distance2 = 10000;
                    int u2;
               
                    for (int c = 0; c < roads[i].road_points.size(); c++)//找到起点
                    {
                         double distance = sqrt(pow(origin.first-roads[i].road_points[c].x, 2) 
                                               +pow(origin.second-roads[i].road_points[c].y, 2));
                                               
                         if(distance <= min_distance1){
                              u1=c;
                              min_distance1=distance;
                         }
                    }
                    for (int z = 0; z < roads[i].road_points.size(); z++)
                    {
                         double distance = sqrt(pow(end.first-roads[i].road_points[z].x, 2) 
                                                  + pow(end.second-roads[i].road_points[z].y, 2));
                         if(distance <= min_distance2){
                              u2 = z;
                              min_distance2 = distance;
                         }
                    }
                    cout<<"road"<<djstl_vec[0]<<"在头"<<endl;
                    if(djstl_vec.size() == 1 ){//如果只有一条路
                         if(u1<u2){
                              for (int j = u1; j < u2; j++)
                              {
                                   Point p;
                                   p.x = roads[i].road_points[j].x;
                                   p.y = roads[i].road_points[j].y;
                                   path.push_back(p);
                              }
                         }else{
                              for (int j = u2; j < u1; j++)
                              {
                                   Point p;
                                   p.x = roads[i].road_points[j].x;
                                   p.y = roads[i].road_points[j].y;
                                   path.push_back(p);
                              }
                         }
                    }else{
                         if(djstl_vec[0]<djstl_vec[1]){
                              for (int j = u1; j < roads[i].road_points.size(); j++)
                              {
                                   Point p;
                                   p.x = roads[i].road_points[j].x;
                                   p.y = roads[i].road_points[j].y;
                                   path.push_back(p);
                              }
                         }else{
                              for (int j = u1; j >= 0; j--)
                              {
                                   Point p;
                                   p.x = roads[i].road_points[j].x;
                                   p.y = roads[i].road_points[j].y;
                                   path.push_back(p);
                              }
                         }
                    }
                    djstl_road.path_points_vec.push_back(path);
                    djstl_road.id.push_back(djstl_vec[0]);
               }
               else if(i+1 == djstl_vec.back() && !roads[i].ifmap){ ////////////////////////////////////// //道路在尾
                    cout<<"road"<<djstl_vec.back()<<"在尾"<<endl;
                    vector<Point> path;
                    float min_distance1 = 10000;
                    int u1;
                    float min_distance2 = 10000;
                    int u2;
               
                    for (int c = 0; c < roads[i].road_points.size(); c++)//找到起点
                    {
                         double distance = sqrt(pow(origin.first-roads[i].road_points[c].x, 2) 
                                                  + pow(origin.second-roads[i].road_points[c].y, 2));
                         if(distance <= min_distance1){
                              u1 = c;
                              min_distance1 = distance;
                         }
                    }
                    for (int z = 0; z < roads[i].road_points.size(); z++)
                    {
                         double distance = sqrt(pow(end.first-roads[i].road_points[z].x, 2) 
                                                  + pow(end.second-roads[i].road_points[z].y, 2));
                         if(distance <= min_distance2){
                              u2 = z;
                              min_distance2 = distance;
                         }
                    }
                  if(djstl_vec.size() == 1){//如果只有一条路
                         continue;//已经在上一步做过
                    }else{
                         if(djstl_vec.back()>djstl_vec[djstl_vec.size()-2]){
                              for (int j = 0; j < u2; j++)
                              {
                                   Point p;
                                   p.x = roads[i].road_points[j].x;
                                   p.y = roads[i].road_points[j].y;
                                   path.push_back(p);
                              }
                         }else{
                              for (int j = roads[i].road_points.size()-1; j >= u2; j--)
                              {
                                   Point p;
                                   p.x = roads[i].road_points[j].x;
                                   p.y = roads[i].road_points[j].y;
                                   path.push_back(p);
                              }
                         }
                    }
                    djstl_road.path_points_vec.push_back(path);
                    djstl_road.id.push_back(djstl_vec.back());
               }
               else {
                    for(int p=1;p<djstl_vec.size()-1;p++){
                         vector<Point> path;
                         if(djstl_vec[p] == i+1 && !roads[i].ifmap){
                              if(djstl_vec[p]>djstl_vec[p-1]){
                                   for(int m=0;m<roads[i].road_points.size();m++){
                                        Point p;
                                        p.x = roads[i].road_points[m].x;
                                        p.y = roads[i].road_points[m].y;
                                        path.push_back(p);
                                   }
                              }else{
                                    
                                    for(int m=roads[i].road_points.size()-1;m>=0;m--){
                                        Point p;
                                            
                                        p.x = roads[i].road_points[m].x;
                                        p.y = roads[i].road_points[m].y;
                                        path.push_back(p);
                                   }

                              }
                              djstl_road.path_points_vec.push_back(path);
                              djstl_road.id.push_back(djstl_vec[p]);
                         }
                    }
               } 
          }
       
         rviz_road6(marker_pub6,djstl_road.path_points_vec);

 }
void A_star::odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg){
     all_door_rviz(marker_pub_all_door, door);
// 得到车辆的定位信息和速度信息
    double raw, pitch, theta;
    tf::Quaternion q;
    tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(raw, pitch, theta);
    vehicle_pose.theta = theta;
    vehicle_pose.x = odometry_msg->pose.pose.position.x;
    vehicle_pose.y = odometry_msg->pose.pose.position.y;
    start.first = ( vehicle_pose.y- map_msg.orign_y) / map_msg.resolution;
    start.second = ( vehicle_pose.x- map_msg.orign_x) / map_msg.resolution;
}
void A_star::end_odom_callback(const geometry_msgs::PoseStamped::ConstPtr msg){
     cout<<"x: "<<msg->pose.position.x<<endl;
     cout<<"y: "<<msg->pose.position.y<<endl;
     end.first=msg->pose.position.x;
     end.second=msg->pose.position.y;
    }
void A_star::start_odom_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  double x=msg->pose.pose.position.x;
  double y=msg->pose.pose.position.y;
  std::cout<<"绿色箭头所指("<<x<<", "<<y<<")"<<std::endl;
  origin.first=x;
  origin.second=y;
}
void A_star::initMap(int map_width,int  map_height){
     map.clear();
     for (int i = 0; i < map_width; i++) {
     vector<int> t(map_height, 0);//大小map_height，每个值都为0
          map.push_back(t);
     }
     opnlist_point.clear();
}
void A_star::GetMapCallBack(const nav_msgs::OccupancyGrid msg){
     MAP_MSG map_msg_jubu;
     map_msg_jubu.width = msg.info.width;       // 读取宽度384
     map_msg_jubu.height = msg.info.height;     // 读取高度384
     map_msg_jubu.orign_x = msg.info.origin.position.x; // 读取地图源点-10
     map_msg_jubu.orign_y = msg.info.origin.position.y;//-10
     map_msg_jubu.half_x =  map_msg_jubu.width / 2;
     map_msg_jubu.half_y =  map_msg_jubu.height / 2;
     map_msg_jubu.resolution = msg.info.resolution; // 读取地图分辨率0.05
     float x=map_msg_jubu.orign_x+ map_msg_jubu.half_x * map_msg_jubu.resolution;//不出意外xy应该是地图中点坐标？
     float y=map_msg_jubu.orign_y+ map_msg_jubu.half_y * map_msg_jubu.resolution;

     for (size_t i = 0; i < this->roads.size(); i++)
     {
          if(roads[i].ifmap){
               if(x>roads[i].lf.first && x<roads[i].rb.first && y<roads[i].lf.second && y>roads[i].rb.second){
                    cout<<"当前地图为map "<<i+1<<endl;
                    map_msg_jubu.id=i+1;
               }
          }
     }

     initMap( map_msg_jubu.width,  map_msg_jubu.height );  // 创建地图（此时地图只给了大小还未赋值）
     rem_point_vec.clear();
     for (int i = 0; i <  map_msg_jubu.width; i++)
     {
          for (int j = 0; j <  map_msg_jubu.height; j++)
          {
               if (msg.data[i * map_msg_jubu.width + j] == 0)
               {
                    map[i][j] = 0;
               } // 0表示可通过路径
               else if (msg.data[i * map_msg_jubu.width + j] == 100)
               {
                    map[i][j] = 1;
               } // 1表示障碍物
               else
               {
                    map[i][j] = 2;
               } // 2表示未探测的路径

               if (map[i][j] == 1)
               {
                    for (int k = i - pz; k < (i + pz +1); k++)
                    {
                         for (int z = j - pz; z < (j + pz +1); z++)
                         {
                              if (k > map_msg_jubu.width - 1 || z > map_msg_jubu.height - 1 || k < 0 || z < 0)
                              {
                                   continue;
                              } // 防止越界
                              if (map[k][z] != 0)
                                   continue;
                              Point rem_point;
                              rem_point.x = z;
                              rem_point.y = k;
                              rem_point_vec.push_back(rem_point); // 将膨胀的点放入容器
                         }
                    }
               }
          }
     }
     //      //膨胀处理
             for(int i=0;i<rem_point_vec.size();i++)  {
                    int z=rem_point_vec[i].x;
                    int k=rem_point_vec[i].y;
                    map[k][z]=1;
             }
           map_msg_jubu.map=map;
          //////////////////////
          if(map_msg_jubu.id==1)   {map_msg_jubu.num=1;}
          if(map_msg_jubu.id==3)   {map_msg_jubu.num=2;}
          if(map_msg_jubu.id==5)   {map_msg_jubu.num=3;}
          /////////////////////
          map_msg_jubu.rem_point_vec=rem_point_vec;
           map_msg_jubu_vec.push_back(map_msg_jubu);
           cout<<"map"<< map_msg_jubu.id<<"  is  ok"<<endl;
           if(map_msg_jubu_vec.size() == 3){
               init_flag=1;
               cout<<" init_flag=1"<<endl;//地图读取完毕标志位
           }
      }
void A_star::Node_Start(int argc,char **argv){
     ros::init(argc,argv,"A_start_by");
     ros::NodeHandle nc;

     string roadMap_path = "/home/by/code/A_star_by_ws/data/1.txt"; // 地图录制位置///home/lwh/Lpp_files/data/road_path8.txt
     roads = load_direction_Road(roadMap_path);
     initdoor();

  

     marker_pub5 = nc.advertise<visualization_msgs::Marker>("road_rviz5", 1);        // 展示未经平滑处理后的曲线
     marker_pub3 = nc.advertise<visualization_msgs::Marker>("road_rviz3", 1);        // 展示膨胀的地图
     marker_pub2 = nc.advertise<visualization_msgs::Marker>("road_rviz2", 1);        // 展示A星搜索过的路径
     marker_pub4 = nc.advertise<visualization_msgs::Marker>("road2_rviz4", 1);       // 展示平滑处理后的曲线
     marker_pub_all_door = nc.advertise<visualization_msgs::Marker>("all_obs_la", 1);//画门
     marker_pub6 = nc.advertise<visualization_msgs::Marker>("road_rviz6", 1);        //展示road规划的道路



     ros::Subscriber sub_map1 = nc.subscribe("/robot_1/map", 1, &A_star::GetMapCallBack, this); // 只运行一次，读取地图数据
     ros::Subscriber sub_map2 = nc.subscribe("/robot_2/map", 1, &A_star::GetMapCallBack, this); // 只运行一次，读取地图数据
     ros::Subscriber sub_map3 = nc.subscribe("/robot_3/map", 1, &A_star::GetMapCallBack, this); // 只运行一次，读取地图数据

     ros::Subscriber sub_start_odom=nc.subscribe("/initialpose",1,&A_star::start_odom_callback,this);
     ros::Subscriber sub_end_odom = nc.subscribe("/move_base_simple/goal", 1, &A_star::end_odom_callback, this); // 订阅终点位置，并运行A星算法
     ros::Subscriber sub_odom = nc.subscribe("/odom", 1, &A_star::odometryGetCallBack, this);                    // 获取小车当前位置
     ros::Subscriber sub_globle_path = nc.subscribe("globle_path", 1, &A_star::globlepathGetCallBack, this); // 接受local规划路径
     ros::spin();
}  
int main(int argc, char  *argv[])
{
     A_star a_star;
     a_star.Node_Start(argc, argv);
     return 0;
}
