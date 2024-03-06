#include "fast_mapping/fast_mapping.h"
#include <ros/ros.h>
#include <math.h>
#include <unordered_set>

Fast_Mapping::Fast_Mapping():
spin_period(0.05)
{
   dynamic_map_client = node_.serviceClient<nav_msgs::GetMap>("dynamic_map");
   goal_sub = node_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",1,
                             boost::bind(&Fast_Mapping::goalpointCallback, this, _1));
   control_status_sub = node_.subscribe<std_msgs::String>("control_status",1,
                             boost::bind(&Fast_Mapping::statusCallBack,this,_1));
   path_pub = node_.advertise<nav_msgs::Path>("global_path", 10, true);
   map_pub = node_.advertise<nav_msgs::OccupancyGrid>("expandmap",10,true);
   spin_thread_ = new boost::thread(boost::bind(&Fast_Mapping::spin, this, spin_period));
}

Fast_Mapping::~Fast_Mapping()
{

}

void Fast_Mapping::spin(double transform_publish_period)
{
  ros::Rate r(1.0 / transform_publish_period);
  ros::Time start_time = ros::Time::now();
  float path_length_sum = 0;
  while(ros::ok())
  {
     if(fast_mapping)   
     {
      ROS_INFO("fast mapping ok-----------------!!!!!!!!");
      ROS_INFO("fast mapping cost time = %f,total path length = %f",(ros::Time::now() - start_time).toSec(),path_length_sum);
      return;
     }
     getLocation(robot_pose); 
     if(!goal_init)   goal = robot_pose;
     if(force_replan)
     {
       nav_msgs::OccupancyGrid map;
       if(getDynamicMap(map))   
       {
           jps_plan_.expandMap(map,expand_map); 
        //    map_pub.publish(expand_map);
           jps_plan_.mapDataInit(expand_map);
       }
       if(!getGoal(robot_pose,goal))    {r.sleep();continue;}
       ROS_INFO("goal postion = %f,%f,  last_goal postion = %f,%f",goal.position.x,goal.position.y,last_goal.position.x,last_goal.position.y);
       tryPathPlan(robot_pose,goal,true);        //路径规划，需要获得最终路径
       path_length_sum += jps_plan_.getPathLenght();
       last_goal = goal;
       force_replan = false;
     }
     r.sleep();
  }
}

bool Fast_Mapping::getLocation(geometry_msgs::Pose& pos_now)
{
  tf::StampedTransform transform;
  tf::TransformListener listener;
  try
  {
    listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(2.0));
    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_INFO("can not get base_link to map!");
    return false;
  }
    pos_now.position.x = static_cast<double>(transform.getOrigin().x());
    pos_now.position.y = static_cast<double>(transform.getOrigin().y());
    pos_now.orientation.x = static_cast<double>(transform.getRotation().x());
    pos_now.orientation.y = static_cast<double>(transform.getRotation().y());
    pos_now.orientation.z = static_cast<double>(transform.getRotation().z());
    pos_now.orientation.w = static_cast<double>(transform.getRotation().w());
    return true;
}

  bool Fast_Mapping::getDynamicMap(nav_msgs::OccupancyGrid& dynamic_map)
  {
    nav_msgs::GetMap map_srv;
    ROS_INFO("prepare get dynamic map!!!!!!");
    if (dynamic_map_client.call(map_srv))
    {
      ROS_INFO("conect map server ok");
      if(map_srv.response.map.info.resolution != 0)
      { 
         dynamic_map = map_srv.response.map;
         ROS_INFO("get map,resolution is:%f,width = %d,height = %d",
                               dynamic_map.info.resolution,dynamic_map.info.width,dynamic_map.info.height);
         return true;
      }
    }
    ROS_INFO("Dynamic map server fail");
    return false;
  }

 void Fast_Mapping::goalpointCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
 {
     goal_init = true;
     force_replan = true;
 }

 void Fast_Mapping::statusCallBack(const std_msgs::String::ConstPtr& cmd)
 {
    if(cmd->data == "goalreach")    force_replan = true;
 }

bool Fast_Mapping::tryPathPlan(geometry_msgs::Pose start,geometry_msgs::Pose goal,bool plan_modle)
{    
    Eigen::Vector2d start_point(start.position.x,start.position.y);
    Eigen::Vector2d goal_point(goal.position.x,goal.position.y);
    return jps_plan_.globalPathPlaning(start_point,goal_point,expand_map,plan_modle);
}

 bool Fast_Mapping::getGoal(geometry_msgs::Pose& _start,geometry_msgs::Pose& _goal)
{
  if(!goal_init)    return false;
  // ROS_INFO("run to here 2");
  if(expand_map.info.resolution !=0)
  {
    // ROS_INFO("run to here 3");
    std::vector<Eigen::Vector2d>  goal_vec;
    if(getNextGoal(_start,goal_vec,1.0,5.0))   //起始搜索范围为1m-3m,后续依次为3m-6m,6m-9m,9m-12m
    {
      // ROS_INFO("run to here 4");
      float min_distance = 100;
      Eigen::Vector2d temp_goal_min = {-100,-100};
      // ROS_INFO("run to here 5");
      for(int m = 0;m< goal_vec.size();m++)
      {
        /*float diff_x = _start.position.x - goal_vec[m](0);
        float diff_y = _start.position.y - goal_vec[m](1);
        float distance = sqrt(diff_x * diff_x + diff_y * diff_y);*/
        geometry_msgs::Pose temp_goal;
        temp_goal.position.x = goal_vec[m](0);
        temp_goal.position.y = goal_vec[m](1);
        // ROS_INFO("run to here 6");
        if(tryPathPlan(_start,temp_goal,false))         //路径规划，只需要获取长度
        {
           float path_length = jps_plan_.getPathLenght();
          ROS_INFO("goal x = %f,y = %f,distance = %f",temp_goal.position.x,temp_goal.position.y,path_length);
          if(path_length < min_distance)
          {
            min_distance = path_length;
            temp_goal_min(0) = goal_vec[m](0);
            temp_goal_min(1) = goal_vec[m](1);
          }         
        }
        else
        {
          Eigen::Vector2d invalid_point(temp_goal.position.x,temp_goal.position.y);
          ROS_INFO("invalid goal,x = %f,y = %f",temp_goal.position.x,temp_goal.position.y);
          invalid_goal_vec.push_back(invalid_point);
        }
      } 
    if(temp_goal_min(0)== -100 && temp_goal_min(1) == -100)   
    {
      ROS_INFO("No valida goal,fast mapping ok");
      // fast_mapping = true;
      return false;
    }
    _goal.position.x = temp_goal_min(0);
    _goal.position.y = temp_goal_min(1);
    return true;
  }
  else
  {
    ROS_INFO("No next goal");
  }
}
  return false;
}

bool Fast_Mapping::getNextGoal(geometry_msgs::Pose& start,std::vector<Eigen::Vector2d>& candidate_goal_vec,float near_distance_min,float near_distance_max)
{
  float map_width = expand_map.info.width;
  float map_height = expand_map.info.height;
  if(near_distance_min > sqrt(map_width*map_width + map_height*map_height))  {fast_mapping = true;return false;}
  ROS_INFO("prepare find next goal");
  for(int direction = 0;direction < 4;direction++)   //0:右 1：上 2：左 3：下    逆时针方向
  {
    float init_angle = (direction % 4) * 90.0;           //角度制
    ROS_INFO("init angle = %f",init_angle);
    int explor_angle_num = EXPLOR_ANGLE_NUM; 
    int explor_distance_num = std::floor((near_distance_max - near_distance_min) / EXPLOR_DISTANCE_RES);
    for(int j = explor_distance_num;j >0;j--)
    {
       float distance = near_distance_min + j * EXPLOR_DISTANCE_RES;
      //  ROS_INFO("delta angle = %f",angle);
       for(int i =0;i<= explor_angle_num;i++)
       {
         int index_dir = (i % 2) ? std::ceil(i /2) : -std::ceil(i /2);                         //调节角度增加的方向
         float delta_angle = index_dir * EXPLOR_ANGLE_RES;         //角度增量
         float angle = (init_angle + delta_angle) * M_PI / 180.0;
         Eigen::Vector2d temp_goal;
        //  ROS_INFO("search AND angle :distance = %f,angle = %f",distance,angle *180.0 /M_PI);
         if(searchPoint(start,temp_goal,angle,distance))    
         {
           candidate_goal_vec.push_back(temp_goal);
           ROS_INFO("search distance = %f,angle = %f",distance,angle *180.0 /M_PI);
           goto next_search;
         }
       }
    }
    next_search:  
    ROS_INFO(" ");  
  }
  if(!candidate_goal_vec.empty())   return true;
  ROS_INFO("find next goal fail,change distance and find agin");
  return getNextGoal(start,candidate_goal_vec,near_distance_max,near_distance_max+4.0);
}

bool Fast_Mapping::searchPoint(geometry_msgs::Pose& start,Eigen::Vector2d& goal,float angle,float distance)
{
   Eigen::Vector2d point;
   point(0) = start.position.x + distance * cos(angle);
   point(1) = start.position.y + distance * sin(angle);
   Eigen::Vector2i point_grid = jps_plan_.world2grid(point);
//    ROS_INFO("point world x = %f,y = %f ,point grid x = %d,y = %d\n",point(0),point(1),point_grid(0),point_grid(1));
   if(jps_plan_.isVaildCell(point_grid))
   {
        // ROS_INFO("point x = %f,y = %f,it is status is %d",point(0),point(1),expand_map.data[jps_plan_.coord2Id(jps_plan_.world2grid(point))]);
      if(jps_plan_.isUnknown(point_grid))
      {
        //   ROS_INFO("*******************x = %f,y = %f,it is unkown point\n",point(0),point(1));
         if(findFreePoint(point_grid,goal))   
         {
           ROS_INFO("point unkown is:x = %f,y = %f, point free is: x= %f,y = %f\n",point(0),point(1),goal(0),goal(1));
           return true;
         }
      }
    //   ROS_INFO("point x = %f,y = %f,it is status is %d",point(0),point(1),map_data[coord2Id(world2grid(point))]);
   }
//    else
//    {
//     ROS_INFO("point x = %d,y = %d is invalid!",point_grid(0),point_grid(1));
//    }
   return false;
}

bool Fast_Mapping::findFreePoint(Eigen::Vector2i point_grid,Eigen::Vector2d& goal)
{
  for(int i = -5; i <= 5;i++)
  {
    for(int j = -5; j<= 5;j++)
    {
      int x = point_grid(0) + i;
      int y = point_grid(1) + j;
      Eigen::Vector2i tmp_point(x,y);
      if(jps_plan_.isVaildCell(tmp_point))
      {
        if(jps_plan_.isFree(tmp_point))
        {
            Eigen::Vector2d next_goal= jps_plan_.grid2world(tmp_point);
            for(auto point:invalid_goal_vec)
            {
               if(fabs(point(0) - next_goal(0)) < 0.2 &&
                  fabs(point(1) - next_goal(1)) < 0.2)    
                  {
                    invalid_goal_vec.push_back(next_goal);
                    return false;
                  }
            }
            goal(0) = next_goal(0);
            goal(1) = next_goal(1);
            return true;

        }
      }
    }
  }
  return false;
}
