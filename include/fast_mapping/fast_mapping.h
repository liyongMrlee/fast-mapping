#ifndef FAST_MAPPING_
#define FAST_MAPPING_

#include <ros/ros.h>
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_listener.h"
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <Eigen/Eigen>
#include <boost/heap/d_ary_heap.hpp>      // boost::heap::d_ary_heap
#include <memory>                         // std::shared_ptr
#include <limits>                         // std::numeric_limits
#include <vector>                         // std::vector
#include <unordered_map>                  // std::unordered_map
#include <boost/thread.hpp>
#include <fast_mapping/jps_search.h>

#define EXPLOR_ANGLE     90.0
#define EXPLOR_ANGLE_RES 1.0
#define EXPLOR_ANGLE_NUM EXPLOR_ANGLE / EXPLOR_ANGLE_RES
#define EXPLOR_DISTANCE_RES 0.2

class JPS_Search;
class Fast_Mapping
{
public:
    Fast_Mapping();
    ~Fast_Mapping();

    bool getDynamicMap(nav_msgs::OccupancyGrid& dynamic_map);
    void goalpointCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
    void statusCallBack(const std_msgs::String::ConstPtr& cmd);
    bool getLocation(geometry_msgs::Pose& pos_now);
    bool getGoal(geometry_msgs::Pose& _start,geometry_msgs::Pose& _goal);
    bool getNextGoal(geometry_msgs::Pose& start,std::vector<Eigen::Vector2d>& goal,float near_distance_min,float near_distance_max);
    void spin(double transform_publish_period);
    bool tryPathPlan(geometry_msgs::Pose start,geometry_msgs::Pose goal,bool plan_modle);
    bool searchPoint(geometry_msgs::Pose& start,Eigen::Vector2d& goal,float angle,float distance);
    bool findFreePoint(Eigen::Vector2i point_grid,Eigen::Vector2d& goal);

private:
    ros::NodeHandle node_;
    ros::Subscriber map_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber control_status_sub;
    ros::ServiceClient dynamic_map_client;
    ros::Publisher  path_pub;
    ros::Publisher  map_pub;

    boost::thread* spin_thread_;
    bool fast_mapping = false;
    bool force_replan = false;
    bool goal_init = false;
    geometry_msgs::Pose robot_pose,goal,last_goal;
    std::vector<Eigen::Vector2d> invalid_goal_vec;
    nav_msgs::OccupancyGrid expand_map;
    JPS_Search jps_plan_;
    double spin_period;
};



#endif