#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>

namespace global_planner {

class GlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    GlobalPlanner();
    GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    void goalCallback(const nav_msgs::PathConstPtr& msg); 
    int newmessage=0;

private:
    ros::Subscriber goalSub;
    nav_msgs::Path currentGoal_; 
};

} 

#endif 
