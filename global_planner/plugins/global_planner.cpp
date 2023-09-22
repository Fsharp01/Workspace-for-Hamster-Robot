#include <pluginlib/class_list_macros.h>
#include "global_planner.h"

PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {

GlobalPlanner::GlobalPlanner() {

}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros);
}

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    ros::NodeHandle nh;
    goalSub = nh.subscribe("matlab_path", 1, &GlobalPlanner::goalCallback, this);
 
}

bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) 
{

    while (newmessage == 0) 
    {
        ROS_INFO("Waiting for MATLAB path...");
        ros::Duration(0.1).sleep();

    }

    if (newmessage == 1)
     {
        ROS_INFO("Received MATLAB path");
        plan.clear();
       

        // poses from currentGoal_ to the plan
        for (const auto& poseStamped : currentGoal_.poses) {
            plan.push_back(poseStamped);
        }
	newmessage=0;
        return true;
        
    }

    return false;
}

void GlobalPlanner::goalCallback(const nav_msgs::PathConstPtr& msg) { // Updated the type of msg parameter
    //Path message from the "matlab_path" topic
    currentGoal_ = *msg;
    ROS_INFO("Received MATLAB path");
    newmessage=1;
}

} 
