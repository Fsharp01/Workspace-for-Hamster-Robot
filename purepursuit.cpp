#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <cmath>
#include <nav_msgs/Path.h>

ros::Publisher pub;
double d_last_target;
double lookahead_distance;
int target_index;
double theta;
double startposx;
double startposy;
std::vector<std::vector<double>> path;
bool Controllplot;
bool PathnInitplot;
bool MovingPathplot;
std::vector<double> last_point;
double xG; 
double yG;
bool path_arrived = false;

void pathCallback(const nav_msgs::Path::ConstPtr& msg) 
{
    
    /*for (auto& point : msg->poses[0].pose.position) 
    {
        double value = point;
        path.push_back({value});
    }
    path_arrived = true;*/
}

int findTargetIndex(const std::vector<std::vector<double>>& path, double x2, double y2, double lookahead_distance, double theta) 
{
    int target_index = 0;  

   
    for (int i = 0; i < path.size(); ++i) 
    {
        
        double d = std::sqrt(std::pow(path[i][0] - x2, 2) + std::pow(path[i][1] - y2, 2));

       
        if (d > lookahead_distance && ((path[i][0] - x2) * std::cos(theta) + (path[i][1] - y2) * std::sin(theta)) > 0) 
        {
            
            target_index = i;
            break;
        }
    }

    return target_index;
}

double calculate_steering_angle(std::vector<std::vector<double>>& path, double startposx, double startposy, double theta) 
{   
    double x2 = startposx;
    double y2 = startposy;
    std::vector<double> x_list = {x2};
    std::vector<double> y_list = {y2};
    std::vector<double> target_points = {};
    double lookahead_distance = 0.3;
    double max_speed = 0.2;
    double L = 0.25; 
    double max_steering_angle = M_PI / 4; 


    std::vector<double> last_point = path.back();

    int target_index = findTargetIndex(path, x2, y2, lookahead_distance, theta);

    double x_target = path[target_index][0];
    double y_target = path[target_index][1];

    double d_target = std::hypot(x_target - x2, y_target - y2);

    d_last_target = std::hypot(last_point[0] - x2, last_point[1] - y2);

    double alpha = std::atan2(y_target - y2, x_target - x2);

    double desired_steering_angle = std::atan2(2 * L * std::sin(alpha - theta), d_target);

    double steering_angle = std::max(-max_steering_angle, std::min(max_steering_angle, desired_steering_angle));

    double theta_dot = max_speed * std::tan(steering_angle) / L;

    x_list.push_back(x2);
    y_list.push_back(y2);

    return steering_angle;
}


void sendROSMessage(const ros::TimerEvent& event) 
{
    if (path_arrived) 
    {
        double max_speed = 1;
        double steering_angle = calculate_steering_angle(path, startposx, startposy, theta);

        geometry_msgs::Twist twistMsg;
        twistMsg.linear.x = max_speed;
        twistMsg.angular.z = steering_angle;

        if (std::sqrt(std::pow(startposx - xG, 2) + std::pow(startposy - yG, 2)) <= 0.225) {
            twistMsg.linear.x = 0;
            twistMsg.angular.z = 0;
        }

        pub.publish(twistMsg);
        path_arrived = false;
    }
}


void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) 
{
    double positionX = msg->pose.pose.position.x;
    double positionY = msg->pose.pose.position.y;
    double quatX = msg->pose.pose.orientation.x;
    double quatY = msg->pose.pose.orientation.y;
    double quatZ = msg->pose.pose.orientation.z;
    double quatW = msg->pose.pose.orientation.w;
    tf::Quaternion q(quatX, quatY, quatZ, quatW);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    theta = yaw;
    startposx = positionX;
    startposy = positionY;
}




int main(int argc, char** argv) 
{
    // Initialize ROS
    ros::init(argc, argv, "pure_pursuit_controller");
    ros::NodeHandle nh;

    // Create publishers and subscribers
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber sub = nh.subscribe("/agent1/pose/amcl", 1, amclCallback);
    ros::Subscriber path_sub = nh.subscribe("/your_path_topic", 1, pathCallback);

    // Define your timer here
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), sendROSMessage);

    // Spin
    ros::spin();

    return 0;
}
