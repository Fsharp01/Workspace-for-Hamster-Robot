#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include<iostream>
#include <time.h>
#include <tf2/LinearMath/Quaternion.h>
using namespace std;
#define _PID_SOURCE_

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;


struct position
{
  float x;
  float y;
  float z;
};

struct myQuaternion
{
  float x;
  float y; 
  float z;
  float w;
};

float linx, liny,linz, angularx, angulary, angz, newx, newy, orientationx, orientationy, quaternionx, quaternionz, angularz, theta, thetad;
position setpoint, currentPose;
myQuaternion GoalQuat, currentQuat;
double dt,cbtime;
geometry_msgs::PoseStamped p;
 double K_p =0.3;
 double K_d =0.05;
 double K_i =1;
 double old_error(0);
 double integral(0);




void newCoordinates (const geometry_msgs::Twist& msg)
    {
     
      linx = msg.linear.x;
       liny= msg.linear.y;
       linz = msg.linear.z;
       angularx= msg.angular.x;
       angularz = msg.angular.z;     
        cbtime = ros::Time::now().toSec();

     }


void Receivesetpoint(const geometry_msgs::PoseStamped& msg)
{

        setpoint.x= msg.pose.position.x;
        setpoint.y= msg.pose.position.y;
        
        GoalQuat.x= msg.pose.orientation.x;
        GoalQuat.y= msg.pose.orientation.y;
        GoalQuat.z= msg.pose.orientation.z;
        GoalQuat.w= msg.pose.orientation.w;

}


int main(int argc, char **argv)
    {
        ros::init(argc, argv, "listener");  //creates a node called "/subscribe_to_twist"


        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, &newCoordinates); // subscribes to /cmd_vel topic

        geometry_msgs::PoseStamped p;

        ros::NodeHandle n;
        ros::Subscriber sub2 =n.subscribe("move_base_simple/goal", 1000, Receivesetpoint);


        ros::NodeHandle np;
        ros::Publisher pub = np.advertise<geometry_msgs::PoseStamped>("pidnavpose", 1000);
       
           

        double oldTime = ros::Time::now().toSec();

       ros::Rate rate(2);
      
        while(ros::ok())
           {
                double actTime = ros::Time::now().toSec();

                dt = actTime - oldTime;

                p.header.frame_id="map";

                theta += angularz*dt;

                p.pose.position.x += linx*cos(theta)*dt;
                p.pose.position.y += linx*sin(theta)*dt;
              
            tf2::Quaternion myQuaternion;
            myQuaternion.setRPY( 0, 0, theta );  // Create this quaternion from roll/pitch/yaw (in radians)
          

            p.pose.orientation.x = myQuaternion.getX();
             p.pose.orientation.y = myQuaternion.getY();
             p.pose.orientation.z = myQuaternion.getZ();
             p.pose.orientation.w = myQuaternion.getW();           
            
              

               

/******************************************************************************************************/
                                         //PID control//
/******************************************************************************************************/


         thetad=atan2(setpoint.y-liny,setpoint.x-linx);

            // Calculate error
    double error = thetad - theta;

    // Proportional term
    double Pout = K_p * error;

    // Integral term
    integral += error * dt;
    double Iout = K_i * integral;

    // Derivative term
    double derivative = (error - old_error) / dt;
    double Dout = K_d * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Save error to previous error
    old_error = error;

                 
            
    oldTime = actTime;


            pub.publish(p);
            ros::spinOnce();
            rate.sleep();



          }
            
    }
