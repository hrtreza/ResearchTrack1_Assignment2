#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include "laserscanner_robot_ROS/Speedup.h"


ros::Publisher pub; // a public publisher is defined
float speed=0.0;    //Current speed

void clbk_fcn(const sensor_msgs::LaserScan::ConstPtr& msg){
    
    /*
    This is a callback function to subscribe from "/base_scan" topic, which provide information
    about distance from obstacles. In this function we configure the laser scanner data in three
    different area include: front side, left side, and right side to control and drive the robot
    autonomously. Based on these ranges, the robot will be avoided from any collision.
    */

    geometry_msgs::Twist vel;

 //............................ Inital Values............................

    int d_th=5;           //default threshold radious
    float min_len=d_th;  //inital minimum radious
    float front_side=0;  //front-side distance
    float left_side=0;   //left-side distance
    float right_side=0;  //right-side distance


 //...........................Laser scanner configuration................

    for (int k=300; k<=400; k++){
        if(msg->ranges[k]<min_len) min_len=msg->ranges[k];
    }
    front_side=min_len;                      //set the minimum distance for front side
    min_len=d_th;                           //reset the minimum length

    for (int k=0; k<=100; k++){
        if(msg->ranges[k]<min_len) min_len=msg->ranges[k];
    }
    right_side=min_len;                   //set the minimum distance for right side
    min_len=d_th;                        //reset the minimum length

    for (int k=610; k<=710; k++){
        if(msg->ranges[k]<min_len) min_len=msg->ranges[k];
    }
    left_side=min_len;                  //set the minimum distance for left side
    min_len=d_th;                       //reset the minimum length


 //.....................Obstacle Avoidance and velocity command............... 

    if (front_side<2.0){
        if (left_side < right_side){    //if there is obstacle on the left, turn right:
            vel.linear.x=0.5;           
            vel.angular.z=-1.8;
        }
        else if (right_side <left_side) //if there is obstacle on the right, turn left:
        {
            vel.linear.x=0.5;
            vel.angular.z=1.8;
        }
    }
    else{                               //otherwise drive straight based on the input velocity
        vel.linear.x=speed;
        vel.angular.z=0.0;
    }
    
    pub.publish(vel);                  //Publish the final velocity command


}


bool clbk_speedup(laserscanner_robot_ROS::Speedup::Request &req, laserscanner_robot_ROS::Speedup::Response &res){

    /*
    This is a callback function for our arbitrary service. This service gets two character as
    inputs, and outputs the desired speed chosen by user.The maximum speed is assumed 5.0 and
    the minimum is 0.0 (which prohibits the robot to go backward).  


    */
    float max_speed=5;  //Maximum speed
    float min_speed=0; //Minimum speed
    geometry_msgs::Twist vel;

    if(req.data=="+"){                 //if "+" is requested, the speed increases
        speed= speed+0.5;
        if(speed>max_speed)
            speed=max_speed;
    }  
    if(req.data=="-"){                //if "-" is requested, the speed decreases
        speed= speed-1.0;
        if (speed<min_speed)
            speed=min_speed;  
    }
    res.result=speed;               //the new velocity will be returned as the service response

    return true;
}



int main(int argc, char **argv){

    ros::init(argc, argv,"controller" );
    ros::NodeHandle nh; 
    pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    ros::Subscriber sub=nh.subscribe("/base_scan",1, clbk_fcn);
    ros::ServiceServer service = nh.advertiseService("speed_up", clbk_speedup);
    ros::spin();
  
}