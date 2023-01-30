#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include "std_srvs/Empty.h"
#include <std_msgs/String.h>
#include "laserscanner_robot_ROS/Speedup.h"
#include <nav_msgs/Odometry.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "Robot_Services");
    ros::NodeHandle nh;
    ros::ServiceClient client1 = nh.serviceClient<std_srvs::Empty>("reset_positions");
    ros::ServiceClient client2 = nh.serviceClient<laserscanner_robot_ROS::Speedup>("speed_up");
    ros::Rate loop_rate(10);

    std_srvs::Empty reset;
    laserscanner_robot_ROS::Speedup speedup;
    std::string inputString;

    while(ros::ok()){

        // Design the menu to be more user-friendly
        std::cout << "\n___________________________________________________________\n";
        std::cout << "\n                 Please Enter a Command:...\n";
        std::cout << "Input                 Action        |";
        std::cout << "\n____________________________________|_______________\n";        
        std::cout << "  r ------------       Reset        |  current speed (Max=5):\n";
        std::cout << "  + ------------  Increase the speed|      "<<float(speedup.response.result)<<"\n";
        std::cout << "  - ------------  Decrease the speed|\n";
        std::cout << "  q ------------       Exit         |\n\n";
        std::getline(std::cin, inputString); // get special characters from user

        if(inputString.compare("r") == 0){
            client1.waitForExistence();         // Using client 1: wait for the service existance
            client1.call(reset);                // call the reset service
            std::cout << "\n ***Position reseted***\n";
        }
        else if (inputString.compare("+") == 0){
            client2.waitForExistence();         // Using client 2: wait for the service existance
            speedup.request.data="+";           // set the request data to the user command
            client2.call(speedup);              // call the speedup service to increse the speed
            std::cout << "\n ***speed increased***\n";
        }
        else if (inputString.compare("-") == 0){
            client2.waitForExistence();         // Using client 2: wait for the service existance
            speedup.request.data="-";           // set the request data to the user command 
            client2.call(speedup);              // call the speedup service to decrease the speed
            std::cout << "\n ***speed decreased***\n";
        }
        else if(inputString.compare("q") == 0){
        ros::shutdown();       // exit and kill the program by pressing "q"                 
        }

        loop_rate.sleep();    //keep the loop rate at 10 Hz
    }

    ros::spin();
}
