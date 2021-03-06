#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_cmd");
    ros::NodeHandle node;

    ros::Publisher cmd_pub;
    cmd_pub = node.advertise<geometry_msgs::Twist>("/t2/cmd_vel", 100);

    // speed command
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.05;

    ros::Time t1 = ros::Time::now();
    ros::Rate r(100);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();

        // change move direction
        ros::Duration d;
        d = ros::Time::now() - t1;
        if(d.toSec() > 5){
            cmd.linear.x = cmd.linear.x * (-1);
            t1 = ros::Time::now();
            std::cout<<"change move direction."<<std::endl;
        }

        cmd_pub.publish(cmd);
    }

    // stop
    cmd.linear.x = 0;
    cmd_pub.publish(cmd);

    return 0;
}