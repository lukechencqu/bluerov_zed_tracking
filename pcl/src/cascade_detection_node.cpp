// #include "2d_object_detection/CascadeDetection.hpp"
#include "CascadeDetection.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "cascade_detection");

    ros::NodeHandle node("~");

    bluerov_detection_tracking::CascadeDetection cascadeDetection(node);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    ROS_INFO("Node cascade_detection closed.");
    return 0;
}