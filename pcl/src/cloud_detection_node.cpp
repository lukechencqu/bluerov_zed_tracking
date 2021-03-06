#include <ros/ros.h>
#include "CloudDetection.hpp"
#include <fstream>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "republish");
    ros::NodeHandle node("~");

    bluerov_detection_tracking::CloudDetection cloudDetection;
    if(!cloudDetection.initialize(node)){
      ROS_ERROR("Failed to initialize BbX.");
      return EXIT_FAILURE;
    }

    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
    cout<<"ROS shutdown called"<<endl;

    return EXIT_SUCCESS;    
}





/* vector<MeasurementPackage> measurement_pack_list;

  // hardcoded input file with laser and radar measurements
  string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
  ifstream in_file(in_file_name_.c_str(), ifstream::in);

  if (!in_file.is_open()) {
    cout << "Cannot open input file: " << in_file_name_ << endl;
  }

  string line;
  // set i to get only first 3 measurments
  int i = 0;
  while (getline(in_file, line) && (i<=3)) {

    MeasurementPackage meas_package;

    istringstream iss(line);
    string sensor_type;
    iss >> sensor_type; // reads first element from the current line
    int64_t timestamp;
    if (sensor_type.compare("L") == 0) {  // laser measurement
      // read measurements
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x,y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);

    } else if (sensor_type.compare("R") == 0) {
      // Skip Radar measurements
      continue;
    }
    ++i;
  }    */ 