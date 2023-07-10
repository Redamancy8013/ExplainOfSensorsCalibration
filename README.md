# Introduction
This project is another open source project on github SensorCalibration in lidar2camera interpretation and expansion, and this project is based on the ROS.
# Prerequisites
· CMake
· opencv
· PCL 1.9
· eigen3
· Pangolin
# SensorCalibration
In this section, you can get a rough extrinsic parameter by means of manual calibration. 
1. Four input files:
You need to play back the bag file in that link with the rosbag command to get the point cloud information and image information needed for that node. Intrinsic parameter and extrinsic parameter for initialization are in the data  folder. You should download them to the appropriate path, and modify the relevant path in the cpp source code in the src folder.
· intrinsic_json: Camera intrinsic parameter JSON file
· extrinsic_json: JSON file of initial values of extrinsic parameters between sensors
2. 
# pcdvisual
