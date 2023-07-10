# Introduction
This project is another open source project on github SensorCalibration in lidar2camera interpretation and expansion, and this project is based on the ROS.
# Prerequisites
 * CMake  
 * opencv  
 * PCL 1.9  
 * eigen3  
 * Pangolin  
# SensorCalibration
In this section, you can get a rough extrinsic parameter by means of manual calibration.   
## 1. Four input files:  
You need to play back the bag file in that link with the rosbag command to get the point cloud information and image information needed for that node. Intrinsic parameter and extrinsic parameter for initialization are in the data  folder. You should download them to the appropriate path, and modify the relevant path in the cpp source code in the src folder.  
 * intrinsic_json: Camera intrinsic parameter JSON file  
 * extrinsic_json: JSON file of initial values of extrinsic parameters between sensors  
## 2. Run the test sample:
You need to open two terminals, the first terminal is used to run the ROS node, when the first node starts, open the second terminal to play back the bag file. 
## 3. Calibration panel:
The calibration window consists of the left control panel for manual calibration and the right point cloud projection image. Users can check whether the points cloud and the image are aligned by clicking the corresponding button in the panel or using Keyboard as input to adjust the extrinsic parameter. When the points cloud and the image are aligned, the calibration ends, click the save button to save the result.

Extrinsic Params  | Keyboard_input	  | Extrinsic Params	  | Keyboard_input  
 ---- | ----- | ------ | ------  
 单元格内容  | 单元格内容 | 单元格内容 |  
 单元格内容  | 单元格内容 | 单元格内容 |  
# pcdvisual
