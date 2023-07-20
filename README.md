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
You need to play back the bag file in that link with the rosbag command to get the point cloud information and image information needed for that node. Intrinsic parameter and extrinsic parameter for initialization are in the data folder. You should download them to the appropriate path, and modify the relevant path in the cpp source code in the src folder.  
 * intrinsic_json: Camera intrinsic parameter JSON file  
 * extrinsic_json: JSON file of initial values of extrinsic parameters between sensors  
## 2. Run the test sample:
You need to open two terminals, the first terminal is used to run the ROS node, when the first node starts, open the second terminal to play back the bag file.If you use the data in this project, you will get the following results：  
![Maunal_cali_result](https://github.com/Redamancy8013/ExplainOfSensorsCalibration/tree/main/SensorsCalibration/manual_cali_result.jpg)   
**This picture is only one frame out of all results**  
## 3. Calibration panel:
The calibration window consists of the left control panel for manual calibration and the right point cloud projection image. Users can check whether the points cloud and the image are aligned by clicking the corresponding button in the panel or using Keyboard as input to adjust the extrinsic parameter. When the points cloud and the image are aligned, the calibration ends, click the save button to save the result.

Extrinsic Params  | Keyboard_input	  | Extrinsic Params	  | Keyboard_input  
 ---- | ----- | ------ | ------  
 +x degree  | q | -x degree | a |  
 +y degree  | w | -y degree | s |  
 +z degree  | e | -z degree | d |  
 +x trans  | r | -x trans | f |  
 +x trans  | t | -y trans | g |  
 +x trans  | y | -z trans | h |  

 Intrinsic Params  | Keyboard_input	  | Intrinsic Params	  | Keyboard_input  
 ---- | ----- | ------ | ------  
 +fy  | i | -fy | k |  
 +fx  | u | -fx | j |  

`Intensity Color`: LiDAR intensity is recorded as the return strength of a laser beam, partly based on the reflectivity of the object struck by the laser pulse. This button can change the display mode to intensity map display mode. This can help to check if the ground lane lines are aligned.

`Overlap Filter`: Eliminate overlap Lidar points within a depth of 0.4m.

`deg step t step`: fxfy scale : These three buttons change the adjustment step for every click or keyboard input.

`point size`: Adjust the size of Lidar points in the projection image.

`Reset`: Press button to reset all manual adjustment.

`Save Image`: If the this button was pressed, the results (calibrated image, extrinsic and intrinsic matrix) are stored by default at running directory ~./manual_calib/:

**For the impact of different operations on the value of the final extrinsic parameter, please refer to /SensorCalibration/manual_change_extrinsic.docx, it gives the calculation method of the extrinsic parameters in the iteration process**

# pcdvisual
In this section, you can crop the point cloud map so that only specific parts of the point cloud information are retained.
## 1. Four input files:  
You need to play back the bag file in that link with the rosbag command to get the point cloud information needed for that node. Extrinsic parameter for initialization are in the data folder.  
## 2. Run the test sample:  
You need to open two terminals, the first terminal is used to run the ROS node, when the first node starts, open the second terminal to play back the bag file. If you use the data in this project, you will get the following results：  
![Pointcloudvisual_result](https://github.com/Redamancy8013/ExplainOfSensorsCalibration/tree/main/pcdvisual/Pointcloud_cut.png)  
**This picture is only one frame out of all results**

# DnetExample
This part uses the dnet library instead of ros to publish and receive messages. We only provide the code of the receiver here. If you want to use it, you need to use dqt to play back the bag data  
## Additional prerequisites  
 * dnet  
 * dqt
## How to build
Open the terminal in the directory /DnetExample/.Run the following command.
mkdir build
cd build
cmake ..
make







