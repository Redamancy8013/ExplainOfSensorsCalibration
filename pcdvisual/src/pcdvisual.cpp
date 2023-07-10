#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <cmath>

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>

#include <pcl/point_cloud.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Core>
#include <iostream>
#include <string>

 // Include the ROS library
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
 #include <pcl_conversions/pcl_conversions.h>
// c++ lock
#include <mutex>
#include <thread>

#include "extrinsic_param.hpp"
#include "intrinsic_param.hpp"
#include "projector_lidar.hpp"

#include <chrono>

int main(int argc, char **argv) {

    // 记录程序开始时间
    auto start = std::chrono::steady_clock::now();

    // string intrinsic_json = "/home/easycool/project/test/PclVisual/src/pcdvisual/center_camera-intrinsic.json";
    std::string extrinsic_json = "/home/easycool/project/test/PclVisual/src/pcdvisual/top_center_lidar-to-center_camera-extrinsic.json";

    // load extrinsic
    Eigen::Matrix4d T1;
    LoadExtrinsic(extrinsic_json, T1);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::Affine3f transform_T = Eigen::Affine3f::Identity();

    T.matrix() = T1;
    transform_T = T.cast<float>();

    cout << "Loading data completed!" << endl;

    Eigen::Vector3d v ( 1, 0, 0 );
    Eigen::Vector3d v_transformed = T*v;                              // 相当于R*v+t
    cout<<"v tranformed = "<<v_transformed.transpose()<<endl;

     //XYZ点云文件的导入
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered1_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered2_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile("/home/easycool/project/test/PclVisual/src/pcdvisual/0.pcd", *cloud);
    std::cout << "cloud size: " << cloud->size() << std::endl;

    // 定义角度阈值和基准方向
    Eigen::Vector3f reference_direction(v_transformed[0], v_transformed[1], v_transformed[2]);

     // 创建直通滤波器对象1
    pcl::PassThrough<pcl::PointXYZ> pass1;
    pass1.setInputCloud(cloud);
    pass1.setFilterFieldName("x");
    pass1.setFilterLimits(-10, 10);
    pass1.setFilterLimitsNegative(false);
    // 应用滤波器并获取输出点云1
    pass1.filter(*filtered1_cloud);
    std::cout << "Filtered1 cloud size: " << filtered1_cloud->size() << std::endl;

     // 创建直通滤波器对象2
    pcl::PassThrough<pcl::PointXYZ> pass2;
    pass2.setInputCloud(filtered1_cloud);
    pass2.setFilterFieldName("y");
    pass2.setFilterLimits(-20, 0);
    pass2.setFilterLimitsNegative(false);
    // 应用滤波器并获取输出点云2
    pass2.filter(*filtered2_cloud);
    std::cout << "Filtered2 cloud size: " << filtered2_cloud->size() << std::endl;

    // 可视化处理后的点云
    pcl::visualization::PCLVisualizer viewer("Filtered Point Cloud");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud<pcl::PointXYZ>(filtered2_cloud, "filtered_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "filtered_cloud");

    // 添加一个0.5倍缩放的坐标系（非必须）
    viewer.addCoordinateSystem (5, transform_T);

    // 记录程序结束时间
    auto end = std::chrono::steady_clock::now();

    // 计算程序运行时间
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Processing time: " << duration.count() << " milliseconds" << std::endl;

    // 直到窗口关闭才结束循环
    while (!viewer.wasStopped()) {
        // 每次循环调用内部的重绘函数
        viewer.spinOnce();
    }
    return 0;
}

