/** 
 * \brief A module that receives the control command from  other module.
 * \author ZBSu
 * 
 */
#include <receive/receive_module.h>
#include <cstring>

#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

//pcl库
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>

#include <unistd.h>
#include <netinet/in.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <stdarg.h>
#include <fcntl.h>
#include <sys/time.h>
#include <mutex>

#define  ADDR_LIDARDATA_IPC "ipc:///tmp/ADDR_3DLIDARDATA_IPC"

pcl::PointCloud<PointCfans>::Ptr show_cloud_g (new pcl::PointCloud<PointCfans>);
// pcl::PointCloud<PointCfans>::Ptr select_cloud_ (new pcl::PointCloud<PointCfans>);
std::mutex lidar_mutex;
int count_g = 0;
int precount = 0;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
bool select_ = true;


namespace cngicuvc
{
ReceiveModule::ReceiveModule()	
{
	onInit();
}

ReceiveModule::~ReceiveModule()
{
	
}

void ReceiveModule::onInit()
{

    subscriber.sock_init(DNET_SUB, 10);
    subscriber.sock_addr((char *)ADDR_LIDARDATA_IPC, false);
	
}

void ReceiveModule::task0_loop()
{
	is_stopped[0] = false;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewert(new pcl::visualization::PCLVisualizer);
	viewer = viewert;
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    viewer->addCoordinateSystem(1);
	pcl::PointCloud<PointCfans>::Ptr line_cloud(new pcl::PointCloud<PointCfans>);
	int total_pointnum = 147;
	line_cloud->points.resize(total_pointnum);
	int x_start = -15;
	int y_start = 105;
	for(int i=0; i<total_pointnum; ++i){
		if(i%7==0){
			x_start = -15;
		}
		line_cloud->points[i].y = x_start;
		x_start += 5;
	}

	for(int i=0; i<total_pointnum; ++i){
		if(i%7==0){
			y_start -=5;
		}
		line_cloud->points[i].x = y_start;
	}

	for(int i=0; i<total_pointnum; ++i){
		if((i+1)%7==0)
			continue;
		viewer->addLine(line_cloud->points[i],line_cloud->points[i+1], to_string(i));
	}


	for(int i=0; i<total_pointnum; ++i){
		if((i+7)>=total_pointnum)
			continue;
		viewer->addLine(line_cloud->points[i],line_cloud->points[i+7],"line"+to_string(i));
	}

	y_start = 105;
	for(int i=0; i<total_pointnum;){
		y_start -= 5;
		viewer->addText3D(to_string(y_start),line_cloud->points[i],1.0, 10.0, 10.0, 10.0, "meter"+to_string(i));
		i+=7;
	}

	viewer->setCameraPosition( -74.5549, -1.22657, 119.982, 23.228, -1.65148, 20.3152, 0.713823, 0.00377888, 0.700316);
  	// viewer->registerAreaPickingCallback(AreaPickingCallback);

	pcl::PointCloud<PointCfans>::Ptr show_cloud(new pcl::PointCloud<PointCfans>);
	// pcl::PointCloud<PointCfans>::Ptr tempselect_cloud(new pcl::PointCloud<PointCfans>);
	show_cloud_g = show_cloud;
	// select_cloud_ = tempselect_cloud;

	pcl::visualization::PointCloudColorHandlerCustom <PointCfans> color(show_cloud, (255), (0),(0));
	// pcl::visualization::PointCloudColorHandlerCustom <PointCfans> colorselect(select_cloud_, (0), (255),(0));

	string cs = "cloud";
	string css = "selectcloud";

	while (!is_stopping[0]&&!viewer->wasStopped())
	{

		taskReceivePackage();

		std::unique_lock<std::mutex> mlock(lidar_mutex);
		
		cout<<show_cloud_g->points.size()<<" "<<precount<<" "<<count_g<<endl;
		
		if(show_cloud_g->points.size()>0 && precount != count_g){
			cout<<"show cloud"<<endl;
			precount = count_g;
			viewer->removeAllPointClouds();
			viewer->addPointCloud(show_cloud_g,color, cs);
		}

		// if(select_){
		// 	viewer->removePointCloud(css);

		// 	cout<<select_cloud_->points.size()<<endl;

		// 	viewer->addPointCloud(select_cloud_,colorselect, css);
		// 	// select_ = false;
		// 	viewer->setPointCloudRenderingProperties(
		// 	pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, css);
		// }

		mlock.unlock();
		viewer->spinOnce(100);

	}	

	// select_ = false;

	is_stopped[0] = true;
}

void ReceiveModule::taskReceivePackage()
{

	dnet_msg_t recv_msg;
    cngicuvc::messages::PointClouds lidar_msg;

	int recv_size = subscriber.recv_pack(&lidar_msg.header, sizeof(lidar_msg.header), recv_msg);

	if(recv_size<0)
		return;

	switch(lidar_msg.header.message_type){
		case cngicuvc::messages::MessageType::PointClouds:{
			timeval tv;
			gettimeofday(&tv,NULL);
			cout<<"receive"<<endl;
			// int64_t now = tv.tv_sec*1e6 + tv.tv_usec;
			// pretime = now;
			memcpy(&lidar_msg.size, recv_msg.data(), sizeof(lidar_msg) - sizeof(lidar_msg.header));
			std::unique_lock<std::mutex> mlock(lidar_mutex);
			show_cloud_g->points.clear();
			// select_cloud_->points.clear();
			for(int i=0; i<lidar_msg.size; ++i){
				PointCfans p;
				p.x = lidar_msg.points[i].x;
				p.y = lidar_msg.points[i].y;
				p.z = lidar_msg.points[i].z;
				p.laserid = lidar_msg.points[i].ring;
				p.intensity = lidar_msg.points[i].intensity;
				show_cloud_g->points.push_back(p);		
			}
			count_g++;
			mlock.unlock();
			break;
		}
		default:
			break;
	}
}
}//end namespace

