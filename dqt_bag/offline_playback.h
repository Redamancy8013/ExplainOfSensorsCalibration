#ifndef OFFLINE_PLAYBACK_H
#define OFFLINE_PLAYBACK_H
#include "msg/ugv_topics.h"
#include "msg/sensor_topics.h"
#include "my_log.h"
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>

#include <dlfcn.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdarg.h>
#include <fcntl.h>
#include <sys/time.h>

#include <dirent.h>
#include <sys/types.h>    
#include <sys/stat.h> 
#include <sys/time.h>


//BOOOST
#include <boost/tokenizer.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>  
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>

//dnet
#include <shudao.h>

//pcl
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h> 
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>//自定义点云类型时要加

#include <opencv2/opencv.hpp>

typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

static int64_t GetTime(){
	struct timeval tm;
	gettimeofday(&tm, 0);
	int64_t re = (((int64_t)tm.tv_sec)*1000*1000 + tm.tv_usec);
    return re;
} 

//也适用于各种以此格式录制的雷达数据
struct PointRs{
	PCL_ADD_POINT4D;
	float intensity;
	int ring;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointRs,
	(float,x,x)
	(float,y,y)
	(float,z,z)
	(float,intensity,intensity)
	(int, ring, ring)
)

typedef struct {
	float x,y,z,intensity,ring;
} point_data_t;


/**
 * @brief translate string to any format number
 * @param  time input
 */
template<class type>
static type stringToNum(const string& str) {
	istringstream iss(str);
	type num;
	iss >> num;
	return num;
}

static int FileNameFilter(const struct dirent *cur){
	std::string str(cur->d_name);
	if (str.find(".bin") != std::string::npos
			|| str.find(".pcd") != std::string::npos
			|| str.find(".png") != std::string::npos
			|| str.find(".jpg") != std::string::npos
			|| str.find(".txt") != std::string::npos) {
		return 1;
	}
	return 0;
}


/**
 * @brief return the format time
 * @param  time input
 */
static string toDateTime(uint64_t time_in){
	// Convert to chrono nanoseconds type
  	chrono::nanoseconds tp(time_in), tt_nano(time_in);
  	// Get nanoseconds part
  	tp -= chrono::duration_cast<chrono::seconds>(tp);
	//to_time_t方法把时间点转化为time_t
	time_t tt =  std::time_t(chrono::duration_cast<chrono::seconds>(chrono::nanoseconds(tt_nano)).count());
  	//把time_t转化为tm
 	tm *localTm =  localtime(&tt);
 	//tm格式化输出
 	char buffer[20];
 	char format[] = "%Y-%m-%d %H:%M:%S";
 	strftime(buffer, sizeof(buffer), format, localTm);
 	string nanosec_format = "%|09|";
  	stringstream ss;
  	ss<<buffer<<"."<<(boost::format(nanosec_format) % tp.count()).str();
  	return ss.str();
}

class TimeSetting{
public:
	TimeSetting(){};
	~TimeSetting(){};

	/**
	 * @brief translate the data time to current time
	 * @param  time_in data time input
	 */
	int64_t TranslateTime(int64_t time_in);

	/**
	 * @brief translate current time to the data time 
	 * @param  time_in current time input
	 */
	int64_t ReverseTranslateTime(int64_t time_in);

	/**
	 * @brief set the data start time
	 * @param  time_in data time input
	 */
	void SetRealStartTime(int64_t time_in);

	/**
	 * @brief set the current sysytem publish time 
	 * @param  time_in current time input
	 */
	void SetStartTime(int64_t time_in);

	
	int64_t real_start();
	int64_t translate_start();

private:
	int64_t translate_start_;
	int64_t real_start_;
};


class OfflinePub{
public:
    OfflinePub(){
		Init();
	};
    ~OfflinePub(){

    }

    void Init();
	void ReadConfig();
	/**
	 * @brief preprocess the data
	 */
    unsigned char PreProcess();

	/**
	 * @brief input the data path
	 * @param  path: data path
	 */
	void InputFilePath(string& path);
    bool LoadLidarFile();
    bool LoadRadarFile();
	bool LoadImuFile();
    bool LoadCameraFile();
    bool ReadLidarData(pcl::PointCloud<PointRs>& cloud_in,string& file_name);
    bool ReadRadarData(vector<RadarDataNode>& radar_msg_in, string& file_name);
    void PubLidarData();
    void PubRadarData();
    void PubImuData();
    void PubCameraData();
	void GetSeq();

	/**
	 * @brief Find the dataframe closest to the input time
	 */
	void FindStartSeq(int64_t time_in);
    
	/**
	 * @brief start play the data
	 */
	void Start();

	/**
	 * @brief stop play the data
	 */
	void Stop();

	/**
	 * @brief end play current data
	 */
	void End();
	/**
	 * @brief loop publish
	 * @details Start from the beginning when the data reaches the end of the playback
	 */
    void Loop();
    bool GetAllFiles(const std::string& dir_in, std::vector<std::string>& files);

	/**
	 * @brief get the data max recrod time
	 */
	int64_t max_time();

	/**
	 * @brief get the data min recrod time
	 */
	int64_t min_time();
	int64_t duration();

    void Reset();
    /**
     * @brief parseTime
     * @param timestamp in Epoch
     * @return long long int
     *
     * Epoch time conversion
     * http://www.epochconverter.com/programming/functions-c.php
     */
    long long int ParseTime(string timestamp);
	
    bool lidar_flag_; //publish lidar
    bool radar_flag_; //publish radar
    bool imu_flag_;   //publish imu
    bool camera_flag_;//publish camera
    int bag_seq_;
    int pre_bag_seq_;

private:
	dnet_socket_t pub_lidar_;
    cngicuvc::messages::PointClouds pointcloud_message;

	dnet_socket_t pub_imu_;
	cngicuvc::messages::LocatorData imu_message_;
 	dnet_msg_t send_imu_msg_;

	dnet_socket_t pub_imu_heartbeat_;
	dnet_msg_t imu_hearbeat_msg_;
	cngicuvc::messages::HeartBeat heartbeat_data_;
    cngicuvc::messages::Header heartbeat_header_;

	dnet_socket_t pub_radar_;
	cngicuvc::messages::RadarData radar_message_;

    dnet_socket_t pub_camera_;
    cngicuvc::messages::ImageDataTW camera_message_;
    dnet_msg_t send_camera_msg_;

    string data_path_;
    bool stop_flag_;
    bool end_flag_;
    bool lidar_end_; 	//lidar data playback end flag
    bool radar_end_; 	//radar data playback end flag
    bool ins_end_;   	//imu data playback end flag
    bool camera_end_;   //camera data playback end flag
    bool radar_file_;	//open radar file correct
    bool lidar_file_;	//open lidar file correct
    bool ins_file_;		//open imu file correct
    bool camera_file_;  //open camera file correct
	
	std::vector<string> lidar_timestamp_;
	std::vector<string> radar_timestamp_;
	std::vector<string> imu_timestamp_;
    std::vector<string> camera_timestamp_;

	std::vector<int64_t> lidar_timestamps_;
	std::vector<int64_t> radar_timestamps_;
	std::vector<int64_t> imu_timestamps_;
    std::vector<int64_t> camera_timestamps_;


	int64_t max_time_ = 0;
	int64_t min_time_ = DBL_MAX;
	int64_t now_time = 0;
	int64_t duration_;
	int64_t realstart_time_ = 0;

	std::vector<string> lidar_filename_;
	std::vector<string> radar_filename_;
	std::vector<string> imu_data_;
    std::vector<string> camera_filename_;

	std::mutex seq_mutex_;

	TimeSetting time_set;

    int lidar_frame_num_;
    int radar_frame_num_;
    int imu_frame_num_;
    int camera_frame_num_;
    int lidar_seq_;
    int radar_seq_;
    int imu_seq_;
    int camera_seq_;

	string bag_time_;
    string lidar_time_;
    string radar_time_;
    string imu_time_;
    string camera_time_;
	string bag_begin_time_;
	string bag_end_time_;
};

#endif //OFFLINE_PLAYBACK_H
