#include "offline_playback.h"

void OfflinePub::Init(){

    //##################### Lidar #############################
	if(pub_lidar_.sock_init(DNET_PUB, 10)){
		printf("[INFO] pub lidar socket init ok \n");
	}
	if(pub_lidar_.sock_addr((char *)ADDR_LIDARDATA_IPC, true)){
		printf("[INFO] pub lidar socket addr ok \n");
	}

    //##################### Radar #############################
	if(pub_radar_.sock_init(DNET_PUB, 20)){
		printf("[INFO] pub radar socket init ok \n");
	}
	if(pub_radar_.sock_addr((char *)ADDR_RADARDATA_IPC, true)){
		printf("[INFO] pub radar socket addr ok \n");
	}

    //##################### IMU #############################
    if(pub_imu_heartbeat_.sock_init(DNET_PUB, 100)){
		printf("[INFO] pub imu heartbeat socket init ok \n");
    }

    if(pub_imu_heartbeat_.sock_addr((char *)ADDR_HEART_BEAT_IPC, false)){
		printf("[INFO] pub imu heartbeat socket init ok \n");
    }

    heartbeat_data_.status = 0xff;

	if(pub_imu_.sock_init(DNET_PUB, 10)){
		printf("[INFO] pub imu socket init ok \n");
	}
    if(pub_imu_.sock_addr((char*)ADDR_LOCATOR_DRIVER_IPC, true)){
		printf("[INFO] pub imu socket addr ok \n");
	}

    // ReadConfig();
    end_flag_   = true;
	stop_flag_  = true;
    lidar_flag_ = true;
    radar_flag_ = true;
    imu_flag_   = true;

	lidar_seq_ = 0;
	radar_seq_ = 0;
	imu_seq_ = 0;
	lidar_frame_num_ = 0;
	radar_frame_num_ = 0;
	imu_frame_num_ = 0;

	duration_ = 0;
}

void OfflinePub::ReadConfig(){
	char *buffer;
    string buffers;
    if((buffer = getcwd(NULL, 0)) == NULL){
		perror("getcwd error");
    }
    else{
        printf("%s\n", buffer);
		buffers = buffer;
		free(buffer);
    }

    boost::char_separator<char> sep { "/" };
    tokenizer tokn(buffers, sep);
    vector<string> path(tokn.begin(), tokn.end());
    string config_file;
	for(int i=0; i<path.size()-2; ++i){
		config_file += "/" + path[i];
   	}
	config_file += "/config/offline_param.xml";
	boost::property_tree::ptree pt;
    boost::property_tree::xml_parser::read_xml(config_file, pt);
	lidar_flag_ = pt.get<bool>("offlineParam.lidar");
	radar_flag_ = pt.get<bool>("offlineParam.radar");
	imu_flag_	= pt.get<bool>("offlineParam.imu");
	data_path_	= pt.get<string>("offlineParam.datapath");
}

void OfflinePub::InputFilePath(string & path){
    data_path_ = path;
}

bool OfflinePub::PreProcess(){
    if(!LoadLidarFile()){
		printf("\033[40;31m [ERROR] Load Lidar File Ouccur Mistake, Please Check the data path! \033[0m \n");
		return false;
	}
    if(!LoadRadarFile()){
		printf("\033[40;31m [ERROR] Load Radar File Ouccur Mistake, Please Check the data path! \033[0m \n");
		return false;
	}
    if(!LoadImuFile()){
		printf("\033[40;31m [ERROR] Load Imu File Ouccur Mistake, Please Check the data path! \033[0m \n");
		return false;
	}
    return true;
}

long long int OfflinePub::ParseTime(string timestamp)
{
    // example: 2011-09-26 13:21:35.134391552
    //          01234567891111111111222222222
    //                    0123456789012345678
    struct tm t = {0};  // Initalize to all 0's
    t.tm_year = boost::lexical_cast<int>(timestamp.substr(0, 4)) - 1900;
    t.tm_mon  = boost::lexical_cast<int>(timestamp.substr(5, 2)) - 1;
    t.tm_mday = boost::lexical_cast<int>(timestamp.substr(8, 2));
    t.tm_hour = boost::lexical_cast<int>(timestamp.substr(11, 2));
    t.tm_min  = boost::lexical_cast<int>(timestamp.substr(14, 2));
    t.tm_sec  = boost::lexical_cast<int>(timestamp.substr(17, 2));
    t.tm_isdst = -1;
    time_t timeSinceEpoch = mktime(&t);

    long long int time = timeSinceEpoch * 1e6 + boost::lexical_cast<int>(timestamp.substr(20, 9))/1e3;
    return time;
}

bool OfflinePub::GetAllFiles(const std::string& dir_in, std::vector<std::string>& files){

	if (dir_in.empty()) {
		return false;
	}
	struct stat s;
	stat(dir_in.c_str(), &s);
	if (!S_ISDIR(s.st_mode)) {
		return false;
	}
	DIR* open_dir = opendir(dir_in.c_str());
	if (NULL == open_dir) {
		std::exit(EXIT_FAILURE);
	}
	dirent* p = nullptr;
	while ((p = readdir(open_dir)) != nullptr) {
		struct stat st;
		if (p->d_name[0] != '.') {
			//因为是使用devC++ 获取windows下的文件，所以使用了 "\" ,linux下要换成"/"
			std::string name = dir_in + std::string("/")
					+ std::string(p->d_name);
			stat(name.c_str(), &st);
			if (S_ISDIR(st.st_mode)) {
				GetAllFiles(name, files);
			} else if (S_ISREG(st.st_mode)) {
				boost::char_separator<char> sepp { "." };
				tokenizer tokn(std::string(p->d_name), sepp);
				vector<string> filename_sep(tokn.begin(), tokn.end());
				string type_ = "." + filename_sep[1];
				break;
			}
		}
	}

	struct dirent **namelist;
	int n = scandir(dir_in.c_str(), &namelist, FileNameFilter, alphasort);
	if (n < 0) {
		return false;
	}
	for (int i = 0; i < n; ++i) {
		std::string filePath(namelist[i]->d_name);
		files.push_back(filePath);
		free(namelist[i]);
	};
	free(namelist);
	closedir(open_dir);
	return true;
}

bool OfflinePub::LoadLidarFile(){
    string lidar_file_path = data_path_ + "/lidar/data/";
	if(!GetAllFiles(lidar_file_path, lidar_filename_))
		return false;

	string time_path = data_path_ + "/lidar/lidar_timestamp.txt";
	std::ifstream lidartime(time_path);
	if (lidartime) {
		boost::char_separator<char> sep_line { "\n" };
		std::stringstream buffer;
		buffer << lidartime.rdbuf();
		std::string contents(buffer.str());
		tokenizer tok_line(contents, sep_line);
		std::vector<std::string> lines(tok_line.begin(), tok_line.end());
		lidar_timestamp_ = lines;
		int size = lidar_timestamp_.size();
		for(auto time_stamp:lidar_timestamp_){
			int64_t lidartime = ParseTime(time_stamp);
			lidar_timestamps_.push_back(lidartime);
			min_time_ = min(lidartime, min_time_);
			max_time_ = max(lidartime, max_time_);
		}
		if(size < lidar_filename_.size()){
			printf("\033[40;31m [ERROR] File number wrong! \033[0m \n");
		    return false;
		}
	}

	lidar_frame_num_ = lidar_filename_.size();
	return true;
}

bool OfflinePub::LoadRadarFile(){
	string radar_file_path = data_path_ + "/radar/data/";
	if(!GetAllFiles(radar_file_path, radar_filename_))
		return false;
	string time_path = data_path_ + "/radar/radar_timestamp.txt";

	std::ifstream radartime(time_path);
	if (radartime){
		boost::char_separator<char> sep_line { "\n" };
		std::stringstream buffer;
		buffer << radartime.rdbuf();
		std::string contents(buffer.str());
		tokenizer tok_line(contents, sep_line);
		std::vector<std::string> lines(tok_line.begin(), tok_line.end());
		radar_timestamp_ = lines;
		int size = radar_timestamp_.size();
		for(auto time_stamp:radar_timestamp_){
			int64_t radartime = ParseTime(time_stamp);
			radar_timestamps_.push_back(radartime);
			min_time_ = min(radartime, min_time_);
			max_time_ = max(radartime, max_time_);
		}
		if(size < radar_timestamp_.size()){
			printf("\033[40;31m [ERROR] File number wrong! \033[0m \n");
			return false;
		}
	}
	radar_frame_num_ = radar_filename_.size();
    return true;
}

bool OfflinePub::LoadImuFile(){
	string imu_data_path = data_path_ + "/navigation/nav_data.txt";

	std::ifstream imudata(imu_data_path);
	if (imudata) {
		boost::char_separator<char> sep_line { "\n" };
		std::stringstream buffer;
		buffer << imudata.rdbuf();
		std::string contents(buffer.str());
		tokenizer tok_line(contents, sep_line);
		std::vector<std::string> lines(tok_line.begin(), tok_line.end());
		imu_data_ = lines;
	}

	string time_path = data_path_ + "/navigation/nav_timestamp.txt";

	std::ifstream imutime(time_path);
	if (imutime) {
		boost::char_separator<char> sep_line { "\n" };
		std::stringstream buffer;
		buffer << imutime.rdbuf();
		std::string contents(buffer.str());
		tokenizer tok_line(contents, sep_line);
		std::vector<std::string> lines(tok_line.begin(), tok_line.end());
		imu_timestamp_ = lines;
		int size = imu_timestamp_.size();
		for(auto time_stamp:imu_timestamp_){
			int64_t imutime = ParseTime(time_stamp);
			imu_timestamps_.push_back(imutime);
			min_time_ = min(imutime, min_time_);
			max_time_ = max(imutime, max_time_);
		}
		if(size < imu_data_.size()){
			printf("\033[40;31m [ERROR] File number wrong! \033[0m \n");
			return false;
		}
	}

	imu_frame_num_ = imu_data_.size();
    return true;
}
int64_t ltime = 0;

void OfflinePub::PubLidarData(){
    while(true){
		if(!end_flag_ || lidar_seq_ > (lidar_frame_num_ - 1)){
			break;
		}

		if(stop_flag_)
			continue;

		int64_t t0 = GetTime();
		string file_path = data_path_ + "/lidar/data/" + lidar_filename_[lidar_seq_];
		pcl::PointCloud<PointRs>::Ptr cloud(new pcl::PointCloud<PointRs>);
		ReadLidarData(*cloud,file_path);
		printf("[INFO] Lidar file path: %s \n", file_path.c_str());

		pointcloud_message.header.time_stamp = (lidar_timestamps_[lidar_seq_]) *1e-3;
		int64_t sub_time;

		pointcloud_message.size = cloud->points.size();
		pointcloud_message.header.seq++;
		pointcloud_message.header.message_type = cngicuvc::messages::MessageType::PointClouds; 
		for(int k =0; k<cloud->points.size(); ++k){
			pointcloud_message.points[k].x = cloud->points[k].x;
			pointcloud_message.points[k].y = cloud->points[k].y;
			pointcloud_message.points[k].z = cloud->points[k].z;
			pointcloud_message.points[k].ring = cloud->points[k].ring;
			pointcloud_message.points[k].intensity = cloud->points[k].intensity;
		}
		unsigned int size_header = (unsigned int)sizeof(cngicuvc::messages::Header);
		unsigned int size_all = (unsigned int)sizeof(cngicuvc::messages::PointClouds);
		dnet_msg_t send_msg(&pointcloud_message.size, size_all - size_header);

		int64_t now = GetTime();
		int64_t translate = time_set.TranslateTime(lidar_timestamps_[lidar_seq_]);
		std::this_thread::sleep_for(std::chrono::microseconds(translate - now));

		if(lidar_flag_){
			int num_send = pub_lidar_.send_pack(&pointcloud_message.header, size_header, &send_msg);
			printf("[INFO] Pub Lidar Data! msg size: %d \n", cloud->points.size());
		}
        
		++lidar_seq_;

		int64_t t1 = GetTime();
		string currentt = toDateTime(t1*1e3);
		printf("[INFO] Lidar time : %s \n", lidar_timestamp_[(lidar_seq_-1)].c_str());
		printf("[INFO] lidar pub time : %s \n", currentt.c_str());
	}
	return;
}

bool OfflinePub::ReadLidarData(pcl::PointCloud<PointRs>& cloud_in,string& file_name){
	std::ifstream inputfile;
	inputfile.open(file_name, ios::binary);
	if (!inputfile) {
		cerr << "ERROR: Cannot open file " << file_name
					<< "! Aborting..." << endl;
		return false;
	}
	inputfile.seekg(0, ios::beg);
	for (int i = 0; inputfile.good() && !inputfile.eof(); i++) {
		point_data_t data;
		inputfile.read(reinterpret_cast<char*>(&data), sizeof(data));
		PointRs p;
		p.x = data.x;
		p.y = data.y;
		p.z = data.z;
		p.intensity = data.intensity;
		p.ring = data.ring;
		cloud_in.points.push_back(p);
	}
	return true;
}

int64_t timet = 0;

void OfflinePub::PubRadarData(){
    while(true){
		if(!end_flag_ || radar_seq_ > (radar_frame_num_ - 1)){
			break;
		}
		if(stop_flag_){
			continue;
		}

		int64_t t0 = GetTime();
		string file_path = data_path_ + "/radar/data/" + radar_filename_[radar_seq_];
		printf("[INFO] Radar file path: %s \n", file_path.c_str());

		radar_message_.header.time_stamp = radar_timestamps_[radar_seq_] *1e-3;
		radar_time_ = radar_timestamp_[radar_seq_];
		radar_message_.header.seq++;
	    radar_message_.header.message_type = cngicuvc::messages::MessageType::RadarData;

		vector<RadarDataNode> radar_vec;
		ReadRadarData(radar_vec, file_path);
		for(int k=0; k<64; ++k){
			radar_message_.data[k] = radar_vec[k];
		}

		int64_t now = GetTime();
		int64_t translate = time_set.TranslateTime(radar_timestamps_[radar_seq_]);
		std::this_thread::sleep_for(std::chrono::microseconds(translate - now));

	    unsigned int size_header = (unsigned int)sizeof(cngicuvc::messages::Header);
	    unsigned int size_all = (unsigned int)sizeof(cngicuvc::messages::RadarData);
	    dnet_msg_t send_msg(&radar_message_.data, size_all - size_header);

		if(radar_flag_)	{
		    int num_send = pub_radar_.send_pack(&radar_message_.header, size_header, &send_msg);
	    	printf("[INFO] Pub Radar Data! \n");
		}
        
		radar_seq_++;

		int64_t t1 = GetTime();
		printf("[INFO] Radar time : %s \n", radar_timestamp_[(radar_seq_-1)].c_str());
		string timett = toDateTime(t1*1e3);
		printf("[INFO] Radar pub time : %s \n", timett.c_str());
	}
	return;
}

bool OfflinePub::ReadRadarData(vector<RadarDataNode>& radar_msg_in,  string& file_name){
	std::vector<string> radar_data_s;
	radar_msg_in.resize(64);
	std::ifstream radar_data(file_name);
	if (radar_data) {
		boost::char_separator<char> sep_line { "\n" };
		std::stringstream buffer;
		buffer << radar_data.rdbuf();
		std::string contents(buffer.str());
		tokenizer tok_line(contents, sep_line);
		std::vector<std::string> lines(tok_line.begin(), tok_line.end());
		radar_data_s = lines;
		int size = radar_data_s.size();
		for(int i=0; i<size; ++i){
			boost::char_separator<char> sep_blank { " " };
			tokenizer tok_part(radar_data_s[i], sep_blank);
			std::vector<std::string> data_part(tok_part.begin(), tok_part.end());
			radar_msg_in[i].idx 	 = stringToNum<int>(data_part[1]);
			radar_msg_in[i].angle 	 = stringToNum<float>(data_part[2]);
			radar_msg_in[i].flag 	 = stringToNum<bool>(data_part[3]);
			radar_msg_in[i].velocity = stringToNum<float>(data_part[4]);
			radar_msg_in[i].range 	 = stringToNum<float>(data_part[5]);
			radar_msg_in[i].localX 	 = stringToNum<double>(data_part[6]);
			radar_msg_in[i].localY 	 = stringToNum<double>(data_part[7]);
		}
	}else{
		return false;
	}
	return true;
}

void OfflinePub::PubImuData(){
    while(true){
		if(!end_flag_ || imu_seq_ > (imu_frame_num_ - 1)){
			break;
		}
		
		if(stop_flag_){
			continue;
		}

    	memset(&imu_message_,0,sizeof(imu_message_));
		imu_message_.header.time_stamp = ParseTime(imu_timestamp_[imu_seq_]) *1e-3;
		imu_time_ = imu_timestamp_[imu_seq_];
		imu_message_.header.seq++;
		imu_message_.header.message_type = cngicuvc::messages::MessageType::LOCATOR_DATA;
		boost::char_separator<char> sep_blank { " " };
		tokenizer tok_part(imu_data_[imu_seq_], sep_blank);
		std::vector<std::string> data_part(tok_part.begin(), tok_part.end());
		imu_message_.longitude 		= stringToNum<double>(data_part[0])*M_PI/180.0;
		imu_message_.latitude 		= stringToNum<double>(data_part[1])*M_PI/180.0;
		imu_message_.altitude 		= stringToNum<double>(data_part[2])*10;
		imu_message_.yaw 			= stringToNum<double>(data_part[3])*M_PI/180.0;
		imu_message_.roll 			= stringToNum<double>(data_part[6])*M_PI/180.0;
		imu_message_.pitch		 	= stringToNum<double>(data_part[7])*M_PI/180.0;
		imu_message_.velocity_all 	= stringToNum<double>(data_part[8]);
		imu_message_.GPS_state 		= stringToNum<int>(data_part[9]);

		imu_message_.year	= boost::lexical_cast<int>(imu_timestamp_[imu_seq_].substr(0, 4));
		imu_message_.month  = boost::lexical_cast<int>(imu_timestamp_[imu_seq_].substr(5, 2));
		imu_message_.day 	= boost::lexical_cast<int>(imu_timestamp_[imu_seq_].substr(8, 2));
		imu_message_.hour 	= boost::lexical_cast<int>(imu_timestamp_[imu_seq_].substr(11, 2));
		imu_message_.min  	= boost::lexical_cast<int>(imu_timestamp_[imu_seq_].substr(14, 2));
		imu_message_.sec  	= boost::lexical_cast<int>(imu_timestamp_[imu_seq_].substr(17, 2));

		send_imu_msg_.re_data(&imu_message_.wc_x, sizeof(imu_message_) - sizeof(imu_message_.header));

		int64_t now = GetTime();
		int64_t translate = time_set.TranslateTime(imu_timestamps_[imu_seq_]);
		std::this_thread::sleep_for(std::chrono::microseconds(translate - now));
		
		if(imu_flag_){
			int num_send = pub_imu_.send_pack(&imu_message_.header, sizeof(imu_message_.header), &send_imu_msg_);
        	printf("[INFO] Pub IMU Data \n");
		}

		heartbeat_header_.message_type = cngicuvc::messages::MessageType::XWGI_HEART_BEAT;
		heartbeat_header_.seq++;
		imu_hearbeat_msg_.re_data((void *)&heartbeat_data_, sizeof(heartbeat_data_));
		pub_imu_heartbeat_.send_pack((void *)&heartbeat_header_, sizeof(heartbeat_header_), &imu_hearbeat_msg_);
        dnet_msleep(1);
		++imu_seq_;
	}
	return;
}

void OfflinePub::Start(){
	end_flag_ = true;
	bool start_flag = false;
	bool loop = false;
	while(true){
		if(!start_flag){
			int64_t now = GetTime();
			time_set.SetStartTime(now);
			time_set.SetRealStartTime(min_time_);
			std::thread pub_radar(&OfflinePub::PubRadarData, this);
			pub_radar.detach();
			std::thread pub_imu(&OfflinePub::PubImuData, this);
			pub_imu.detach();
			std::thread pub_lidar(&OfflinePub::PubLidarData, this);
			pub_lidar.detach();
			start_flag = true;
		}
		int64_t now = GetTime();
		if(now > (time_set.translate_start() + (max_time_ - min_time_))){
			imu_seq_ = 0;
			lidar_seq_ = 0;
			radar_seq_ = 0;
			start_flag = false;
		}
		if(!loop)
			break;
	}
}

void OfflinePub::End(){
    end_flag_ = false;
}

void OfflinePub::Stop(){
    stop_flag_ = !stop_flag_;
	if(stop_flag_){
		int64_t now = GetTime();
		int64_t duration = now - time_set.translate_start();
		realstart_time_ = time_set.real_start() + duration;
		time_set.SetRealStartTime(realstart_time_);
	}else{
		int64_t now = GetTime();
		time_set.SetStartTime(now);
	}
}

int BinarySearch(vector<int64_t>& times, int64_t time_search){
	int right = times.size();
	int left = 0;
	while(left<right){
		int mid = (left + right)/2;
		if(times[mid] > time_search){
			right = mid;
		}else if(times[mid] < time_search){
			left = mid + 1;
		}else{
			return mid;
		}
	}
	return right;
}


void OfflinePub::FindStartSeq(int64_t time_in){
	std::unique_lock<std::mutex> seqlock(seq_mutex_);
	time_in += min_time_;
	lidar_seq_ = BinarySearch(lidar_timestamps_, time_in);
	radar_seq_ = BinarySearch(radar_timestamps_, time_in);
	imu_seq_ = BinarySearch(imu_timestamps_, time_in);
	// cout<<"lidar "<<lidar_timestamp_[lidar_seq_]<<" "<<lidar_seq_<<endl;
	// cout<<"radar "<<radar_timestamp_[radar_seq_]<<" "<<radar_seq_<<endl;
	// cout<<"imu "<<imu_timestamp_[imu_seq_]<<" "<<imu_seq_<<endl;
	// cout<<"find time "<<toDateTime(time_in*1e3)<<endl;
	time_set.SetRealStartTime(time_in);
	seqlock.unlock();
}

int64_t OfflinePub::max_time(){
	return max_time_;
}

int64_t OfflinePub::min_time(){
	return min_time_;
}

int64_t OfflinePub::duration(){
	int64_t now = GetTime();
	int64_t realt = time_set.ReverseTranslateTime(now);
	return  (realt - min_time_)*1e-3;
}

void TimeSetting::SetRealStartTime(int64_t time_in){
	real_start_ = time_in;
}

void TimeSetting::SetStartTime(int64_t time_in){
	translate_start_ = time_in;
}

int64_t TimeSetting::TranslateTime(int64_t time_in){
	return translate_start_ + (time_in - real_start_);
}

int64_t TimeSetting::ReverseTranslateTime(int64_t time_in){
	return real_start_ + (time_in - translate_start_);
}

int64_t TimeSetting::real_start(){
	return real_start_;
}

int64_t TimeSetting::translate_start(){
	return translate_start_;
}