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

    if(pub_camera_.sock_init(DNET_PUB, 10)){
        printf("[INFO] pub camera socket init ok \n");
    }
    if(pub_camera_.sock_addr((char*)ADDR_OBJECT_DETECT_CAMBRICON_IPC, true)){
        printf("[INFO] pub camera socket addr ok \n");
    }

    // ReadConfig();
    Reset();
}

void OfflinePub::Reset(){
    cout<<"reset"<<endl;
    lidar_timestamp_.clear();
    radar_timestamp_.clear();
    imu_timestamp_.clear();
    camera_timestamp_.clear();
    lidar_timestamps_.clear();
    lidar_filename_.clear();
    radar_timestamps_.clear();
    radar_filename_.clear();
    camera_timestamps_.clear();
    camera_filename_.clear();
    imu_timestamps_.clear();
    imu_data_.clear();

    end_flag_   = true;
    stop_flag_  = true;

    lidar_flag_  = true;
    radar_flag_  = true;
    imu_flag_    = true;
    camera_flag_ = true;

    lidar_end_  = false;
    ins_end_    = false;
    radar_end_  = false;
    camera_end_ = false;

    lidar_file_ = true;
    radar_file_ = true;
    ins_file_   = true;
    camera_file_ = true;

    lidar_seq_  = 0;
    radar_seq_  = 0;
    imu_seq_    = 0;
    camera_seq_ = 0;

    lidar_frame_num_  = 0;
    radar_frame_num_  = 0;
    imu_frame_num_    = 0;
    camera_frame_num_ = 0;

    duration_ = 0;

    max_time_ = 0;
    min_time_ = DBL_MAX;

    now_time = 0;
    realstart_time_ = 0;
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
    camera_flag_= pt.get<bool>("offlineParam.camera");
    data_path_	= pt.get<string>("offlineParam.datapath");
}

void OfflinePub::InputFilePath(string & path){
    data_path_ = path;
}

unsigned char OfflinePub::PreProcess(){
    unsigned char file_status = 0x00;
    Reset();
    if(!LoadLidarFile()){
        LOG(ERROR) << "Load Lidar File Ouccur Mistake, Please Check the data path!"
                    << '\n';
		printf("\033[40;31m [ERROR] Load Lidar File Ouccur Mistake, Please Check the data path! \033[0m \n");
        lidar_file_ = false;
        lidar_end_ = true;
        file_status += 1<<0;
    }else{
        LOG(INFO) << "Load Lidar File!"
                    << '\n';
    }

    if(!LoadRadarFile()){
        LOG(ERROR) << "Load Radar File Ouccur Mistake, Please Check the data path! "
                    << '\n';
		printf("\033[40;31m [ERROR] Load Radar File Ouccur Mistake, Please Check the data path! \033[0m \n");
        radar_file_ = false;
        radar_end_ = true;
        file_status += 1<<1;
    }else{
        LOG(INFO) << "Load Radar File!"
                    << '\n';
    }

    if(!LoadImuFile()){
        LOG(ERROR) << "Load Imu File Ouccur Mistake, Please Check the data path!"
                    << '\n';
		printf("\033[40;31m [ERROR] Load Imu File Ouccur Mistake, Please Check the data path! \033[0m \n");
        ins_file_ = false;
        ins_end_ = true;
        file_status += 1<<2;
    }else{
        LOG(INFO) << "Load Imu File!"
                    << '\n';
    }

    if(!LoadCameraFile()){
        LOG(ERROR) << "Load Camera File Ouccur Mistake, Please Check the data path!"
                    << '\n';
        printf("\033[40;31m [ERROR] Load Camera File Ouccur Mistake, Please Check the data path! \033[0m \n");
        camera_file_ = false;
        camera_end_ = true;
        file_status += 1<<3;
    }else{
        LOG(INFO) << "Load Camera File!"
                    << '\n';
    }

    if(lidar_timestamps_.size() < lidar_filename_.size()
            || lidar_timestamps_.size() == 0
            || lidar_filename_.size() == 0){
        printf("\033[40;31m [ERROR] File number wrong! \033[0m \n");
        LOG(ERROR) << "Load Lidar File number wrong!!"
                    << '\n';
        file_status += 1<<4;
    }

    if(radar_timestamps_.size() < radar_filename_.size()
            || radar_timestamps_.size() == 0
            || radar_filename_.size() == 0){
        printf("\033[40;31m [ERROR] File number wrong! \033[0m \n");
        LOG(ERROR) << "Load Radar File number wrong!!"
                    << '\n';
        file_status += 1<<5;
    }

    if(imu_timestamps_.size() < imu_data_.size()
            || imu_timestamps_.size() == 0
            || imu_data_.size() == 0){
        printf("\033[40;31m [ERROR] File number wrong! \033[0m \n");
        LOG(ERROR) << "Load Imu File number wrong!!"
                    << '\n';
        file_status += 1<<6;
    }

    if(camera_timestamps_.size() < camera_filename_.size()
            || camera_timestamps_.size() == 0
            || camera_filename_.size() == 0){
        printf("\033[40;31m [ERROR] File number wrong! \033[0m \n");
        LOG(ERROR) << "Load Camera File number wrong!!"
                    << '\n';
        file_status += 1<<7;
    }
//    cout<<"status "<<(int)file_status<<endl;
    return file_status;
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
        cout<<"lidar mintime "<<min_time_<<endl;

        if(size < lidar_filename_.size() || lidar_filename_.size() == 0){
		    return false;
		}
    }else{
        printf("\033[40;31m [ERROR] Lidar Timestamp file open wrong! \033[0m \n");
        LOG(ERROR) << "Load Lidar timestamp File wrong!!"
                    << '\n';
        return false;
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
		printf("\033[40;31m [ERROR] File number wrong! \033[0m \n");
        LOG(ERROR) << "Load Radar File number wrong!!"
                    << '\n';
		std::string contents(buffer.str());
		tokenizer tok_line(contents, sep_line);
		std::vector<std::string> lines(tok_line.begin(), tok_line.end());
		radar_timestamp_ = lines;
		int size = radar_timestamp_.size();
        cout<<"radar_timestamp_ "<<radar_timestamp_[0]<<" "<<ParseTime(radar_timestamp_[0])<<endl;
		for(auto time_stamp:radar_timestamp_){
			int64_t radartime = ParseTime(time_stamp);
			radar_timestamps_.push_back(radartime);
            if(radartime == 1651363200921000)
                cout<<time_stamp<<endl;
			min_time_ = min(radartime, min_time_);
			max_time_ = max(radartime, max_time_);
		}
        cout<<"radar mintime "<<min_time_<<endl;

        if(size < radar_filename_.size()|| radar_filename_.size() == 0){
			return false;
		}
    }else{
        printf("\033[40;31m [ERROR] Radar Timestamp file open wrong! \033[0m \n");
        LOG(ERROR) << "Load Radar timestamp File wrong!!"
                    << '\n';
        return false;
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
	}else{
		return false;
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
        cout<<"imu mintime "<<min_time_<<endl;
        if(size < imu_data_.size() || imu_data_.size()==0){
			return false;
		}
    }else{
        printf("\033[40;31m [ERROR] Imu Timestamp file open wrong! \033[0m \n");
        LOG(ERROR) << "Load Imu timestamp File wrong!!"
                    << '\n';
        return false;
    }

	imu_frame_num_ = imu_data_.size();
    return true;
}


bool OfflinePub::LoadCameraFile(){
    string camera_file_path = data_path_ + "/camera/data/";
    if(!GetAllFiles(camera_file_path, camera_filename_))
        return false;
    string time_path = data_path_ + "/camera/cam_timestamp.txt";

    std::ifstream cameratime(time_path);
    if (cameratime){
        boost::char_separator<char> sep_line { "\n" };
        std::stringstream buffer;
        buffer << cameratime.rdbuf();
        std::string contents(buffer.str());
        tokenizer tok_line(contents, sep_line);
        std::vector<std::string> lines(tok_line.begin(), tok_line.end());
        camera_timestamp_ = lines;
        int size = camera_timestamp_.size();
        for(auto time_stamp:camera_timestamp_){
            int64_t cameratime = ParseTime(time_stamp);
            camera_timestamps_.push_back(cameratime);
            min_time_ = min(cameratime, min_time_);
            max_time_ = max(cameratime, max_time_);
        }
        if(size < camera_filename_.size() || camera_filename_.size() == 0){
            return false;
        }
    }else{
        printf("\033[40;31m [ERROR] Camera Timestamp file open wrong! \033[0m \n");
        LOG(ERROR) << "Load Camera timestamp File wrong!!"
                    << '\n';
        return false;
    }
    camera_frame_num_ = camera_filename_.size();
    return true;
}

void OfflinePub::PubLidarData(){
    cout<<"pub lidar"<<endl;
    while(true){
		if(!end_flag_ || lidar_seq_ > (lidar_frame_num_ - 1)){
			break;
		}

        if(stop_flag_){
            usleep(1e4);
            continue;
        }
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
        cout<<"translate - now "<<(translate - now)<<" lidar_timestamps_ "<<lidar_timestamps_[lidar_seq_]<<endl;
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
    lidar_end_ = true;
	return;
}

bool OfflinePub::ReadLidarData(pcl::PointCloud<PointRs>& cloud_in,string& file_name){
	std::ifstream inputfile;
	inputfile.open(file_name, ios::binary);
	if (!inputfile) {
		cerr << "ERROR: Cannot open file " << file_name
					<< "! Aborting..." << endl;
        LOG(ERROR) << "Cannot open file "<< file_name
                    <<"! Aborting..." << '\n';
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

void OfflinePub::PubRadarData(){
    cout<<"pub radar"<<endl;

    while(true){
		if(!end_flag_ || radar_seq_ > (radar_frame_num_ - 1)){
			break;
		}

		if(stop_flag_){
            usleep(1e4);
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
    radar_end_ = true;
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
		for(int i=0; i<64; ++i){
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
        LOG(ERROR) << "Cannot open file "<< file_name
                    <<"! Aborting..." << '\n';
		return false;
	}
	return true;
}

void OfflinePub::PubImuData(){
    cout<<"pub imu"<<endl;

    while(true){
		if(!end_flag_ || imu_seq_ > (imu_frame_num_ - 1)){
			break;
		}
		
		if(stop_flag_){
            usleep(1e4);
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


		imu_message_.bversion	= 1;
		imu_message_.mversion  =  0;
		imu_message_.lversion 	= 2;
		imu_message_.ver_year 	= 22;
		imu_message_.ver_month  = 5;
		imu_message_.ver_day  	= 3;

		//send_imu_msg_.re_data(&imu_message_.wc_x, sizeof(imu_message_) - sizeof(imu_message_.header));
		send_imu_msg_.re_size(sizeof(imu_message_) - sizeof(imu_message_.header));
	   	memcpy(send_imu_msg_.data(), &imu_message_.wc_x, sizeof(imu_message_) - sizeof(imu_message_.header));

		int64_t now = GetTime();
		int64_t translate = time_set.TranslateTime(imu_timestamps_[imu_seq_]);
		std::this_thread::sleep_for(std::chrono::microseconds(translate - now));
		
		if(imu_flag_){
			int num_send = pub_imu_.send_pack(&imu_message_.header, sizeof(imu_message_.header), &send_imu_msg_);
        	printf("[INFO] Pub IMU Data \n");
		}

		heartbeat_header_.message_type = cngicuvc::messages::MessageType::XWGI_HEART_BEAT;
		int64_t htime = GetTime();
		heartbeat_header_.time_stamp = htime * 1e-3;
		cout<<"heartbeat_header_.time_stamp "<<heartbeat_header_.time_stamp<<endl;
		heartbeat_header_.seq++;
		imu_hearbeat_msg_.re_data((void *)&heartbeat_data_, sizeof(heartbeat_data_));
		pub_imu_heartbeat_.send_pack((void *)&heartbeat_header_, sizeof(heartbeat_header_), &imu_hearbeat_msg_);
        dnet_msleep(1);
		++imu_seq_;
	}
    ins_end_ = true;
	return;
}

void OfflinePub::PubCameraData(){
    while(true){
        if(!end_flag_ || camera_seq_ > (camera_frame_num_ - 1)){
            break;
        }

        if(stop_flag_){
            usleep(1e4);
            continue;
        }

        int64_t t0 = GetTime();
        string file_path = data_path_ + "/camera/data/" + camera_filename_[camera_seq_];
        pcl::PointCloud<PointRs>::Ptr cloud(new pcl::PointCloud<PointRs>);

        cv::Mat img_read = cv::imread(file_path);
        printf("[INFO] Camera file path: %s \n", file_path.c_str());

        camera_message_.header.message_type = cngicuvc::messages::MessageType::IMAGE_DATA_RAW_TW;
        camera_message_.header.seq++;
        camera_message_.header.time_stamp = ParseTime(camera_timestamp_[camera_seq_]) *1e-3;
        camera_message_.channels = img_read.channels();
        camera_message_.rows = img_read.rows;
        camera_message_.image_vec = img_read.reshape(1, 1);

        int size_id = sizeof(camera_message_.channels) + sizeof(camera_message_.rows);
        int data_id = camera_message_.image_vec.size() * sizeof(u_char);
        send_camera_msg_.re_size(size_id + data_id);

        memcpy((u_char *)send_camera_msg_.data(), &camera_message_.channels, size_id);
        memcpy((u_char *)send_camera_msg_.data() + size_id, &camera_message_.image_vec[0], data_id);

        int64_t now = GetTime();
        int64_t translate = time_set.TranslateTime(camera_timestamps_[camera_seq_]);
        std::this_thread::sleep_for(std::chrono::microseconds(translate - now));

        if(camera_flag_){
            int num_send = pub_camera_.send_pack((void *)&camera_message_.header, sizeof(camera_message_.header), &send_camera_msg_);
            printf("[INFO] Pub Camera Data! msg size: %d \n", cloud->points.size());
        }

        ++camera_seq_;

        int64_t t1 = GetTime();
        string currentt = toDateTime(t1*1e3);
        printf("[INFO] camera time : %s \n", camera_timestamp_[(camera_seq_-1)].c_str());
        printf("[INFO] camera pub time : %s \n", currentt.c_str());
    }
    camera_end_ = true;
    return;
}

void OfflinePub::Start(){
    end_flag_ = true;
    cout<<"[INFO] Start"<<endl;
    LOG(INFO) << "Start play the data!"
              << '\n';
	bool start_flag = false;
    int64_t now = GetTime();
    time_set.SetStartTime(now);
    time_set.SetRealStartTime(min_time_);
    if(radar_file_){
        std::thread pub_radar(&OfflinePub::PubRadarData, this);
        pub_radar.detach();
    }
    if(ins_file_){
        std::thread pub_imu(&OfflinePub::PubImuData, this);
        pub_imu.detach();
    }
    if(lidar_file_){
        std::thread pub_lidar(&OfflinePub::PubLidarData, this);
        pub_lidar.detach();
    }

    if(camera_file_){
        std::thread pub_camera(&OfflinePub::PubCameraData, this);
        pub_camera.detach();
    }

    std::thread loop_th(&OfflinePub::Loop, this);
    loop_th.detach();
    start_flag = true;
    cout<<"endstart"<<endl;
}

void OfflinePub::End(){
    LOG(INFO) << "End play current data!" << '\n';
    cout<<"End play current data!"<<endl;
    Reset();
    end_flag_ = false;
}

void OfflinePub::Stop(){
    stop_flag_ = !stop_flag_;
    cout<<"stop_flag_ "<<stop_flag_<<endl;
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

void OfflinePub::Loop(){
    cout<<"loop"<<endl;
    if(!radar_file_ && !ins_file_ && !lidar_file_ && !camera_file_){
        return;
    }
    while(true && end_flag_){
        std::unique_lock<std::mutex> seqlock(seq_mutex_);
        if(lidar_end_ && radar_end_ && ins_end_ && camera_end_){
            cout<<"Restart"<<endl;
            LOG(INFO) << "Restart play the data!" << '\n';
            int64_t now = GetTime();
            time_set.SetStartTime(now);
            lidar_seq_ = 0;
            radar_seq_ = 0;
            imu_seq_ = 0;
            camera_seq_ = 0;
            time_set.SetRealStartTime(min_time_);
            if(radar_file_){
                std::thread pub_radar(&OfflinePub::PubRadarData, this);
                pub_radar.detach();
                radar_end_ = false;
            }
            if(ins_file_){
                std::thread pub_imu(&OfflinePub::PubImuData, this);
                pub_imu.detach();
                ins_end_ = false;
            }
            if(lidar_file_){
                std::thread pub_lidar(&OfflinePub::PubLidarData, this);
                pub_lidar.detach();
                lidar_end_ = false;
            }
            if(camera_file_){
                std::thread pub_camera(&OfflinePub::PubCameraData, this);
                pub_camera.detach();
                camera_end_ = false;
            }
        }
        seqlock.unlock();
        usleep(1e5);
    }
    return;
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
    //	cout<<"time_in "<<time_in<<endl;
	time_in += min_time_;
	lidar_seq_ = BinarySearch(lidar_timestamps_, time_in);
	radar_seq_ = BinarySearch(radar_timestamps_, time_in);
	imu_seq_ = BinarySearch(imu_timestamps_, time_in);
    //	cout<<"lidar "<<" "<<lidar_seq_<<" "<<lidar_frame_num_<<endl;
    //	cout<<"radar "<<" "<<radar_seq_<<" "<<radar_frame_num_<<endl;
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
    cout<<"real_start_ "<<real_start_<<endl;
}

void TimeSetting::SetStartTime(int64_t time_in){
	translate_start_ = time_in;
    cout<<"translate_start_ "<<translate_start_<<endl;

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
