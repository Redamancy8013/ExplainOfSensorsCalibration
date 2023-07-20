#include "offline_playback.h"


int main(int argc, char * argv[]){
	OfflinePub pub_module_;
	string file_path = "/home/wx/Documents/work201/20220223/ModuleTest/perception/bin/dataset/2022-03-31 17:15:26";
	pub_module_.InputFilePath(file_path);
	pub_module_.PreProcess();
	// pub_module_.FindStartSeq();
	pub_module_.Start();
	// pub_module_.Stop();
 	pub_module_.lidar_flag_ = true;
 	pub_module_.radar_flag_ = true;
 	pub_module_.imu_flag_ = true;
	while(1){
	}
	return 0;
}
