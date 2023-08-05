#include <receive/receive_module.h>
#include <cstring>
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include <thread>

//pangolin
#include <pangolin/pangolin.h>

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

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <unistd.h>
#include <netinet/in.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <stdarg.h>
#include <fcntl.h>
#include <sys/time.h>
#include <mutex>

#include "../common/extrinsic_param.hpp"
#include "../common/intrinsic_param.hpp"
#include "../common/projector_lidar.hpp"

#define  ADDR_OBJECT_DETECT_CAMBRICON_IPC  "ipc:///tmp/ADDR_OBJECT_DETECT_CAMBRICON_IPC"
#define  ADDR_LIDARDATA_IPC "ipc:///tmp/ADDR_3DLIDARDATA_IPC"

dnet_socket_t sub_lidar;
dnet_socket_t sub_camera;

double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
double cali_scale_fxfy_ = 1.005;
static Eigen::Matrix4d calibration_matrix_ = Eigen::Matrix4d::Identity();
static Eigen::Matrix4d orign_calibration_matrix_ = Eigen::Matrix4d::Identity();
static Eigen::Matrix3d intrinsic_matrix_ = Eigen::Matrix3d::Identity();
static Eigen::Matrix3d orign_intrinsic_matrix_ = Eigen::Matrix3d::Identity();
std::vector<float> distortions_;
std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Vector4d>> modification_list_(12);
bool display_mode_ = false;
bool filter_mode_ = false;

cv::Mat img_from_dnet;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_from_dnet(
      new pcl::PointCloud<pcl::PointXYZI>);
std::mutex img_mtx;
std::mutex pc_mtx;



bool kbhit() {
  termios term;
  tcgetattr(0, &term);
  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);
  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

void CalibrationInit(Eigen::Matrix4d json_param) {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  calibration_matrix_ = json_param;
  orign_calibration_matrix_ = json_param;
  modification_list_.resize(12);
  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
    modification_list_[i] = tmp;
  }
  std::cout << "=>Calibration scale Init!\n";
}

void CalibrationScaleChange() {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
    modification_list_[i] = tmp;
  }
  std::cout << "=>Calibration scale update done!\n";
}

void saveResult(const cv::Mat &calib_img, const int &frame_id) {
  std::string file_name = "calibration_" + std::to_string(frame_id) + ".txt";
  std::ofstream fCalib(file_name);
  if (!fCalib.is_open()) {
    std::cerr << "open file " << file_name << " failed." << std::endl;
    return;
  }
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "R:\n"
         << calibration_matrix_(0, 0) << " " << calibration_matrix_(0, 1) << " "
         << calibration_matrix_(0, 2) << "\n"
         << calibration_matrix_(1, 0) << " " << calibration_matrix_(1, 1) << " "
         << calibration_matrix_(1, 2) << "\n"
         << calibration_matrix_(2, 0) << " " << calibration_matrix_(2, 1) << " "
         << calibration_matrix_(2, 2) << std::endl;
  fCalib << "t: " << calibration_matrix_(0, 3) << " "
         << calibration_matrix_(1, 3) << " " << calibration_matrix_(2, 3)
         << std::endl;
  fCalib << "\nIntrinsic:" << std::endl;
  fCalib << intrinsic_matrix_(0, 0) << " " << intrinsic_matrix_(0, 1) << " "
         << intrinsic_matrix_(0, 2) << "\n"
         << intrinsic_matrix_(1, 0) << " " << intrinsic_matrix_(1, 1) << " "
         << intrinsic_matrix_(1, 2) << "\n"
         << intrinsic_matrix_(2, 0) << " " << intrinsic_matrix_(2, 1) << " "
         << intrinsic_matrix_(2, 2) << std::endl;

  fCalib << "************* json format *************" << std::endl;
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "[" << calibration_matrix_(0, 0) << "," << calibration_matrix_(0, 1)
         << "," << calibration_matrix_(0, 2) << "," << calibration_matrix_(0, 3)
         << "],"
         << "[" << calibration_matrix_(1, 0) << "," << calibration_matrix_(1, 1)
         << "," << calibration_matrix_(1, 2) << "," << calibration_matrix_(1, 3)
         << "],"
         << "[" << calibration_matrix_(2, 0) << "," << calibration_matrix_(2, 1)
         << "," << calibration_matrix_(2, 2) << "," << calibration_matrix_(2, 3)
         << "],"
         << "[" << calibration_matrix_(3, 0) << "," << calibration_matrix_(3, 1)
         << "," << calibration_matrix_(3, 2) << "," << calibration_matrix_(3, 3)
         << "]" << std::endl;
  fCalib << "\nIntrinsic:" << std::endl;
  fCalib << "[" << intrinsic_matrix_(0, 0) << "," << intrinsic_matrix_(0, 1)
         << "," << intrinsic_matrix_(0, 2) << "],"
         << "[" << intrinsic_matrix_(1, 0) << "," << intrinsic_matrix_(1, 1)
         << "," << intrinsic_matrix_(1, 2) << "],"
         << "[" << intrinsic_matrix_(2, 0) << "," << intrinsic_matrix_(2, 1)
         << "," << intrinsic_matrix_(2, 2) << "]" << std::endl;

  fCalib << "\nDistortion:" << std::endl;
  fCalib << "[";
  for (size_t i = 0; i < distortions_.size(); i++) {
    fCalib << distortions_[i];
    if (i == distortions_.size() - 1)
      continue;
    fCalib << ",";
  }
  fCalib << "]";
  fCalib.close();

  std::string img_name = "calibimg_" + std::to_string(frame_id) + ".jpg";
  cv::imwrite(img_name, calib_img);
}

bool ManualCalibration(int key_input) {
  char table[] = {'q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h'};
  bool real_hit = false;
  for (int32_t i = 0; i < 12; i++) {
    if (key_input == table[i]) {
      calibration_matrix_ = calibration_matrix_ * modification_list_[i];
      real_hit = true;
    }
  }
  // adjust fx, fy
  if (key_input == 'u') {
    intrinsic_matrix_(0, 0) *= cali_scale_fxfy_;
    std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
  }
  if (key_input == 'j') {
    intrinsic_matrix_(0, 0) /= cali_scale_fxfy_;
    std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
  }
  if (key_input == 'i') {
    intrinsic_matrix_(1, 1) *= cali_scale_fxfy_;
    std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
  }
  if (key_input == 'k') {
    intrinsic_matrix_(1, 1) /= cali_scale_fxfy_;
    std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
  }
  return real_hit;
}

void calibLidar2Camera(){
	// string camera_path = argv[1];
	// string lidar_path = argv[2];
	string intrinsic_json = "/home/easycool/project/config/center_camera-intrinsic.json";
	string extrinsic_json = "/home/easycool/project/config/top_center_lidar-to-center_camera-extrinsic.json";
	// cv::Mat img = cv::imread(camera_path);
	
	std::cout << intrinsic_json << std::endl;
	
	// if (pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_path, *cloud) == -1) {
	//   PCL_ERROR("Couldn't read file test_pcd.pcd \n");
	//   return (-1);
	// }
		std::cout << "\033[0;31m start\n\033[0m";

		std::cout << "\033[0;31m end\n\033[0m";
	// load intrinsic
	Eigen::Matrix3d K;
	std::vector<double> dist;
	LoadIntrinsic(intrinsic_json, K, dist);
	for (size_t i = 0; i < dist.size(); i++) {
		distortions_.push_back(dist[i]);
	}

	intrinsic_matrix_ = K;
	orign_intrinsic_matrix_ = intrinsic_matrix_;
	std::cout << "intrinsic:\n"
				<< K(0, 0) << " " << K(0, 1) << " " << K(0, 2) << "\n"
				<< K(1, 0) << " " << K(1, 1) << " " << K(1, 2) << "\n"
				<< K(2, 0) << " " << K(2, 1) << " " << K(2, 2) << "\n";
	std::cout << "dist:\n" << dist[0] << " " << dist[1] << "\n";

	// load extrinsic
	Eigen::Matrix4d json_param;
	LoadExtrinsic(extrinsic_json, json_param);

	cout << "Loading data completed!" << endl;
	CalibrationInit(json_param);

	std::cout << __LINE__ << "\n";
	std::cout << __LINE__ << "\n";

	while(cloud_from_dnet->empty()){}
	while(img_from_dnet.empty()){}
	
	// view
	int width = img_from_dnet.cols;
	int height = img_from_dnet.rows;
	std::cout << "width:" << width << " , height:" << height << std::endl;
	// const int width = 640, height = 480;
	pangolin::CreateWindowAndBind("lidar2camera player", width, height);
	glEnable(GL_DEPTH_TEST);
	// glDepthMask(GL_TRUE);
	// glDepthFunc(GL_LESS);

	pangolin::OpenGlRenderState s_cam(
		pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
		pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));

	pangolin::View &project_image =
		pangolin::Display("project")
			.SetBounds(0.0, 1.0, pangolin::Attach::Pix(150), 1.0,
						-1.0 * (width - 200) / height)
			.SetLock(pangolin::LockLeft, pangolin::LockTop);

	unsigned char *imageArray = new unsigned char[3 * width * height];
	pangolin::GlTexture imageTexture(width, height, GL_RGB, false, 0, GL_RGB,
									GL_UNSIGNED_BYTE);

	// control panel
	pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0,
											pangolin::Attach::Pix(150));
	pangolin::Var<bool> displayMode("cp.Intensity Color", false,
									true);                            // logscale
	pangolin::Var<bool> filterMode("cp.Overlap Filter", false, true); // logscale
	pangolin::Var<double> degreeStep("cp.deg step", 0.3, 0, 1);       // logscale
	pangolin::Var<double> tStep("cp.t step(cm)", 6, 0, 15);
	pangolin::Var<double> fxfyScale("cp.fxfy scale", 1.005, 1, 1.1);
	pangolin::Var<int> pointSize("cp.point size", 3, 1, 5);

	pangolin::Var<bool> addXdegree("cp.+ x degree", false, false);
	pangolin::Var<bool> minusXdegree("cp.- x degree", false, false);
	pangolin::Var<bool> addYdegree("cp.+ y degree", false, false);
	pangolin::Var<bool> minusYdegree("cp.- y degree", false, false);
	pangolin::Var<bool> addZdegree("cp.+ z degree", false, false);
	pangolin::Var<bool> minusZdegree("cp.- z degree", false, false);
	pangolin::Var<bool> addXtrans("cp.+ x trans", false, false);
	pangolin::Var<bool> minusXtrans("cp.- x trans", false, false);
	pangolin::Var<bool> addYtrans("cp.+ y trans", false, false);
	pangolin::Var<bool> minusYtrans("cp.- y trans", false, false);
	pangolin::Var<bool> addZtrans("cp.+ z trans", false, false);
	pangolin::Var<bool> minusZtrans("cp.- z trans", false, false);

	pangolin::Var<bool> addFx("cp.+ fx", false, false);
	pangolin::Var<bool> minusFx("cp.- fx", false, false);
	pangolin::Var<bool> addFy("cp.+ fy", false, false);
	pangolin::Var<bool> minusFy("cp.- fy", false, false);

	pangolin::Var<bool> resetButton("cp.Reset", false, false);
	pangolin::Var<bool> saveImg("cp.Save Image", false, false);

	std::vector<pangolin::Var<bool>> mat_calib_box;
	mat_calib_box.push_back(addXdegree);
	mat_calib_box.push_back(minusXdegree);
	mat_calib_box.push_back(addYdegree);
	mat_calib_box.push_back(minusYdegree);
	mat_calib_box.push_back(addZdegree);
	mat_calib_box.push_back(minusZdegree);
	mat_calib_box.push_back(addXtrans);
	mat_calib_box.push_back(minusXtrans);
	mat_calib_box.push_back(addYtrans);
	mat_calib_box.push_back(minusYtrans);
	mat_calib_box.push_back(addZtrans);
	mat_calib_box.push_back(minusZtrans);

	// cv::Mat current_frame;
	Projector projector;
	int frame_num = 0;
	std::cout << "\n=>START\n";
	while (!pangolin::ShouldQuit()) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		// {
		//   std::lock_guard<std::mutex> lg(img_mtx);
		//   img = img_from_ros;
		// }
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
		{
		// std::cout << "cloud assignment\n";
		std::lock_guard<std::mutex> lg(pc_mtx);
		cloud = cloud_from_dnet; 
		// std::cout << "cloud assignment finished\n";
		}    
		pcl::PointCloud<pcl::PointXYZI> pcd = *cloud;
		projector.loadPointCloud(pcd);
		
		cv::Mat img;
		{
		std::lock_guard<std::mutex> lg(img_mtx);
		img = img_from_dnet;
		}
		cv::Mat current_frame = projector.ProjectToRawImage(
		img, intrinsic_matrix_, dist, calibration_matrix_);

		if (displayMode) {
		if (display_mode_ == false) {
			projector.setDisplayMode(true);
			current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
														dist, calibration_matrix_);
			display_mode_ = true;
		}
		} else {
		if (display_mode_ == true) {
			projector.setDisplayMode(false);
			current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
														dist, calibration_matrix_);
			display_mode_ = false;
		}
		}

		if (filterMode) {
		if (filter_mode_ == false) {
			projector.setFilterMode(true);
			current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
														dist, calibration_matrix_);
			filter_mode_ = true;
		}
		} else {
		if (filter_mode_ == true) {
			projector.setFilterMode(false);
			current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
														dist, calibration_matrix_);
			filter_mode_ = false;
		}
		}

		if (degreeStep.GuiChanged()) {
		cali_scale_degree_ = degreeStep.Get();
		CalibrationScaleChange();
		std::cout << "Degree calib scale changed to " << cali_scale_degree_
					<< " degree\n";
		}
		if (tStep.GuiChanged()) {
		cali_scale_trans_ = tStep.Get() / 100.0;
		CalibrationScaleChange();
		std::cout << "Trans calib scale changed to " << cali_scale_trans_ * 100
					<< " cm\n";
		}
		if (fxfyScale.GuiChanged()) {
		cali_scale_fxfy_ = fxfyScale.Get();
		std::cout << "fxfy calib scale changed to " << cali_scale_fxfy_
					<< std::endl;
		}
		if (pointSize.GuiChanged()) {
		int ptsize = pointSize.Get();
		projector.setPointSize(ptsize);
		current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
													calibration_matrix_);
		std::cout << "point size changed to " << ptsize << std::endl;
		}
		for (int i = 0; i < 12; i++) {
		if (pangolin::Pushed(mat_calib_box[i])) {
			calibration_matrix_ = calibration_matrix_ * modification_list_[i];
			std::cout << "Changed!\n";
			current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_,
														dist, calibration_matrix_);
		}
		}

		if (pangolin::Pushed(addFx)) {
		intrinsic_matrix_(0, 0) *= cali_scale_fxfy_;
		current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
													calibration_matrix_);
		std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
		}
		if (pangolin::Pushed(minusFx)) {
		intrinsic_matrix_(0, 0) /= cali_scale_fxfy_;
		current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
													calibration_matrix_);
		std::cout << "fx changed to " << intrinsic_matrix_(0, 0) << std::endl;
		}
		if (pangolin::Pushed(addFy)) {
		intrinsic_matrix_(1, 1) *= cali_scale_fxfy_;
		current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
													calibration_matrix_);
		std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
		}
		if (pangolin::Pushed(minusFy)) {
		intrinsic_matrix_(1, 1) /= cali_scale_fxfy_;
		current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
													calibration_matrix_);
		std::cout << "fy changed to " << intrinsic_matrix_(1, 1) << std::endl;
		}

		if (pangolin::Pushed(resetButton)) {
		calibration_matrix_ = orign_calibration_matrix_;
		intrinsic_matrix_ = orign_intrinsic_matrix_;
		current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
													calibration_matrix_);
		std::cout << "Reset!\n";
		}
		if (pangolin::Pushed(saveImg)) {
		saveResult(current_frame, frame_num);
		std::cout << "\n==>Save Result " << frame_num << std::endl;
		Eigen::Matrix4d transform = calibration_matrix_;
		cout << "Transfromation Matrix:\n" << transform << std::endl;
		frame_num++;
		}

		if (kbhit()) {
		int c = getchar();
		if (ManualCalibration(c)) {
			Eigen::Matrix4d transform = calibration_matrix_;
			cout << "\nTransfromation Matrix:\n" << transform << std::endl;
		}
		current_frame = projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
													calibration_matrix_);
		}

		imageArray = current_frame.data;
		imageTexture.Upload(imageArray, GL_BGR, GL_UNSIGNED_BYTE);

		project_image.Activate();
		glColor3f(1.0, 1.0, 1.0);
		imageTexture.RenderToViewportFlipY();

		pangolin::FinishFrame();
		glFinish();
	}

	// delete[] imageArray;

	Eigen::Matrix4d transform = calibration_matrix_;
	cout << "\nFinal Transfromation Matrix:\n" << transform << std::endl;
}


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

    sub_camera.sock_init(DNET_SUB, 10);
    sub_camera.sock_addr((char *)ADDR_OBJECT_DETECT_CAMBRICON_IPC, false);

    sub_lidar.sock_init(DNET_SUB, 10);
    sub_lidar.sock_addr((char *)ADDR_LIDARDATA_IPC, false);
	
}

void ReceiveModule::task0_loop()
{
	is_stopped[0] = false;

  	std::thread calib_lidar_camera_thread(calibLidar2Camera);
	

	while (!is_stopping[0])
	{	
		taskReceivePackage();
	}	

	is_stopped[0] = true;
}

void ReceiveModule::taskReceivePackage()
{
	//camera parameters
	dnet_msg_t msg, mheader0;
	cngicuvc::messages::Header header;
	cngicuvc::messages::ImageDataTW mImageData;
	int recv_size_cam = sub_camera.recv_pack(&header, sizeof(header), msg);
	// cout << "recv_size_cam:" << recv_size_cam << endl;

	//lidar parameters
	dnet_msg_t recv_msg;
    cngicuvc::messages::PointClouds lidar_msg;
	int recv_size_lidar = sub_lidar.recv_pack(&lidar_msg.header, sizeof(lidar_msg.header), recv_msg);
	// cout << "recv_size_lidar:" << recv_size_lidar << endl;

	if(recv_size_lidar < 0 && recv_size_cam < 0)
		return;

	if (recv_size_cam > 0)
	{
		int size_id = sizeof(mImageData.channels) + sizeof(mImageData.rows);
		int data_id = msg.size() - size_id;

		mImageData.image_vec.clear();
		mImageData.image_vec.resize(data_id / sizeof(u_char));
		memcpy(&mImageData.channels, msg.data(), size_id);               //void *memcpy(void *str1, const void *str2, size_t n) 从存储区 str2 复制 n 个字符到存储区 str1。
		memcpy(&mImageData.image_vec[0], msg.data() + size_id, data_id); //void *memcpy(void *str1, const void *str2, size_t n) 从存储区 str2 复制 n 个字符到存储区 str1。

		cv::Mat mat = cv::Mat(mImageData.image_vec);

		img_from_dnet = mat.reshape(mImageData.channels, mImageData.rows);
		cout << "channels: " << mImageData.channels << endl;
		cout << "rows: " << mImageData.rows << endl;

	}


	if (recv_size_lidar > 0)
	{
		switch(lidar_msg.header.message_type){
			case cngicuvc::messages::MessageType::PointClouds:{

				memcpy(&lidar_msg.size, recv_msg.data(), sizeof(lidar_msg) - sizeof(lidar_msg.header));
				std::unique_lock<std::mutex> mlock(pc_mtx);
				cloud_from_dnet->points.resize(lidar_msg.size);
				for (size_t i = 0; i < lidar_msg.size; i++) {
					cloud_from_dnet->points[i].x = lidar_msg.points[i].x;
					cloud_from_dnet->points[i].y = lidar_msg.points[i].y;
					cloud_from_dnet->points[i].z = lidar_msg.points[i].z;
					cloud_from_dnet->points[i].intensity = lidar_msg.points[i].intensity;
				}
				mlock.unlock();
				break;
			}
			default:
				break;
		}

	}

}

}//end namespace

