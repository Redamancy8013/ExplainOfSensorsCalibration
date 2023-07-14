#include <receive/receive_module.h>
#include <cstring>

#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

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

#define  ADDR_OBJECT_DETECT_CAMBRICON_IPC  "ipc:///tmp/ADDR_OBJECT_DETECT_CAMBRICON_IPC" //axy

cv::Mat img;
// cngicuvc::messages::ImageDataTW mImageData;


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
    subscriber.sock_addr((char *)ADDR_OBJECT_DETECT_CAMBRICON_IPC, false);
	
}

void ReceiveModule::task0_loop()
{
	is_stopped[0] = false;

	// 创建窗口
    cv::namedWindow("Image", cv::WINDOW_NORMAL);

	while (!is_stopping[0])
	{

		taskReceivePackage();

		// Mat img;
        // img=cvarrToMat(img_);
        //cam >> img;

        // image_cam_[i].push(dst);
        // this->pushImages(dst);

        //      image_frame_[i].push_back(frame);
        //      frame++;

	}	

	// 关闭窗口
    cv::destroyAllWindows();


	is_stopped[0] = true;
}

void ReceiveModule::taskReceivePackage()
{

	dnet_msg_t msg, mheader0;
	cngicuvc::messages::Header header;
	cngicuvc::messages::ImageDataTW mImageData;
	//timer0.start();
	int recv_size = subscriber.recv_pack(&header, sizeof(header), msg);
	cout << "recv_size:" << recv_size << endl;
	// cout<<"situational_environmentalFlag "<<situational_environmentalFlag<<" "<<recv_size<<endl;
	// if (recv_size > 0  && situational_environmentalFlag)
	if (recv_size > 0)
	{
		cv::destroyAllWindows();
		int size_id = sizeof(mImageData.channels) + sizeof(mImageData.rows);
		int data_id = msg.size() - size_id;

		std::cout << "recv bytes" << data_id << std::endl;
		
		mImageData.image_vec.clear();
		mImageData.image_vec.resize(data_id / sizeof(u_char));
		memcpy(&mImageData.channels, msg.data(), size_id);               //void *memcpy(void *str1, const void *str2, size_t n) 从存储区 str2 复制 n 个字符到存储区 str1。
		memcpy(&mImageData.image_vec[0], msg.data() + size_id, data_id); //void *memcpy(void *str1, const void *str2, size_t n) 从存储区 str2 复制 n 个字符到存储区 str1。

		cv::Mat mat = cv::Mat(mImageData.image_vec);
		//mtx.lock();

		cv::Mat dst = mat.reshape(mImageData.channels, mImageData.rows);
		cout << "channels: " << mImageData.channels << endl;
		cout << "rows: " << mImageData.rows << endl;

		cv::resize(dst, dst, cv::Size(640, 480));
		cv::imshow("recv windows", dst);
		cv::waitKey(2000);
	

	return;

	//mheader0.re_data(&header, sizeof(header));
	//std::cout << "recv_size=" << recv_size << std::endl;
	//else
	//{
	//std::cout << "recv_size=" << recv_size << std::endl;
	//return;
	//}

	//cout << "send time" << mImageData.header.time_stamp << endl;

	//switch (mImageData.header.message_type)
	//{
	//case cngicuvc::messages::MessageType::PERSON_TRACKING:

	//tempData.set_pack(&mheader0, &msg);

	//cout<<dst.rows<<" "<<dst.cols<<endl;
	//mtx.unlock();
	//imshow("windows", dst);
	//cv::waitKey(1);

	//break;
	//}
	//cout << "timer0.elapse_ms():" << timer0.elapse_ms() << endl;

	//cout << "recv time" << getCurrentTime() << endl;
	//cout << "delay time" << getCurrentTime() - mImageData.header.time_stamp << endl;
	//timer0.reset();
	//dnet_msleep(100);
	}
}

}//end namespace

