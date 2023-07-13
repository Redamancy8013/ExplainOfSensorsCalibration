/** 
 * \brief A module that send the control command to  other module.
 * \author ZBSu
 * 
 */
#include <send/send_module.h>
#include<cstring>

namespace cngicuvc
{
SendModule::SendModule()	
{
	onInit();
}

SendModule::~SendModule()
{
	
}

void SendModule::onInit()
{
	mPublisher.sock_init(DNET_PUB, 100);
	mPublisher.sock_addr("ipc://ADDR_PLATFORM_STEERING_IPC", true);
	mTimes = 0;
}

void SendModule::task0_loop()
{
	is_stopped[0] = false;
	while (!is_stopping[0])
	{
		taskSendPackage();
	}	
	is_stopped[0] = true;
}

void SendModule::taskSendPackage()
{
	mPlatformSteeringCommand.header.message_type = cngicuvc::messages::MessageType::PLATFORM_STEERING_COMMAND;	
	mPlatformSteeringCommand.header.seq++;
	mPlatformSteeringCommand.header.time_stamp = xtime_microseconds();	

	mPlatformSteeringCommand.trans_vel = 123;                   
	mPlatformSteeringCommand.curvature = 456;
	
	int size_head = (int)sizeof(mPlatformSteeringCommand.header);
	int size_data = (int)sizeof(mPlatformSteeringCommand.trans_vel) + (int)sizeof(mPlatformSteeringCommand.curvature);

	dnet_msg_t snd_msg(&mPlatformSteeringCommand.trans_vel, size_data);
	int num_send = mPublisher.send_pack(&mPlatformSteeringCommand.header, size_head, &snd_msg);
	
	printf("send time = %lld, speed = %f, cur = %f\n", mPlatformSteeringCommand.header.time_stamp, mPlatformSteeringCommand.trans_vel, mPlatformSteeringCommand.curvature); 
}

}//end namespace

