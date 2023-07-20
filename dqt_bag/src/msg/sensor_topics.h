#ifndef SENSOR_TOPPICS_H
#define SENSOR_TOPPICS_H

#include "ugv_topics.h"
// #include "radardriver.h"

#define  ADDR_RADARDATA_IPC "ipc:///tmp/ADDR_RADARDATA_IPC"
#define  ADDR_LIDARDATA_IPC "ipc:///tmp/ADDR_3DLIDARDATA_IPC"


struct PointXYZIR{
	float x = -1;
	float y = -1;
	float z = -1;
	float intensity = 0;
	uint16_t ring = 0;
};

struct RadarDataNode
{
    float range;
    float angle;    //angle 0~180
    float velocity;
    float accel;
    float lat_rate;
    float width;
    bool flag=0;
    int dnyprop;
    double localX;
    double localY;
    int px,py;
    int times;
    double lastX;
    double lastY;
    double globalX;
    double globalY;
    bool update;
    int idx;
};


//############# sensors ############################
namespace cngicuvc
{
	namespace messages
	{

		struct RadarData{
			Header header;
			RadarDataNode data[64];
		};


		struct PointClouds{
			Header header;
			int size = 0;
			PointXYZIR points[95000]; //for cfans or rslidar m1
		};
	}
}

#endif

