#include "ros/ros.h"
#include "std_msgs/String.h"
#include "SendState.h"
#include <cmath>

#define BATT_PUB "bebop/states/common/CommonState/BatteryStateChanged"
#define SATL_PUB "bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged"
#define ALTI_PUB "bebop/states/ardrone3/PilotingState/AltitudeChanged"
#define LOCA_PUB "bebop/states/ardrone3/PilotingState/PositionChanged"
#define SPEE_PUB "bebop/states/ardrone3/PilotingState/SpeedChanged"

#define PI 3.14159265

SendStatue::SendStatue(int argc, char** argv)
: init_argc(argc), init_argv(argv)
{

}

void SendStatue::init()
{
    ros::init(init_argc, init_argv, "Batt_Satl_Alti");
    ros::NodeHandle statuenh;

	batt_remain = statuenh.subscribe
		(BATT_PUB, 1, &SendStatue::batteryCallback, this);

    num_of_satl = statuenh.subscribe
		(SATL_PUB, 1, &SendStatue::satelliteCallback, this);

 	altitude = statuenh.subscribe
		(ALTI_PUB, 1, &SendStatue::altitudeCallback, this);

	location = statuenh.subscribe
		(LOCA_PUB, 1, &SendStatue::locationCallback, this);

	speed = statuenh.subscribe
		(SPEE_PUB, 1, &SendStatue::speedCallback, this);
}

void SendStatue::batteryCallback(const BATTERY)
{
	int BatteryRemained = msg->percent;
//    Q_EMIT Batt_sendstatue(BatteryRemained);
	ROS_INFO("battery = %d percent", BatteryRemained);
}

void SendStatue::satelliteCallback(const SATELLITE)
{
	int NumberOfSatellites = msg->numberOfSatellite;
//    Q_EMIT Satl_sendstatue(NumberOfSatellites);
	ROS_INFO("satellite = %d", NumberOfSatellites);
}

void SendStatue::altitudeCallback(const ALTITUDE)
{
	float altitude = msg->altitude;
//	Q_EMIT Alti_sendstatue(altitude);
	ROS_INFO("altitude = %f", altitude);
}

void SendStatue::locationCallback(const LOCATION)
{
    double latitude, longitude;
    latitude = msg->latitude;
    longitude = msg->longitude;
//	Q_EMIT sendlocation(latitude, longitude);
	ROS_INFO("latituude = %lf, longitude = %lf", latitude, longitude);
}

void SendStatue::speedCallback(const SPEED)
{
	float speed = msg->speedX;
//	Q_EMIT Spee_sendstatue(speed);
	ROS_INFO("Speed = %f m/s", speed);
}

double SendStatue::distance(const double c_lati, const double c_long, const double g_lati, const double g_long)
{
	double dist = (((acos(sin(c_lati * PI / 180.0) * sin(g_lati * PI / 180.0) +
				   cos(c_lati * PI / 180.0) * cos(g_lati * PI / 180.0) *
				   cos((c_long - g_long) * PI / 180.0))) * 180 / PI) * 60 * 1.1515) * 1609.344;
//	Q_EMIT Dist_sendstatue(dist);
	return dist;
}

int main(int argc, char** argv){

	SendStatue a(argc, argv);
	a.init();
//ex	double dist = a.distance(36.5204184, 127.1730486, 36.5203376, 127.1731411);
//ex	ROS_INFO("distance = %lf m", dist);
	ros::spin();
	return 0;
}

