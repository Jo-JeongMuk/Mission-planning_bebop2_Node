#include "ros/ros.h"
#include "Takeoff_Land.h"

#define LOCA_PUB "bebop/states/ardrone3/PilotingState/PositionChanged"

Takeoff_Land::Takeoff_Land(int argc, char** argv)
: init_argc(argc), init_argv(argv), order(false)
{

}

void Takeoff_Land::init()
{
    ros::init(init_argc, init_argv, "Takeoff_Land");
    ros::NodeHandle takeoffnh;
    takeoff = takeoffnh.advertise<std_msgs::Empty>("bebop/takeoff", 1);
    land = takeoffnh.advertise<std_msgs::Empty>("bebop/land", 1);
    location = takeoffnh.subscribe 
		(LOCA_PUB, 1, &Takeoff_Land::TakeoffOrder, this);
}

void Takeoff_Land::TakeoffOrder(const LOCATION)
{
    double latitude, longitude;

    if (this->order == true) {
        takeoff.publish(this->empty_msg);
        latitude = msg->latitude;
        longitude = msg->longitude;
//    	Q_EMIT sendhomepoint(latitude, longitude);
        ROS_INFO("latituude = %lf, longitude = %lf", latitude, longitude);
    }
    else
        land.publish(this->empty_msg);
}

int main(int argc, char** argv)
{
	Takeoff_Land a(argc, argv);
	a.init();
	a.order = false;
	return 0;
}
