#ifndef TAKEOFF_LAND
#define TAKEOFF_LAND

#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "bebop_msgs/Ardrone3PilotingStatePositionChanged.h"

#define LOCATION bebop_msgs::Ardrone3PilotingStatePositionChangedConstPtr& msg

class Takeoff_Land {
public:
    int init_argc;
    char** init_argv;
	bool order;
	ros::Subscriber location;
	ros::Publisher land;
	ros::Publisher takeoff;
    std_msgs::Empty empty_msg;

    Takeoff_Land(int argc, char** argv);
    ~Takeoff_Land() {}
    void init();
    void TakeoffOrder(const LOCATION);

//Q_SIGNALS:
//    void sendhomepoint(double latitude, double longitude)
};

#endif
