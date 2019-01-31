#ifndef POINTTOPOINT
#define POINTTOPOINT
#define LOCATION bebop_msgs::Ardrone3PilotingStatePositionChangedConstPtr& msg
#define ALTITUDE bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& msg
#define ATTITUDE bebop_msgs::Ardrone3PilotingStateAttitudeChangedConstPtr& msg

#include "bebop_msgs/Ardrone3PilotingStatePositionChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"
#include <geometry_msgs/Twist.h>

class PointToPoint {
private:
    int init_argc;
    char** init_argv;
    double c_lati;
    double c_long;
    double c_alti;
    double c_yaw;
    ros::Publisher ptop_pub;
    ros::Subscriber location;
    ros::Subscriber altitude;
    ros::Subscriber attitude;
    geometry_msgs::Twist twist;

public:
    PointToPoint(int argc, char** argv);
    ~PointToPoint() {}
    void init();
    void locationCallback(const LOCATION);
    void altitudeCallback(const ALTITUDE);
    void attitudeCallback(const ATTITUDE);
    bool Move(const double g_lati, const double g_long, const double g_alti, const double speed);
};

#endif