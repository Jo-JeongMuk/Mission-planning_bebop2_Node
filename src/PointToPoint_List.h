#ifndef POINTTOPOINT
#define POINTTOPOINT
#define LOCATION bebop_msgs::Ardrone3PilotingStatePositionChangedConstPtr& msg
#define ALTITUDE bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& msg
#define ATTITUDE bebop_msgs::Ardrone3PilotingStateAttitudeChangedConstPtr& msg

#include "bebop_msgs/Ardrone3PilotingStatePositionChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"
#include <geometry_msgs/Twist.h>

typedef struct _List{//
    double Latitude;//
    double Longitude;//
    double Altitude;//
    double Speed;//
    struct _List *next;//
} List;//

class PointToPoint {
public:
    int init_argc;
    char** init_argv;
    double h_lati, h_long;
    bool homeOrNot;
    double c_lati, c_long, c_alti, c_yaw;

    ros::Publisher ptop_pub;
    ros::Subscriber location;
    ros::Subscriber altitude;
    ros::Subscriber attitude;
    geometry_msgs::Twist twist;

    PointToPoint(int argc, char** argv);
    ~PointToPoint() {}

    void init();
    void locationCallback(const LOCATION);
    void altitudeCallback(const ALTITUDE);
    void attitudeCallback(const ATTITUDE);
    void Move(List *list);
};

#endif