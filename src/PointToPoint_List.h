#ifndef POINTTOPOINT
#define POINTTOPOINT
#define LOCATION bebop_msgs::Ardrone3PilotingStatePositionChangedConstPtr& msg
#define ALTITUDE bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& msg
#define ATTITUDE bebop_msgs::Ardrone3PilotingStateAttitudeChangedConstPtr& msg

#include "bebop_msgs/Ardrone3PilotingStatePositionChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"
#include "std_msgs/Empty.h"
#include <geometry_msgs/Twist.h>

typedef struct _List{
    double Latitude;
    double Longitude;
    double Altitude;
    double Speed;
//    struct _List *next;
} List;

class PointToPoint {
public:
    int     init_argc;
    char**  init_argv;
    double  h_lati, h_long;
    double  c_lati, c_long, c_alti, c_yaw;
    bool    homeOrNot, pause;

    ros::Publisher ptop_pub;
    ros::Subscriber location;
    ros::Subscriber altitude;
    ros::Subscriber attitude;
    ros::Publisher land;
    ros::Publisher takeoff;
    std_msgs::Empty empty_msg;
    geometry_msgs::Twist twist;

    PointToPoint(int argc, char** argv);
    ~PointToPoint() {}

    void init();
    void locationCallback(const LOCATION);
    void altitudeCallback(const ALTITUDE);
    void attitudeCallback(const ATTITUDE);
    void Move(List *list);
    void Pause();
    void Stop();
};

#endif