#ifndef P2P_DUE_NORTH
#define P2P_DUE_NORTH
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

class P2P_DueNorth {
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
    geometry_msgs::Twist twist;

    P2P_DueNorth(int argc, char** argv);
    ~P2P_DueNorth() {}

    void init();
    void locationCallback(const LOCATION);
    void altitudeCallback(const ALTITUDE);
    void attitudeCallback(const ATTITUDE);
    void Move(List *list);
    void Pause();
    void Stop();
};

#endif