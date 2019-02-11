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
#include <list>

typedef struct _List{
    double Latitude;
    double Longitude;
    double Altitude;
    double Speed;
} List;

class PointToPoint_List {
public:
    int     init_argc;
    char**  init_argv;
    double  h_lati, h_long;
    bool    homeOrNot, pause;
    double  c_lati, c_long, c_alti, c_yaw;

    ros::Subscriber      location;
    ros::Subscriber      altitude;
    ros::Subscriber      attitude;
    ros::Publisher       ptop_pub;
    ros::Publisher       land;
    ros::Publisher       takeoff;
    std_msgs::Empty      empty_msg;
    geometry_msgs::Twist twist;

    PointToPoint_List(int argc, char** argv);
    ~PointToPoint_List() {}

    void init();
    void altitudeCallback(const ALTITUDE);
    void attitudeCallback(const ATTITUDE);
    void locationCallback(const LOCATION);
    void Pause();
    void Stop();
    void Ros_SpinOnce();
    void P2P(double g_lati, double g_long, double g_alti, double speed);
    double TrueBearing(double g_lati, double g_long);
    void Move(std::list<List>* _list);
};

#endif