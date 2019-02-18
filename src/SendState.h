#ifndef BATT_SATL_ALTI
#define BATT_SATL_ALTI

#include "bebop_msgs/CommonCommonStateBatteryStateChanged.h"
#include "bebop_msgs/Ardrone3GPSStateNumberOfSatelliteChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h"
#include "bebop_msgs/Ardrone3PilotingStatePositionChanged.h"
#include "bebop_msgs/Ardrone3PilotingStateSpeedChanged.h"

#define BATTERY   bebop_msgs::CommonCommonStateBatteryStateChangedConstPtr& msg
#define SATELLITE bebop_msgs::Ardrone3GPSStateNumberOfSatelliteChangedConstPtr& msg
#define ALTITUDE  bebop_msgs::Ardrone3PilotingStateAltitudeChangedConstPtr& msg
#define LOCATION  bebop_msgs::Ardrone3PilotingStatePositionChangedConstPtr& msg
#define SPEED     bebop_msgs::Ardrone3PilotingStateSpeedChangedConstPtr& msg

class SendStatue {
private:
    int init_argc;
    char** init_argv;
    ros::Subscriber num_of_satl;
    ros::Subscriber batt_remain;
    ros::Subscriber altitude;
    ros::Subscriber location;
    ros::Subscriber speed;

public:
    SendStatue(int argc, char** argv);
    ~SendStatue() {}
    void init();
    void batteryCallback(const BATTERY);
    void satelliteCallback(const SATELLITE);
    void altitudeCallback(const ALTITUDE);
    void locationCallback(const LOCATION);
    void speedCallback(const SPEED);
    double distance(const double c_lati, const double c_long, const double g_lati, const double g_long);
//
//Q_SIGNALS:
//    void Batt_sendstatue(int BatteryRemained);
//    void Satl_sendstatue(int NumberOfSatellites);
//    void Alti_sendstatue(float altitude);
//    void sendlocation(double latitude, double longitude);
//    void Spee_sendstatue(float speed);
//    void Dist_sendstatue(double dist);
//
};
#endif