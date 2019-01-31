#include "ros/ros.h"
#include "PointToPoint.h"
#include <cmath>
#include <cstdlib>

#define LOCA_PUB "bebop/states/ardrone3/PilotingState/PositionChanged"
#define ALTI_PUB "bebop/states/ardrone3/PilotingState/AltitudeChanged"
#define ATTI_PUB "bebop/states/ardrone3/PilotingState/AttitudeChanged"
#define PI 3.14159265
#define LIMITE_DIST 1.5

PointToPoint::PointToPoint(int argc, char** argv)
: init_argc(argc), init_argv(argv)
{

}

void PointToPoint::init()
{
    ros::init(init_argc, init_argv, "PointToPoint");
    ros::NodeHandle ptopnh;
    ptop_pub = ptopnh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
    location = ptopnh.subscribe
            (LOCA_PUB, 1, &PointToPoint::locationCallback, this);
    altitude = ptopnh.subscribe
            (ALTI_PUB, 1, &PointToPoint::altitudeCallback, this);
    attitude = ptopnh.subscribe
            (ATTI_PUB, 1, &PointToPoint::attitudeCallback, this);
}

void PointToPoint::locationCallback(const LOCATION)
{
    this->c_lati = 36.5199273333;//msg->latitude;
    this->c_long = 127.172784333;//msg->longitude;
//    ROS_INFO("%lf , %lf\n", this->c_lati, this->c_long);
}

void PointToPoint::altitudeCallback(const ALTITUDE)
{
    this->c_alti = msg->altitude;
//    ROS_INFO("hight : %lf\n", this->c_alti);
}

void PointToPoint::attitudeCallback(const ATTITUDE)
{
    this->c_yaw = msg->yaw;
//    ROS_INFO("yaw : %lf ", this->c_yaw);
}

bool PointToPoint::Move(const double g_lati, const double g_long, const double g_alti, const double speed)
{
    float linXYZ_AngZ[4] = {0, 0, 0, 0};
    double c_lati_radian = this->c_lati * (PI / 180);
    double c_long_radian = this->c_long * (PI / 180);
    double g_lati_radian = g_lati * (PI / 180);
    double g_long_radian = g_long * (PI / 180);

    double dist = (((acos(sin(c_lati * PI / 180.0) * sin(g_lati * PI / 180.0) +
            cos(c_lati * PI / 180.0) * cos(g_lati * PI / 180.0) * cos((c_long - g_long) *
            PI / 180.0))) * 180.0 / PI) * 60.0 * 1.1515) * 1609.344;

    double dist_rad = acos(sin(c_lati_radian) * sin(g_lati_radian) + cos(c_lati_radian)
            * cos(g_lati_radian) * cos(c_long_radian - g_long_radian));

    double radian = acos((sin(g_lati_radian) - sin(c_lati_radian) * cos(dist_rad))
            / (cos(c_lati_radian) * sin(dist_rad)));

    double true_bearing = radian * (180 / PI);
    double c_bearing = (this->c_yaw * 180 / PI) + 60;

    //if(sin(g_long_radian - c_long_radian) < 0) c_bearing += 180;

    if (c_bearing < 0) c_bearing += 360;
    if (c_bearing >= 360) c_bearing = 360 - c_bearing;

    if (this->c_alti < g_alti - 0.3)
        linXYZ_AngZ[2] = 0.2;
    else if (this->c_alti > g_alti + 0.3)
        linXYZ_AngZ[2] = - 0.2;
    else linXYZ_AngZ[2] = 0;

    if (true_bearing < c_bearing + 10 && dist > LIMITE_DIST)
    {linXYZ_AngZ[3] = 0.15;}
    else if (true_bearing > c_bearing - 10 && dist > LIMITE_DIST)
    {linXYZ_AngZ[3] = -0.15;}
    else {linXYZ_AngZ[3] = 0;}

    if (true_bearing < c_bearing + 10 && true_bearing > c_bearing - 10 && dist > LIMITE_DIST)
        linXYZ_AngZ[0] = 1.0 * speed;
    else linXYZ_AngZ[0] = 0.01;

    this->twist.linear.x = linXYZ_AngZ[0];
    this->twist.linear.y = linXYZ_AngZ[1];
    this->twist.linear.z = linXYZ_AngZ[2];
    this->twist.angular.x = 0;
    this->twist.angular.y = 0;
    this->twist.angular.z = linXYZ_AngZ[3];
    this->ptop_pub.publish(twist);

    ROS_INFO("goal : %.2lf , Current : %.2lf, dist : %.2lf", true_bearing, c_bearing, dist);

    return (dist < LIMITE_DIST) ? true : false;
}

int main(int argc, char** argv){
    PointToPoint a(argc, argv);
    a.init();
    while(true) {
//        if(a.Move(36.520319, 127.174252, 2.0, 0.1) == true) break; // e
//        if(a.Move(36.520261, 127.170583, 2.0, 0.1) == true) break; // w
//        if(a.Move(36.518981, 127.172672, 2.0, 0.1) == true) break; // s
//        if(a.Move(36.521791, 127.172483, 2.0, 0.1) == true) break; // n
//        if(a.Move(36.5197055, 127.1733759, 2.0, 0.1) == true) break; // 운동장 동남쪽(1.1318)
        if(a.Move(36.5196345, 127.172773667, 2.0, 0.1) == true) break; // 운동장 서남쪽(2.0202)
        try {
            ros::spinOnce();
        } catch (...) {
            ROS_ERROR("--- ERROR IN spin(), shutting down! ---");
            ros::shutdown();
        }
    }
    return 0;
}