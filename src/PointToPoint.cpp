#include "ros/ros.h"
#include "PointToPoint.h"
#include <cmath>
#include <cstdlib>

#define LOCA_PUB "bebop/states/ardrone3/PilotingState/PositionChanged"
#define ALTI_PUB "bebop/states/ardrone3/PilotingState/AltitudeChanged"
#define ATTI_PUB "bebop/states/ardrone3/PilotingState/AttitudeChanged"

#define PI          3.14159265
#define ANGLE       57.2957795786 // 180 / PI
#define RADIAN      0.0174532925  // PI / 180
#define METER       111189.57696  // 60.0 * 1.1515 * 1609.344;
#define LIMITE_DIST 2.5

PointToPoint::PointToPoint(int argc, char** argv)
: init_argc(argc), init_argv(argv){

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
    this->c_lati = msg->latitude;
    this->c_long = msg->longitude;
}

void PointToPoint::altitudeCallback(const ALTITUDE)
{
    this->c_alti = msg->altitude;
}

void PointToPoint::attitudeCallback(const ATTITUDE)
{
    this->c_yaw = msg->yaw;
}

void PointToPoint::Move(const double g_lati, const double g_long, const double g_alti, const double speed)
{
    double g_lati_radian = g_lati * RADIAN;
    double g_long_radian = g_long * RADIAN;

    while (true) {
        try {
            ros::spinOnce();
        } catch (...) {
            ROS_ERROR("--- ERROR IN spin(), shutting down! ---");
            ros::shutdown();
        }

        float linXYZ_AngZ[4] = {0, 0, 0, 0};

        double c_lati_radian = this->c_lati * RADIAN;
        double c_long_radian = this->c_long * RADIAN;

        double dist_rad = acos(sin(c_lati_radian) * sin(g_lati_radian) + cos(c_lati_radian)
                          * cos(g_lati_radian) * cos(c_long_radian - g_long_radian));

        double dist = dist_rad * ANGLE * METER - (speed * LIMITE_DIST);

        if (dist < LIMITE_DIST) break;

        double radian = acos((sin(g_lati_radian) - sin(c_lati_radian) * cos(dist_rad))
                        / (cos(c_lati_radian) * sin(dist_rad)));

        double c_bearing = this->c_yaw * ANGLE;
        if (c_bearing <= 0)   c_bearing = 360 - abs(c_bearing);
		if (c_bearing >= 355) c_bearing = 355;
		if (c_bearing <= 5)   c_bearing = 5;

        double true_bearing = radian * ANGLE;
        if (g_long < this->c_long) true_bearing = 360 - true_bearing;
		if (true_bearing >= 355)   true_bearing = 355;
		if (true_bearing <= 5)     true_bearing = 5;

        if (this->c_alti < g_alti - 0.3)
            linXYZ_AngZ[2] = 0.3;
        else if (this->c_alti > g_alti + 0.3)
            linXYZ_AngZ[2] = - 0.3;
        else
        {
            if (true_bearing < c_bearing + 15 && true_bearing > c_bearing - 15)
            {
                linXYZ_AngZ[0] = speed;
                if (true_bearing <= c_bearing + 15 && true_bearing >= c_bearing + 5)
                {
                    linXYZ_AngZ[1] =  - (speed / 3);
                    linXYZ_AngZ[3] =  - (speed / 2);
                }
                else if (true_bearing >= c_bearing - 15 && true_bearing <= c_bearing - 5)
                {
                    linXYZ_AngZ[1] = speed / 3;
                    linXYZ_AngZ[3] = speed / 2;
                }
            }
            else
            {
                if (true_bearing < c_bearing)
                    linXYZ_AngZ[3] = speed;
                else if (true_bearing > c_bearing)
                    linXYZ_AngZ[3] = - (speed);
            }
        }

        this->twist.linear.x  = linXYZ_AngZ[0];
        this->twist.linear.y  = linXYZ_AngZ[1];
        this->twist.linear.z  = linXYZ_AngZ[2];
        this->twist.angular.z = linXYZ_AngZ[3];
        this->ptop_pub.publish(twist);

        ROS_INFO("goal : %.2lf , Current : %.2lf, dist : %.2lf", true_bearing, c_bearing, dist);
    }
}

int main(int argc, char** argv){
    PointToPoint a(argc, argv);
    a.init();
// ex
    a.Move(36.519794, 127.172729, 2.0, 0.5);
	a.Move(36.519668, 127.173469, 2.0, 0.5);	
	a.Move(36.519634, 127.172790, 2.0, 0.5);
	a.Move(36.519886, 127.173316, 2.0, 0.5);
	a.Move(36.519756, 127.173248, 2.0, 0.5);
//    a.Move(36.519619, 127.172811, 2.0, 0.5);
//    a.Move(36.519668, 127.173425, 2.0, 0.5);

    return 0;
}
