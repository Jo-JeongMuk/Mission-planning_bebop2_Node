#include "ros/ros.h"
#include "PointToPoint_List.h"
#include <cmath>
#include <cstdlib>

#define LOCA_PUB    "bebop/states/ardrone3/PilotingState/PositionChanged"
#define ALTI_PUB    "bebop/states/ardrone3/PilotingState/AltitudeChanged"
#define ATTI_PUB    "bebop/states/ardrone3/PilotingState/AttitudeChanged"
#define PI          3.14159265
#define LIMITE_DIST 1.5

PointToPoint::PointToPoint(int argc, char** argv)
: init_argc(argc), init_argv(argv), homeOrNot(true)
{

}

void PointToPoint::init()
{
    ros::init(init_argc, init_argv, "PointToPoint_List");
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
    if (this->homeOrNot == true)
    {
        this->h_lati    = msg->latitude;
        this->h_long    = msg->longitude;
        this->homeOrNot = false;
    }

    this->c_lati = msg->latitude;
    this->c_long = msg->longitude;
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

void PointToPoint::Move(List* list) {

    List* Current = list;
    while(Current != NULL)
    {
        double g_lati = Current->Latitude;
        double g_long = Current->Longitude;
        double g_alti = Current->Altitude;
        double speed  = Current->Speed;
        while(true)
        {
            try {
                ros::spinOnce();
            } catch (...) {
                ROS_ERROR("--- ERROR IN spin(), shutting down! ---");
                ros::shutdown();
            }

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
            double c_bearing    = (this->c_yaw * 180 / PI) + 60;

//            if (sin(g_long_radian - c_long_radian) < 0) true_bearing = true_bearing + 180;

            if (c_bearing <  0  ) c_bearing = 360 + c_bearing;
            if (c_bearing >= 360) c_bearing = 360 - c_bearing;

            if (this->c_alti < g_alti - 0.3)
                linXYZ_AngZ[2]  = 0.2;
            else if (this->c_alti > g_alti + 0.3)
                linXYZ_AngZ[2]  = -0.2;
            else linXYZ_AngZ[2] = 0;

            if (true_bearing < c_bearing + 10 && dist > LIMITE_DIST)
                linXYZ_AngZ[3]  = 0.15;
            else if (true_bearing > c_bearing - 10 && dist > LIMITE_DIST)
                linXYZ_AngZ[3]  = -0.15;
            else linXYZ_AngZ[3] = 0;

            if (true_bearing < c_bearing + 10 && true_bearing > c_bearing - 10 && dist > LIMITE_DIST)
                linXYZ_AngZ[0]  = 0.5 * speed;
            else linXYZ_AngZ[0] = 0;

            this->twist.linear.x  = linXYZ_AngZ[0];
            this->twist.linear.y  = linXYZ_AngZ[1];
            this->twist.linear.z  = linXYZ_AngZ[2];
            this->twist.angular.x = 0;
            this->twist.angular.y = 0;
            this->twist.angular.z = linXYZ_AngZ[3];
            this->ptop_pub.publish(twist);

            ROS_INFO("goal : %.2lf , Current : %.2lf, dist : %.2lf", true_bearing, c_bearing, dist);
            if (dist < LIMITE_DIST && Current->next != NULL)
            {
                Current = Current->next;
                break;
            }
            else if (dist < LIMITE_DIST && Current->next == NULL &&  g_lati != this->h_lati && g_long != this->h_long)
            {
                g_lati = this->h_lati;
                g_long = this->h_long;
            }
        }
    }
}

int main(int argc, char** argv){

    List* list;//
    list = new List;//
    list->Latitude = 36.519547;// 운동장 서남쪽
    list->Longitude = 127.172806;//
    list->Altitude = 2.0;//
    list->Speed = 0.2;//
    list->next = new List;//
    list->next->Latitude = 36.519686;// 운동장 동남쪽
    list->next->Longitude = 127.173387;//
    list->next->Altitude = 2.0;//
    list->next->Speed = 0.2;//
    list->next->next = NULL;//

    PointToPoint a(argc, argv);
    a.init();
    a.Move(list);

    while(list)//
    {//
        List* tmp = list;//
        list = list->next;//
        delete tmp;//
    }//
    return 0;
}
