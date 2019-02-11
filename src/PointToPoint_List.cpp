#include "ros/ros.h"
#include "PointToPoint_List.h"
#include <cmath>
#include <cstdlib>

#define LOCA_PUB "bebop/states/ardrone3/PilotingState/PositionChanged"
#define ALTI_PUB "bebop/states/ardrone3/PilotingState/AltitudeChanged"
#define ATTI_PUB "bebop/states/ardrone3/PilotingState/AttitudeChanged"
#define PI 3.14159265
#define LIMITE_DIST 3

using std::list;

PointToPoint_List::PointToPoint_List(int argc, char** argv)
: init_argc(argc), init_argv(argv), homeOrNot(true), pause(false), c_lati(0), c_long(0) {
}

void PointToPoint_List::init() {
    ros::init(init_argc, init_argv, "PointToPoint_List");
    ros::NodeHandle ptopnh;
    ptop_pub = ptopnh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
    takeoff = ptopnh.advertise<std_msgs::Empty>("bebop/takeoff", 1);
    land = ptopnh.advertise<std_msgs::Empty>("bebop/land", 1);
    altitude = ptopnh.subscribe
               (ALTI_PUB, 1, &PointToPoint_List::altitudeCallback, this);
    attitude = ptopnh.subscribe
               (ATTI_PUB, 1, &PointToPoint_List::attitudeCallback, this);
    location = ptopnh.subscribe
               (LOCA_PUB, 1, &PointToPoint_List::locationCallback, this);
}

void PointToPoint_List::altitudeCallback(const ALTITUDE) {
    this->c_alti = msg->altitude;
}

void PointToPoint_List::attitudeCallback(const ATTITUDE) {
    this->c_yaw = msg->yaw;
}

void PointToPoint_List::locationCallback(const LOCATION) {
    if (this->homeOrNot == true) {
        this->h_lati = msg->latitude;
        this->h_long = msg->longitude;
        this->homeOrNot = false;
        if (this->c_alti < 0.1)
            takeoff.publish(this->empty_msg);
    }

    this->c_lati = msg->latitude;
    this->c_long = msg->longitude;
}

void PointToPoint_List::Pause() {
    while (this->pause) {
        PointToPoint_List::Stop();
    }
}

void PointToPoint_List::Stop() {
    this->twist.linear.x = 0; this->twist.linear.y = 0; this->twist.linear.z = 0;
    this->twist.angular.x = 0; this->twist.angular.y = 0; this->twist.angular.z = 0;
    this->ptop_pub.publish(twist);
}

void PointToPoint_List::Ros_SpinOnce() {
    try {
        ros::spinOnce();
    } catch (...) {
        ROS_ERROR("--- ERROR IN spin(), shutting down! ---");
        ros::shutdown();
    }
}

void PointToPoint_List::P2P(double g_lati, double g_long, double g_alti, double speed) {
    while (true) {
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

        double dist = ((((acos(sin(c_lati * PI / 180.0) * sin(g_lati * PI / 180.0) +
                      cos(c_lati * PI / 180.0) * cos(g_lati * PI / 180.0) * cos((c_long - g_long) *
                      PI / 180.0))) * 180.0 / PI) * 60.0 * 1.1515) * 1609.344);

        double dist_rad = acos(sin(c_lati_radian) * sin(g_lati_radian) + cos(c_lati_radian)
                          * cos(g_lati_radian) * cos(c_long_radian - g_long_radian));

        double radian = acos((sin(g_lati_radian) - sin(c_lati_radian) * cos(dist_rad))
                        / (cos(c_lati_radian) * sin(dist_rad)));

        double c_bearing = this->c_yaw * (180 / PI);
        if (c_bearing < 0)
            c_bearing = 360 - abs(c_bearing);

        double true_bearing = (acos((sin(g_lati_radian) - sin(c_lati_radian) * cos(dist_rad))
                                    / (cos(c_lati_radian) * sin(dist_rad)))) * (180 / PI);
        if (g_long < this->c_long)
            true_bearing = 360 - true_bearing;

        if (dist < LIMITE_DIST) break;

        if (this->c_alti < g_alti - 0.3)
            linXYZ_AngZ[2] = 0.2;
        else if (this->c_alti > g_alti + 0.3)
            linXYZ_AngZ[2] = - 0.2;
        else {
            if (true_bearing < c_bearing + 5) {
                linXYZ_AngZ[1] = 0.2;
                linXYZ_AngZ[3] = 0.2;
            }
            else if (true_bearing > c_bearing - 5) {
                linXYZ_AngZ[1] = - 0.2;
                linXYZ_AngZ[3] = - 0.2;
            }
            if (true_bearing < c_bearing + 30 && true_bearing > c_bearing - 30)
                linXYZ_AngZ[0] = 0.5 * speed;
        }

        this->twist.linear.x = linXYZ_AngZ[0];
        this->twist.linear.y = linXYZ_AngZ[1];
        this->twist.linear.z = linXYZ_AngZ[2];
        this->twist.angular.z = linXYZ_AngZ[3];
        this->ptop_pub.publish(twist);

        ROS_INFO("goal : %.2lf , Current : %.2lf, dist : %.2lf", true_bearing, c_bearing, dist);
    }
}

void PointToPoint_List::Move(list<List>* _list) {
    list<List>::iterator iter;
    for(iter = _list->begin(); iter != _list->end(); ++iter) {
        PointToPoint_List::P2P(iter->Latitude, iter->Longitude, iter->Altitude, iter->Speed);
    }
    PointToPoint_List::P2P(this->h_lati, this->h_long, iter->Altitude, iter->Speed);
    PointToPoint_List::Stop();
    land.publish(this->empty_msg);

    return ;
}

int main(int argc, char** argv) {
// ex)
    list<List> _list;
    List* wp_1st;
    wp_1st = new List;
    wp_1st->Latitude = 36.519996;
    wp_1st->Longitude = 127.173408;
    wp_1st->Altitude = 2.0;
    wp_1st->Speed = 0.5;
    List* wp_2nd;
    wp_2nd = new List;
    wp_2nd->Latitude = 36.519752;
    wp_2nd->Longitude = 127.172782;
    wp_2nd->Altitude = 2.0;
    wp_2nd->Speed = 0.5;
    List* wp_3rd;
    wp_3rd = new List;
    wp_3rd->Latitude = 36.519714;
    wp_3rd->Longitude = 127.173431;
    wp_3rd->Altitude = 2.0;
    wp_3rd->Speed = 0.5;
    _list.push_back(*wp_1st);
    _list.push_back(*wp_2nd);
    _list.push_back(*wp_3rd);
//
    PointToPoint_List a(argc, argv);
    a.init();
    a.Move(&_list);

    delete wp_1st;
    delete wp_2nd;
    delete wp_3rd;

    return 0;
}
