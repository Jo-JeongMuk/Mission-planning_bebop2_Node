#include "ros/ros.h"
#include "P2P_DueNorth.h"
#include <cmath>
#include <cstdlib>

#define LOCA_PUB    "bebop/states/ardrone3/PilotingState/PositionChanged"
#define ALTI_PUB    "bebop/states/ardrone3/PilotingState/AltitudeChanged"
#define ATTI_PUB    "bebop/states/ardrone3/PilotingState/AttitudeChanged"
#define PI          3.14159265
#define LIMITE_DIST 2.5

P2P_DueNorth::P2P_DueNorth(int argc, char** argv)
: init_argc(argc), init_argv(argv), homeOrNot(true), pause(false) {
}

void P2P_DueNorth::init() {
    ros::init(init_argc, init_argv, "P2P_DueNorth");
    ros::NodeHandle ptopnh;
    ptop_pub = ptopnh.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1);
    location = ptopnh.subscribe
            (LOCA_PUB, 1, &P2P_DueNorth::locationCallback, this);
    altitude = ptopnh.subscribe
            (ALTI_PUB, 1, &P2P_DueNorth::altitudeCallback, this);
    attitude = ptopnh.subscribe
            (ATTI_PUB, 1, &P2P_DueNorth::attitudeCallback, this);
}

void P2P_DueNorth::locationCallback(const LOCATION) {
    if (this->homeOrNot == true) {
        this->h_lati    = msg->latitude;
        this->h_long    = msg->longitude;
        this->homeOrNot = false;
    }

    this->c_lati = msg->latitude;
    this->c_long = msg->longitude;
}

void P2P_DueNorth::altitudeCallback(const ALTITUDE) {
    this->c_alti = msg->altitude;
}

void P2P_DueNorth::attitudeCallback(const ATTITUDE) {
    this->c_yaw = msg->yaw;
}

void P2P_DueNorth::Pause() {
    while (this->pause) {
        P2P_DueNorth::Stop();
        sleep(500);
    }
}

void P2P_DueNorth::Stop() {
    this->twist.linear.x = 0;
    this->twist.linear.y = 0;
    this->twist.linear.z = 0;
    this->twist.angular.x = 0;
    this->twist.angular.y = 0;
    this->twist.angular.z = 0;
    this->ptop_pub.publish(twist);
}

void P2P_DueNorth::Move(List* list) {
    List* Current = list;
    while(Current != NULL) {
        double g_lati = Current->Latitude;
        double g_long = Current->Longitude;
        double g_alti = Current->Altitude;
        double speed  = Current->Speed;
        while(true) {
            try {
                ros::spinOnce();
            } catch (...) {
                ROS_ERROR("--- ERROR IN spin(), shutting down! ---");
                ros::shutdown();
            }
            double lati_ratio = this->c_lati - g_lati;
            double long_ratio = this->c_long - g_long;
            double temp = lati_ratio / long_ratio;
            long_ratio = long_ratio / lati_ratio;
            lati_ratio = temp;

            if (lati_ratio >= 1)
                lati_ratio = 1;
            else if(lati_ratio <= -1)
                lati_ratio = -1;
            else if ((lati_ratio > 0 && lati_ratio <= 0.01) || (lati_ratio < 0 && lati_ratio >= -0.01))
                lati_ratio = 0;

            if (long_ratio >= 1)
                long_ratio = 1;
            else if(long_ratio <= -1)
                long_ratio = -1;
            else if ((long_ratio > 0 && long_ratio <= 0.01) || (long_ratio < 0 && long_ratio >= -0.01))
                long_ratio = 0;

            if (this->pause == true)
                P2P_DueNorth::Pause();

            float linXYZ_AngZ[4] = {0, 0, 0, 0};

            double dist = (((acos(sin(c_lati * PI / 180.0) * sin(g_lati * PI / 180.0) +
                  cos(c_lati * PI / 180.0) * cos(g_lati * PI / 180.0) * cos((c_long - g_long) *
                  PI / 180.0))) * 180.0 / PI) * 60.0 * 1.1515) * 1609.344;

            (this->c_alti < g_alti - 0.3) ? linXYZ_AngZ[2]  = 0.2 :
                    (this->c_alti > g_alti + 0.3) ? linXYZ_AngZ[2]  = - 0.2 : linXYZ_AngZ[2] = 0;

            (this->c_yaw > 0.08 && dist > LIMITE_DIST) ? linXYZ_AngZ[3] = - 0.1 :
                    (this->c_yaw < - 0.08 && dist > LIMITE_DIST) ? linXYZ_AngZ[3] = 0.1 : linXYZ_AngZ[3] = 0;

            if (dist > LIMITE_DIST) {
                linXYZ_AngZ[0] = lati_ratio * speed;
                linXYZ_AngZ[1] = long_ratio * speed;
            }

            this->twist.linear.x  = linXYZ_AngZ[0];
            this->twist.linear.y  = linXYZ_AngZ[1];
            this->twist.linear.z  = linXYZ_AngZ[2];
            this->twist.angular.x = 0;
            this->twist.angular.y = 0;
            this->twist.angular.z = linXYZ_AngZ[3];
            this->ptop_pub.publish(twist);

//            ROS_INFO("goal : %.2lf , Current : %.2lf, dist : %.2lf", true_bearing, c_bearing, dist);
            if (dist < LIMITE_DIST && Current->next != NULL) {
                Current = Current->next;
                break;
            }
            else if (dist < LIMITE_DIST && Current->next == NULL &&  g_lati != this->h_lati && g_long != this->h_long) {
                g_lati = this->h_lati;
                g_long = this->h_long;
            }
            else if (dist < LIMITE_DIST && g_lati == this->h_lati && g_long == this->h_long) {
                P2P_DueNorth::Stop();
                return ;
            }
        }
    }
}

int main(int argc, char** argv) {

    List* list;//
    list = new List;//
    list->Latitude = 36.519644;//
    list->Longitude = 127.173052;//
    list->Altitude = 2;//
    list->Speed = 0.5;//
    list->next = new List;//
    list->next->Latitude = 36.519901;//
    list->next->Longitude = 127.173231;//
    list->next->Altitude = 2;//
    list->next->Speed = 0.5;//
    list->next->next = NULL;//

//  36.519547; 운동장 서남쪽
//  127.172806;
//  36.519686; 운동장 동남쪽
//  127.173387;
//  36.519896; 본관 정면 운동장
//  127.1728021;

    P2P_DueNorth a(argc, argv);
    a.init();
    a.Move(list);

    while(list) {//
        List* tmp = list;//
        list = list->next;//
        delete tmp;//
    }//
    return 0;
}