#include "ros/ros.h"
#include "temp.h"

#define ATTI_PUB "bebop/states/ardrone3/PilotingState/AttitudeChanged"
#define PI 3.14159265

temp::temp(int argc, char** argv)
: init_argc(argc), init_argv(argv)
{

}

void temp::init()
{
    ros::init(init_argc, init_argv, "temp");
    ros::NodeHandle tempnh;
    ros::Subscriber attitude = tempnh.subscribe
            (ATTI_PUB, 100, &temp::attitudeCallback, this);
}

void temp::attitudeCallback(const ATTITUDE)
{
    this->c_yaw = msg->yaw;
    ROS_INFO("yaw : %lf", this->c_yaw);
    double c_bearing = (this->c_yaw * 180 / PI);
    ROS_INFO("degree : %lf\n", c_bearing);
}

int main(int argc, char** argv) {

    temp a(argc, argv);
    a.init();
    ros::spin();
    return 0;
}