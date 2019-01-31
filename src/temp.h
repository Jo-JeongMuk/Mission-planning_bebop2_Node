#ifndef TEMP
#define TEMP

#define ATTITUDE bebop_msgs::Ardrone3PilotingStateAttitudeChangedConstPtr& msg
#include "bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h"

class temp {
private:
    int init_argc;
    char **init_argv;
    double c_yaw;
public:
    temp(int argc, char **argv);
    ~temp() {}
    void init();
    void attitudeCallback(const ATTITUDE);
};
#endif