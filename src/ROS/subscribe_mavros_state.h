#ifndef SUBSCRIBE_MAVROS_STATE_H
#define SUBSCRIBE_MAVROS_STATE_H

#include <ros/ros.h>
#include <mavros/State.h>

class Subscribe_mavros_state
{
public:
    Subscribe_mavros_state();
    void mavrosStateCb(const mavros::StateConstPtr &msg);

    std::string getMode();
    bool getGuided();
    bool getArmed();

private:
    pthread_mutex_t mutex;

    ros::NodeHandle nh_;

    std::string mode;
    bool armed;
    bool guided;
};
#endif // SUBSCRIBE_MAVROS_STATE_H
