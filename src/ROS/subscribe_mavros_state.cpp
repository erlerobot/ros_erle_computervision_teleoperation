#include "subscribe_mavros_state.h"

Subscribe_mavros_state::Subscribe_mavros_state()
{
    if (pthread_mutex_init(&mutex, NULL) != 0){
        std::cout << "mutex init failed" << std::endl;
    }
}

void Subscribe_mavros_state::mavrosStateCb(const mavros::StateConstPtr &msg)
{
    ROS_INFO("I heard: [%s] [%d] [%d]", msg->mode.c_str(), msg->armed, msg->guided);
    pthread_mutex_lock( &mutex );
    this->mode = msg->mode;
    this->guided = msg->guided==128;
    this->armed = msg->armed==128;
    pthread_mutex_unlock( &mutex );

}

std::string Subscribe_mavros_state::getMode()
{
    std::string result;
    pthread_mutex_lock( &mutex );
    result = this->mode;
    pthread_mutex_unlock( &mutex );
    return result;
}

bool Subscribe_mavros_state::getGuided()
{
    bool result;
    pthread_mutex_lock( &mutex );
    result = this->guided;
    pthread_mutex_unlock( &mutex );
    return result;
}

bool Subscribe_mavros_state::getArmed()
{
    bool result;
    pthread_mutex_lock( &mutex );
    result = this->armed;
    pthread_mutex_unlock( &mutex );
    return result;
}
