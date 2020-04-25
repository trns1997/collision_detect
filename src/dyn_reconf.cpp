#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <collision_detect/safetyTreshConfig.h>

void callback(collision_detect::safetyTreshConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %d %d",
             config.Min_Safe_Distance, config.Max_Crash_Distance);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_tutorials");

    dynamic_reconfigure::Server<collision_detect::safetyTreshConfig> server;
    dynamic_reconfigure::Server<collision_detect::safetyTreshConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}