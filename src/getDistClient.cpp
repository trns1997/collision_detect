// #include "ros/ros.h"
// #include "collision_detect/GetDist.h"
// #include <cstdlib>

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "getDistClient");
//     if (argc != 7)
//     {
//         ROS_INFO("usage: getDistClient Carx Cary Carz Obsx Obsy Obsz");
//         return 1;
//     }

//     ros::NodeHandle n;
//     ros::ServiceClient client = n.serviceClient<collision_detect::GetDist>("getDist");
//     collision_detect::GetDist srv;
//     srv.request.carEnu_x = atoll(argv[1]);
//     srv.request.carEnu_y = atoll(argv[2]);
//     srv.request.carEnu_z = atoll(argv[3]);
//     srv.request.obsEnu_x = atoll(argv[4]);
//     srv.request.obsEnu_y = atoll(argv[5]);
//     srv.request.obsEnu_z = atoll(argv[6]);
//     if (client.call(srv))
//     {
//         ROS_INFO("Dist: %lf", (float)srv.response.dist);
//     }
//     else
//     {
//         ROS_ERROR("Failed to call service getDist");
//         return 1;
//     }

//     return 0;
// }

#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include "collision_detect/GetDist.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class Subscribe_And_Publish
{
public:
    Subscribe_And_Publish()
    {
        client = nh.serviceClient<collision_detect::GetDist>("getDist");
        subCarEnu.subscribe(nh, "/CarEnu", 1);
        subObsEnu.subscribe(nh, "/ObsEnu", 1);
        sync.reset(new Sync(MySyncPolicy(10), subCarEnu, subObsEnu));
        sync->registerCallback(boost::bind(&Subscribe_And_Publish::callback, this, _1, _2));
    }

    void callback(const nav_msgs::Odometry::ConstPtr &car, const nav_msgs::Odometry::ConstPtr &obs)
    {
        srv.request.carEnu_x = car->pose.pose.position.x;
        srv.request.carEnu_y = car->pose.pose.position.y;
        srv.request.carEnu_z = car->pose.pose.position.z;
        srv.request.obsEnu_x = obs->pose.pose.position.x;
        srv.request.obsEnu_y = obs->pose.pose.position.y;
        srv.request.obsEnu_z = obs->pose.pose.position.z;
        if (client.call(srv))
        {
            ROS_INFO("Dist: %lf", (float)srv.response.dist);
        }
        else
        {
            ROS_ERROR("Failed to call service getDist");
        }
    }

private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    collision_detect::GetDist srv;
    message_filters::Subscriber<nav_msgs::Odometry> subCarEnu;
    message_filters::Subscriber<nav_msgs::Odometry> subObsEnu;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "getDistClient");

    Subscribe_And_Publish SAPObject;

    ros::spin();

    return 0;
}