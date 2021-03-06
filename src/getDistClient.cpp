#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include "collision_detect/GetDist.h"
#include "collision_detect/collisionInfo.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <collision_detect/safetyTreshConfig.h>

class Subscribe_And_Publish
{
public:
    Subscribe_And_Publish()
    {
        client = nh.serviceClient<collision_detect::GetDist>("getDist");
        pubCollisionStatus = nh.advertise<collision_detect::collisionInfo>("/DistStatus", 1);

        reconfServ.setCallback(boost::bind(&Subscribe_And_Publish::reconfCallback, this, _1, _2));

        subCarEnu.subscribe(nh, "/CarEnu", 1);
        subObsEnu.subscribe(nh, "/ObsEnu", 1);
        sync.reset(new Sync(MySyncPolicy(10), subCarEnu, subObsEnu));
        sync->registerCallback(boost::bind(&Subscribe_And_Publish::callback, this, _1, _2));
    }

    void reconfCallback(collision_detect::safetyTreshConfig &config, uint32_t level)
    {
        // ROS_INFO("Reconfigure Request: %d %d",
        //          config.Min_Safe_Distance, config.Max_Crash_Distance);
        minSafeDist = config.Min_Safe_Distance;
        maxCrashDist = config.Max_Crash_Distance;
    }

    void callback(const nav_msgs::Odometry::ConstPtr &car, const nav_msgs::Odometry::ConstPtr &obs)
    {
        srv.request.carEnu_x = car->pose.pose.position.x;
        srv.request.carEnu_y = car->pose.pose.position.y;
        srv.request.carEnu_z = car->pose.pose.position.z;
        srv.request.obsEnu_x = obs->pose.pose.position.x;
        srv.request.obsEnu_y = obs->pose.pose.position.y;
        srv.request.obsEnu_z = obs->pose.pose.position.z;

        collision_detect::collisionInfo collisionStatus;

        if (client.call(srv))
        {
            ROS_INFO("Dist: %lf", (float)srv.response.dist);
            collisionStatus.dist = (float)srv.response.dist;
            if (collisionStatus.dist > minSafeDist)
            {
                collisionStatus.flag = "Safe";
            }
            else if (collisionStatus.dist > maxCrashDist && collisionStatus.dist < minSafeDist)
            {
                collisionStatus.flag = "Unsafe";
            }
            else
            {
                collisionStatus.flag = "Crash";
            }
            pubCollisionStatus.publish(collisionStatus);
        }
        else
        {
            ROS_ERROR("Failed to call service getDist");
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pubCollisionStatus;
    ros::ServiceClient client;

    collision_detect::GetDist srv;

    dynamic_reconfigure::Server<collision_detect::safetyTreshConfig> reconfServ;
    dynamic_reconfigure::Server<collision_detect::safetyTreshConfig>::CallbackType f;

    message_filters::Subscriber<nav_msgs::Odometry> subCarEnu;
    message_filters::Subscriber<nav_msgs::Odometry> subObsEnu;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    int minSafeDist, maxCrashDist;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "getDistClient");

    Subscribe_And_Publish SAPObject;

    ros::spin();

    return 0;
}