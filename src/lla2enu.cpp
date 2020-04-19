#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class Subscribe_And_Publish
{
public:
    Subscribe_And_Publish(ros::NodeHandle nh, float latInit, float longInit, float h0)
    {
        this->nh = nh;
        this->latInit = latInit;
        this->longInit = longInit;
        this->h0 = h0;

        pubCarEnu = nh.advertise<nav_msgs::Odometry>("/CarEnu", 100);
        pubObsEnu = nh.advertise<nav_msgs::Odometry>("/ObsEnu", 100);

        subCarLla = nh.subscribe("/swiftnav/front/gps_pose", 100, &Subscribe_And_Publish::CarCallback, this);
        subObsLla = nh.subscribe("/swiftnav/obs/gps_pose", 100, &Subscribe_And_Publish::ObsCallback, this);
    }

    void CarCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        // input data from msg
        float latitude = msg->latitude;
        float longitude = msg->longitude;
        float h = msg->altitude;

        float *enu;
        enu = lla2enu(latitude, longitude, h);

        nav_msgs::Odometry enuData;
        tf::TransformBroadcaster car_broadcaster;

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = msg->header.stamp;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = msg->header.frame_id;
        odom_trans.transform.translation.x = enu[0];
        odom_trans.transform.translation.y = enu[1];
        odom_trans.transform.translation.z = enu[2];
        //send the transform
        car_broadcaster.sendTransform(odom_trans);

        enuData.header.stamp = msg->header.stamp;
        enuData.header.frame_id = "odom";
        enuData.child_frame_id = msg->header.frame_id;
        enuData.pose.pose.position.x = enu[0];
        enuData.pose.pose.position.y = enu[1];
        enuData.pose.pose.position.z = enu[2];

        pubCarEnu.publish(enuData);
    }

    void ObsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        // input data from msg
        float latitude = msg->latitude;
        float longitude = msg->longitude;
        float h = msg->altitude;

        float *enu;
        enu = lla2enu(latitude, longitude, h);

        nav_msgs::Odometry enuData;
        tf::TransformBroadcaster obs_broadcaster;

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = msg->header.stamp;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = msg->header.frame_id;
        odom_trans.transform.translation.x = enu[0];
        odom_trans.transform.translation.y = enu[1];
        odom_trans.transform.translation.z = enu[2];
        //send the transform
        obs_broadcaster.sendTransform(odom_trans);

        enuData.header.stamp = msg->header.stamp;
        enuData.header.frame_id = "odom";
        enuData.child_frame_id = msg->header.frame_id;
        enuData.pose.pose.position.x = enu[0];
        enuData.pose.pose.position.y = enu[1];
        enuData.pose.pose.position.z = enu[2];

        pubObsEnu.publish(enuData);
    }

    float *lla2enu(float latitude, float longitude, float h)
    {
        //lla to ecef
        float lamb = deg_to_rad * (latitude);
        float phi = deg_to_rad * (longitude);
        float s = sin(lamb);
        float N = a / sqrt(1 - e_sq * s * s);

        float sin_lambda = sin(lamb);
        float cos_lambda = cos(lamb);
        float sin_phi = sin(phi);
        float cos_phi = cos(phi);

        float x = (h + N) * cos_lambda * cos_phi;
        float y = (h + N) * cos_lambda * sin_phi;
        float z = (h + (1 - e_sq) * N) * sin_lambda;

        // ecef to enu

        lamb = deg_to_rad * (latInit);
        phi = deg_to_rad * (longInit);
        s = sin(lamb);
        N = a / sqrt(1 - e_sq * s * s);

        sin_lambda = sin(lamb);
        cos_lambda = cos(lamb);
        sin_phi = sin(phi);
        cos_phi = cos(phi);

        float x0 = (h0 + N) * cos_lambda * cos_phi;
        float y0 = (h0 + N) * cos_lambda * sin_phi;
        float z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

        float xd = x - x0;
        float yd = y - y0;
        float zd = z - z0;

        static float enu[3];

        enu[0] = -sin_phi * xd + cos_phi * yd;                                             // float xEast = -sin_phi * xd + cos_phi * yd;
        enu[1] = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd; // float yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
        enu[2] = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;  // float zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

        return enu;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pubCarEnu;
    ros::Publisher pubObsEnu;
    ros::Subscriber subCarLla;
    ros::Subscriber subObsLla;

    float latInit, longInit, h0;

    // fixed values

    double a = 6378137;
    double b = 6356752.3142;
    double f = (a - b) / a;
    double e_sq = f * (2 - f);
    float deg_to_rad = 0.0174533;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lla2enu");

    ros::NodeHandle nh("~");

    float latInit, longInit, h0;
    nh.param<float>("latInit", latInit, 45.6311926152);
    nh.param<float>("longInit", longInit, 9.2947495255);
    nh.param<float>("h0", h0, 231.506675163);

    Subscribe_And_Publish SAPObject(nh, latInit, longInit, h0);

    ros::spin();

    return 0;
}