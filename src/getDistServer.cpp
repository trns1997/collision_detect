#include "ros/ros.h"
#include "collision_detect/GetDist.h"

bool getDist(collision_detect::GetDist::Request &req,
             collision_detect::GetDist::Response &res)
{
    res.dist = sqrt(pow(req.carEnu_x - req.obsEnu_x, 2.0) + pow(req.carEnu_y - req.obsEnu_y, 2.0) + pow(req.carEnu_z - req.obsEnu_z, 2.0));
    ROS_INFO("request: Carx=%lf, Cary=%lf, Carz=%lf, Obsx=%lf, Obsy=%lf, Obsz=%lf, ", (float)req.carEnu_x, (float)req.carEnu_y, (float)req.carEnu_z, (float)req.obsEnu_x, (float)req.obsEnu_y, (float)req.obsEnu_z);
    ROS_INFO("sending back response: [%lf]", (float)res.dist);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "getDistServer");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("getDist", getDist);
    ROS_INFO("Ready to Get Distance.");
    ros::spin();

    return 0;
}