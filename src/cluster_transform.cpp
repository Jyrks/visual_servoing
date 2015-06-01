#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class PointCloudTransformer {
public:
    PointCloudTransformer()
    {
        sub_ = n_.subscribe("/camera/depth/points", 1, &PointCloudTransformer::callback, this);
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        tf::Transform camera_transform;
        camera_transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        camera_transform.setRotation(q);
        br_.sendTransform(tf::StampedTransform(camera_transform, msg->header.stamp, "xtion_link", "camera_link"));
    }


private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    tf::TransformBroadcaster br_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_cloud");

    PointCloudTransformer cloudTransformer;

    ros::Rate r(10);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}