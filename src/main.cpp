#include <ros/ros.h>
#include <xm_camera_tf/xm_camera_tf_publish.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xm_camera_transform");
    ros::NodeHandle node;
    xm_camera_tf pos(node);
    ros::spin();
}
