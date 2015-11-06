#ifndef XM_CAMERA_TF_PUBLISH_H
#define XM_CAMERA_TF_PUBLISH_H

#include <ros/ros.h>
#include <people_msgs/PositionMeasurement.h>
#include <xm_msgs/xm_Angle.h>
#include <xm_msgs/xm_Tracker.h>
#include <tf/tf.h>
#include <cmath>

typedef struct
{
    double x;
    double y;
    double z;
    double theta;
}PeopleGoal;

class xm_camera_tf
{
public:
    xm_camera_tf(ros::NodeHandle nh);
    virtual ~xm_camera_tf();
private:
    ros::NodeHandle nh_;
    ros::Subscriber xm_angle_sub_;
    ros::Subscriber xm_people_msgs_sub_;
    ros::Publisher xm_tracker_pub_;
    PeopleGoal people_goal_;

private:
    void xmanglecCallBack(const xm_msgs::xm_AngleConstPtr &ptr);
    void xmpeopleCallBack(const people_msgs::PositionMeasurementConstPtr &ptr);
    bool xmcameraTfPublish();
};

#endif //XM_CAMERA_TF_PYBLISH_H
