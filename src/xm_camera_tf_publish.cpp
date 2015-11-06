#include <xm_camera_tf/xm_camera_tf_publish.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

xm_camera_tf::xm_camera_tf(ros::NodeHandle nh)
    :nh_(nh)
{
    people_goal_.theta = 0;
    people_goal_.x = 0;
    people_goal_.y = 0;
    people_goal_.z = 0;
    xm_angle_sub_ = nh_.subscribe("/pan_angle", 1000, &xm_camera_tf::xmanglecCallBack, this);
    xm_people_msgs_sub_ = nh_.subscribe("/people_position_estimation", 1000, &xm_camera_tf::xmpeopleCallBack, this);
    xm_tracker_pub_ = nh_.advertise<xm_msgs::xm_Tracker>("/people_tf_position",1000);
    if (xmcameraTfPublish() == true)
        ROS_INFO("complete once transform!!!");
}

xm_camera_tf::~xm_camera_tf()
{
    ROS_INFO("destroying people position!!!");
    nh_.shutdown();
}

void xm_camera_tf::xmanglecCallBack(const xm_msgs::xm_AngleConstPtr &ptr)
{
    people_goal_.theta = ptr->xm_angle;
    ROS_INFO("the theta is %lf",people_goal_.theta);
}

void xm_camera_tf::xmpeopleCallBack(const people_msgs::PositionMeasurementConstPtr &ptr)
{
    people_goal_.x = ptr->pos.x;
    people_goal_.y = ptr->pos.y;
    people_goal_.z = ptr->pos.z;
    ROS_INFO("the people_goal_x: %lf people_goal_y: %lf people_goal_z: %lf",people_goal_.x,people_goal_.y,people_goal_.z);
    if(xmcameraTfPublish() == true)
        ROS_INFO("complete!!!");
}

bool xm_camera_tf::xmcameraTfPublish()
{
    xm_msgs::xm_TrackerPtr ptracker = boost::make_shared<xm_msgs::xm_Tracker>();
    ROS_ERROR("the tf is ok");
    ptracker->pos.x = people_goal_.x * cos(people_goal_.theta) - people_goal_.y * sin(people_goal_.theta);
    ptracker->pos.y = people_goal_.y * cos(people_goal_.theta) + people_goal_.x * sin(people_goal_.theta);
    ptracker->pos.z = people_goal_.z;
    ROS_INFO("the people goal: %lf %lf %lf", people_goal_.x,people_goal_.y,people_goal_.z);
    ROS_INFO("the people transform goal: %lf %lf %lf",ptracker->pos.x,ptracker->pos.y,ptracker->pos.z);
    xm_tracker_pub_.publish(ptracker);
    return true;
}
