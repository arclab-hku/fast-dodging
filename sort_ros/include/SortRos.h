#ifndef SORT_ROS_H
#define SORT_ROS_H

#include "ros/ros.h"
#include <thread>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include "visualization_msgs/MarkerArray.h"
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <target_ros_msgs/BoundingBoxes.h>
#include <obj_state_msgs/ObjectsStates.h>
#include <obj_state_msgs/State.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "Sort.h"


class SortRos 
{

private:

    ros::NodeHandle &nh;
    image_transport::ImageTransport it;

    ros::Publisher mark_pub;
    ros::Publisher path_pub;
    ros::Publisher ball_vicon_pub;
    ros::Publisher ball_posestamp_pub;
    ros::Publisher obj_pub;
    ros::Subscriber box_sub;
    ros::Subscriber ball_vicon_sub;

    message_filters::Subscriber<geometry_msgs::TwistStamped> uavvel_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped>  uavpos_sub;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, geometry_msgs::PoseStamped> syncpolicy;
    typedef message_filters::Synchronizer<syncpolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    Eigen::Vector3f P;
    Eigen::Matrix3f rotation_matrix;

    double d_t_;

    std::vector<SortRect> rects;
    std::vector<TrackerState> output;
    
    nav_msgs::Path ballpath;
    nav_msgs::Path ballviconpath;
   
    void rectArrayCallback (const target_ros_msgs::BoundingBoxes::ConstPtr& data);
    void ImageCallback (const sensor_msgs::ImageConstPtr& msg);
    void UAVPV_callback (const geometry_msgs::TwistStampedConstPtr& vmsg, const geometry_msgs::PoseStampedConstPtr& pmsg);
    void ballViconCallback (const geometry_msgs::PoseStampedConstPtr& msg);
    void TrackingAlgorithm();
    std::vector<Eigen::Vector3f> TrajactoryGenerator(TrackerState state);

    Sort *s;

public:

    SortRos(ros::NodeHandle &nh);
    ~SortRos(){}

};


#endif