#include "yolo-fastestv2.h"
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdint>
#include <algorithm>
#include <iomanip>
#include <typeinfo>
//ROS
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include "visualization_msgs/MarkerArray.h"
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

//OpenCv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <target_ros_msgs/BoundingBox.h>
#include <target_ros_msgs/BoundingBoxes.h>
#include <target_ros_msgs/CheckForObjectsAction.h>
#include <target_ros_msgs/ObjectCount.h>
#include <obj_state_msgs/ObjectsStates.h>
#include <obj_state_msgs/State.h>

#include <target_ros_msgs/BoundingBox.h>
#include <target_ros_msgs/BoundingBoxes.h>
#include <target_ros_msgs/CheckForObjectsAction.h>
#include <target_ros_msgs/ObjectCount.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>


ros::Publisher pub, mark_pub;

cv_bridge::CvImagePtr cv_depth_ptr;
cv::Mat depthImgCopy_;

Eigen::Vector3f P;
Eigen::Matrix3f rotation_matrix, R;

visualization_msgs::MarkerArray markerArrayOutput;

bool flag;
int t_ = 0;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    sensor_msgs::PointCloud2 pcmsg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    try
    {
      cv_depth_ptr = cv_bridge::toCvCopy(msg, msg->encoding); //16UC1
      depthImgCopy_ = cv_depth_ptr->image.clone();
      int w = depthImgCopy_.cols;
      int h = depthImgCopy_.rows;
      ushort d;
    
      for(int i = 0; i < w; i+=10)
      {
        for(int j = 0; j < h; j+=10)
        {
          d = depthImgCopy_.at<ushort>(j,i);
          if(d!=0 && d < 5000)
          {
            float px = (i - 320.2144470214844) * d * 0.001 /384.16455078125;
            float py = (j - 238.94403076171875) * d * 0.001 /384.16455078125;
            float pz = d * 0.001;
            Eigen::Vector3f position;
            position << px, py, pz;
            R <<  0,  0, 1,
                 -1,  0, 0,
                  0, -1, 0;

            position = R * position;
            position = rotation_matrix * position;
            position = position + P;

            if(position(2) > 0.2)
            {
                pcl::PointXYZ p(position(0), position(1), position(2));
                cloud->push_back(p);

            }

          }

        }
      }
      pcl::toROSMsg(*cloud, pcmsg);
      pcmsg.header.frame_id = "map";

      pub.publish(pcmsg);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

}

void odomCb(const nav_msgs::OdometryConstPtr& pmsg)
{

    float x = pmsg->pose.pose.orientation.x;
    float y = pmsg->pose.pose.orientation.y;
    float z = pmsg->pose.pose.orientation.z;
    float w = pmsg->pose.pose.orientation.w;

    Eigen::Quaternionf Q = Eigen::Quaternionf (w, x, y, z);  
    // Rw = q.toRotationMatrix();

    Eigen::Vector3f eulerAngle;
    eulerAngle = Q.matrix().eulerAngles(0,1,2);

    float x1 = pmsg->pose.pose.position.x;
    float y1 = pmsg->pose.pose.position.y;
    float z1 = pmsg->pose.pose.position.z;

    P << x1, y1, z1;

    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(0),Eigen::Vector3f::UnitX()));
    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1),Eigen::Vector3f::UnitY()));
    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(2),Eigen::Vector3f::UnitZ()));

    rotation_matrix = yawAngle * pitchAngle * rollAngle;

}

void statesCb(const obj_state_msgs::ObjectsStatesConstPtr& msg)
{
   int t_ = 0;

//    for(auto state : msg->states)
//    {
//         visualization_msgs::Marker marker;
//         for(int i = 0; i < 5 ; i++)
//         {
//             if(state.position.z < 5)
//             {
//             marker.header.stamp = ros::Time::now();
//             marker.header.frame_id = "map";

//             marker.frame_locked = true;
//             marker.lifetime = ros::Duration(0);
//             marker.ns = "bounding_box";
//             marker.id = t_;
//             marker.action = visualization_msgs::Marker::ADD;
//             marker.type = visualization_msgs::Marker::SPHERE;
        
//             marker.pose.position.x = state.position.x + state.velocity.x * 0.1 * i;
//             marker.pose.position.y = state.position.y + state.velocity.y * 0.1 * i;
//             marker.pose.position.z = state.position.z + state.velocity.z * 0.1 * i + 0.5 * 9.8 * (0.1 * i)* (0.1 * i);
//             marker.pose.orientation.x = 0.0;
//             marker.pose.orientation.y = 0.0;
//             marker.pose.orientation.z = 0.0;
//             marker.pose.orientation.w = 1.0;
//             marker.scale.x = 0.5;
//             marker.scale.y = 0.5;
//             marker.scale.z = 0.5;
//             marker.color.a = 0.3;
//             marker.color.r = 0.0;
//             marker.color.g = 0.0;
//             marker.color.b = 1.0;

//             t_++;

//             markerArrayOutput.markers.push_back(marker);
//             }

//         }
        
//    }

//    mark_pub.publish(markerArrayOutput);

}

void rectCallback (const target_ros_msgs::BoundingBoxes::ConstPtr& data) 
{

        ROS_INFO("published.");

}

int main(int argc, char** argv)
{   

  ros::init(argc, argv, "object_detector");
  ros::NodeHandle nh;

  pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud_topic", 10);
  mark_pub = nh.advertise<visualization_msgs::MarkerArray> ("/markers_tracked", 1);
  ros::Subscriber depth_sub_, odom_sub_,states_sub_, box_sub_;
  depth_sub_ = nh.subscribe("/camera/depth/image_rect_raw", 1, imageCb);
  odom_sub_ = nh.subscribe("/vicon_imu_ekf_odom", 1, odomCb);
  states_sub_ = nh.subscribe("/objects_states", 1, statesCb);
  box_sub_ = nh.subscribe<target_ros_msgs::BoundingBoxes> ("/objects", 1, rectCallback);
  ros::spin();

    // cv::Mat cvImg = cv::imread("download.jpeg");
    // cv::imwrite("output.png", cvImg);
 
  return 0;
}