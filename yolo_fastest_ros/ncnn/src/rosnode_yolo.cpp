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

//OpenCv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <target_ros_msgs/BoundingBox.h>
#include <target_ros_msgs/BoundingBoxes.h>
#include <target_ros_msgs/CheckForObjectsAction.h>
#include <target_ros_msgs/ObjectCount.h>

static const char* class_names[] = {
  "sports_ball"
};

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher objects_pub_;

private:
    
    cv_bridge::CvImagePtr cv_depth_ptr;
    cv::Mat depthImgCopy_;
    cv::Mat depthAlign;
    bool if_depth = false;

    int w,h;

    double colorIntr[4]; 
    double depthIntr[4]; 
    
public:
//   static const char* class_names;
  yoloFastestv2 api;
  double  depths[10];

  std::vector<TargetBox> boxes;
  ImageConverter() : it_(nh_)
  {
    //read yaml parameters
    std::string cameraTopicName, depthTopicName;
    nh_.getParam("subscribers/camera_reading/img_topic", cameraTopicName);
    nh_.getParam("subscribers/camera_reading/depth_topic", depthTopicName);

    std::string ModelPath, ParaPath;
    nh_.getParam("yolo/model_reading/model_path", ModelPath);
    nh_.getParam("yolo/model_reading/para_path", ParaPath);
    const char *modelpath = ModelPath.data();
    const char *parapath = ParaPath.data();
    api.loadModel(parapath, modelpath);

    bool gpu;
    int input_size;
    nh_.getParam("yolo/detection_parameters/use_gpu", gpu);
    nh_.getParam("yolo/detection_parameters/input_size", input_size);
    float thresh1;
    float thresh2;
    nh_.getParam("yolo/detection_parameters/nmsThresh", thresh1);
    nh_.getParam("yolo/detection_parameters/confThresh", thresh2);
    api.loadParams(gpu, input_size, thresh1, thresh2);

    // Obtain camera intrinsics
    boost::shared_ptr<sensor_msgs::CameraInfo const> imageInfo_sub;  
    imageInfo_sub = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info",ros::Duration(3));
    if(imageInfo_sub != NULL)
    {
      colorIntr[0] = imageInfo_sub->K[0];
      colorIntr[1] = imageInfo_sub->K[4];
      colorIntr[2] = imageInfo_sub->K[2];
      colorIntr[3] = imageInfo_sub->K[5];
    }
    for (int i = 0; i < 4; i++) {
        printf("%f\n", colorIntr[i]);
    }

    boost::shared_ptr<sensor_msgs::CameraInfo const> depthInfo_sub;  
    depthInfo_sub = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/depth/camera_info",ros::Duration(3));
    if(depthInfo_sub != NULL)
    {
      depthIntr[0] = depthInfo_sub->K[0];
      depthIntr[1] = depthInfo_sub->K[4];
      depthIntr[2] = depthInfo_sub->K[2];
      depthIntr[3] = depthInfo_sub->K[5];
    }

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(cameraTopicName, 1, &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe(depthTopicName, 1, &ImageConverter::depthCb, this);
    // imageInfo_sub = nh_.subscribe("/camera/color/camera_info", 1, &ImageConverter::imageInfoCb, this);

    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    objects_pub_ = nh_.advertise<target_ros_msgs::BoundingBoxes>("/objects",10);
 
    // cv::namedWindow(OPENCV_WINDOW);
  }
 
  ~ImageConverter()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
  }

  void get_depth ()
  {
  //  depths.clear();
   ushort dp;
   for (size_t i = 0; i < boxes.size(); i++) {

    // cv::Mat regxx = cv_depth_ptr->image(cv::Rect(boxes[i].x1, boxes[i].y1,boxes[i].x2-boxes[i].x1,boxes[i].y2-boxes[i].y1));
    cv::Mat reg = depthAlign(cv::Rect(boxes[i].x1, boxes[i].y1,boxes[i].x2-boxes[i].x1,boxes[i].y2-boxes[i].y1));
    int count = 0;
    double depth = 1e4;
    // std::cout<< reg.cols<< " "<<reg.rows<<std::endl;
    for (int ii =  reg.cols/4; ii <  reg.cols*3/4; ii++ ) 
        for (int jj =  reg.rows/4; jj < reg.rows*3/4; jj++) {
            dp = reg.ptr<ushort>(jj)[ii];
            if (dp > 0 && dp<1e4 && dp<depth) {
                count ++;  
                depth = dp;   // Do your operations
              //  std::cout<<dp*1e-3<<"---"<<count<<" "<<jj<<" "<<ii<<std::endl;
            }
        }
    depths[i] = depth;
    std::cout<<"depth aligned:"<<depths[i]<<std::endl;

    }
  }

  int clip (int x, int x_min, int x_max)
  {
    if (x<x_min)
    {x=x_min;}
    else if (x>x_max)
    {x=x_max;}
    return x;
  }

  void pub_objects (const sensor_msgs::ImageConstPtr& msg)
  {
      target_ros_msgs::BoundingBoxes bboxes;
      for (size_t i = 0; i < boxes.size(); i++)
      {
         target_ros_msgs::BoundingBox bbox;
         bbox.probability =  boxes[i].score;
         bbox.xmin = boxes[i].x1;
         bbox.ymin = boxes[i].y1;
         bbox.xmax = boxes[i].x2;
         bbox.ymax = boxes[i].y2;
         bbox.distance = depths[i];
         bbox.id = i;
         bbox.Class = class_names[boxes[i].cate];
         bboxes.bounding_boxes.push_back(bbox);
      }
      bboxes.header = msg->header;
      bboxes.image_header = msg->header;
      objects_pub_.publish(bboxes);
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      w = cv_ptr->image.cols;
      h = cv_ptr->image.rows;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // clear screen
    printf("\033[2J");
    printf("\033[1;1H");
    
    boxes.clear();
    std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
    api.detection(cv_ptr->image, boxes);
    double compTime = std::chrono::duration_cast<std::chrono::microseconds>
    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
    std::cout << "detection time cost (ms)： " << compTime <<std::endl;
    for (int i = 0; i < boxes.size(); i++) {
        boxes[i].x1 = clip( boxes[i].x1, 0,  cv_ptr->image.cols );
        boxes[i].y1 = clip( boxes[i].y1, 0,  cv_ptr->image.rows );
        boxes[i].x2 = clip( boxes[i].x2, 0,  cv_ptr->image.cols );
        boxes[i].y2 = clip( boxes[i].y2, 0,  cv_ptr->image.rows );

        std::cout<<boxes[i].x1<<" "<<boxes[i].y1<<" "<<boxes[i].x2<<" "<<boxes[i].y2
                 <<" "<<boxes[i].score<<" "<< class_names[boxes[i].cate] <<std::endl;

        char text[256];
        sprintf(text, "%s %.1f%%", class_names[boxes[i].cate], boxes[i].score * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = boxes[i].x1;
        int y = boxes[i].y1 - label_size.height - baseLine;
        if (y < 0)
            y = 0;
        if (x + label_size.width > cv_ptr->image.cols)
            x = cv_ptr->image.cols - label_size.width;

        cv::rectangle(cv_ptr->image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      cv::Scalar(255, 255, 255), -1);

        cv::putText(cv_ptr->image, text, cv::Point(x, y + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));

        cv::rectangle (cv_ptr->image, cv::Point(boxes[i].x1, boxes[i].y1), 
                       cv::Point(boxes[i].x2, boxes[i].y2), cv::Scalar(255, 255, 0), 2, 2, 0);
        // }
        
        // cv::rectangle (cv_ptr->image, cv::Point(boxes[i].x1, boxes[i].y1), 
        //                 cv::Point(boxes[i].x2, boxes[i].y2), cv::Scalar(0, 0, 255), 2, 8, 0);
                       

    // // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
 
    // // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);
    
    // Output modified video stream
    
  }

  image_pub_.publish(cv_ptr->toImageMsg());
  if (if_depth && boxes.size()>0)
  {
  get_depth();
  // std::cout<<"depths:"<<depths[0]<<std::endl;
  pub_objects(msg);
  if_depth = false;
  }
  ros::spin();
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
    if_depth = true;
    try
    {
      cv_depth_ptr = cv_bridge::toCvCopy(msg, msg->encoding); //16UC1
      depthImgCopy_ = cv_depth_ptr->image.clone();
      // int w = depthImgCopy_.cols;
      // int h = depthImgCopy_.rows;
      ushort d;

      depthAlign = cv::Mat::zeros(h, w, depthImgCopy_.type());
      for(int i = 0; i < w; i++)
      {
        for(int j = 0; j < h; j++)
        {
          d = depthImgCopy_.at<ushort>(j,i);
          if(d!=0)
          {
            float x = (i - depthIntr[2]) * d * 0.001 /depthIntr[0] + 0.0150;
            float y = (j - depthIntr[3]) * d * 0.001 /depthIntr[1];
            float z = d * 0.001 - 0.0027;

            int u = int(x/z * colorIntr[0] + colorIntr[2] + 0.5);
            int v = int(y/z * colorIntr[1] + colorIntr[3] + 0.5);

            if(u > 0 && u < w && v > 0 && v < h)
              depthAlign.at<ushort>(v, u) = d;

          }

        }
      }


      std::cout<<"depth receive!"<<std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  // void imageInfoCb(const sensor_msgs::CameraInfoConstPtr& msg)
  // {
  //   //
  // }

};

int main(int argc, char** argv)
{   

  ros::init(argc, argv, "object_detector");
  ros::NodeHandle nh;
  ImageConverter ic;

  ros::spin();

    // cv::Mat cvImg = cv::imread("download.jpeg");
    // cv::imwrite("output.png", cvImg);

  return 0;
}
