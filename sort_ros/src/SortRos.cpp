#include "SortRos.h"

bool flag;

cv_bridge::CvImagePtr cam_image;
cv::Mat ImgCopy_;
cv::Point p3, p4;
cv::Scalar colorRectangle1(0, 0, 255);

ros::Time t_stamp;
double t_box;
double last_t = 0;

SortRos::SortRos(ros::NodeHandle &nh):nh(nh),it(nh) 
{

    double maxAge = 2;
    double minHits = 3;
    double iouThreshold = 0.15;

    ros::param::get("/sort_ros/max_age", maxAge);
    ros::param::get("/sort_ros/min_hits", minHits);
    ros::param::get("/sort_ros/iou_threshold", iouThreshold);

    SortRos::s = new Sort(maxAge, minHits, iouThreshold);

    uavvel_sub.subscribe(nh, "/lmh_uav/viconros/mocap/vel", 1);
    uavpos_sub.subscribe(nh, "/mavros/vision_pose/pose", 1);
    sync_.reset(new Sync(syncpolicy(10), uavvel_sub, uavpos_sub));
    sync_->registerCallback(boost::bind(&SortRos::UAVPV_callback, this, _1, _2));

    SortRos::image_sub = it.subscribe("/image_converter/output_video", 1, &SortRos::ImageCallback, this);
    SortRos::box_sub = nh.subscribe<target_ros_msgs::BoundingBoxes> ("/objects", 1, &SortRos::rectArrayCallback, this);
    SortRos::ball_vicon_sub = nh.subscribe<geometry_msgs::PoseStamped> ("/lmh_ball/viconros/mocap/pos", 1, &SortRos::ballViconCallback, this);

    SortRos::mark_pub = nh.advertise<visualization_msgs::MarkerArray> ("/markers_tracked", 1);
    SortRos::path_pub = nh.advertise<nav_msgs::Path> ("/ball_path", 1);
    SortRos::ball_vicon_pub = nh.advertise<nav_msgs::Path> ("/ball_vicon_path", 1);
    SortRos::ball_posestamp_pub = nh.advertise<geometry_msgs::PoseStamped> ("/ball_pose_tracking", 1);
    SortRos::obj_pub = nh.advertise<obj_state_msgs::ObjectsStates> ("/objects_states", 1);
    SortRos::image_pub = it.advertise("/tracking_img", 1);

}

void SortRos::UAVPV_callback (const geometry_msgs::TwistStampedConstPtr& vmsg, const geometry_msgs::PoseStampedConstPtr& pmsg)
{
    // std::cout << "hello" << std::endl;
    double t_now = vmsg->header.stamp.toSec();

    float x = pmsg->pose.orientation.x;
    float y = pmsg->pose.orientation.y;
    float z = pmsg->pose.orientation.z;
    float w = pmsg->pose.orientation.w;

    float x1 = pmsg->pose.position.x;
    float y1 = pmsg->pose.position.y;
    float z1 = pmsg->pose.position.z;

    float vx = vmsg->twist.linear.x;
    float vy = vmsg->twist.linear.y;
    float vz = vmsg->twist.linear.z;

    float wx = vmsg->twist.angular.x;
    float wy = vmsg->twist.angular.y;
    float wz = vmsg->twist.angular.z;

    Eigen::Quaternionf Q = Eigen::Quaternionf (w, x, y, z);  
    // Rw = q.toRotationMatrix();

    Eigen::Vector3f eulerAngle;
    eulerAngle = Q.matrix().eulerAngles(0,1,2);
    // std::cout << x << std::endl;
    // std::cout << Q.x() << std::endl;
    double dt_ = t_now - t_box;

    x1 = x1 - vx * dt_;
    y1 = y1 - vy * dt_;
    z1 = z1 - vz * dt_;
    // uav position
    P << x1, y1, z1;

    // uav orientation
    eulerAngle(0) = eulerAngle(0) - wx * dt_;
    eulerAngle(1) = eulerAngle(1) - wy * dt_;
    eulerAngle(2) = eulerAngle(2) - wz * dt_;

    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(0),Eigen::Vector3f::UnitX()));
    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1),Eigen::Vector3f::UnitY()));
    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(2),Eigen::Vector3f::UnitZ()));

    rotation_matrix = yawAngle * pitchAngle * rollAngle;

}


void SortRos::ballViconCallback (const geometry_msgs::PoseStampedConstPtr& msg)
{
    
    ballviconpath.header.stamp = msg->header.stamp;
    ballviconpath.header.frame_id = "map";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.stamp = msg->header.stamp;
    this_pose_stamped.header.frame_id = "map";
    this_pose_stamped.pose.position.x = msg->pose.position.x;
    this_pose_stamped.pose.position.y = msg->pose.position.y;
    this_pose_stamped.pose.position.z = msg->pose.position.z;
    this_pose_stamped.pose.orientation.x = 0.0;
    this_pose_stamped.pose.orientation.y = 0.0;
    this_pose_stamped.pose.orientation.z = 0.0;
    this_pose_stamped.pose.orientation.w = 1.0;
    ballviconpath.poses.push_back(this_pose_stamped);

    ball_vicon_pub.publish(ballviconpath);

}


void SortRos::ImageCallback (const sensor_msgs::ImageConstPtr& msg) 
{
    // TrackingAlgorithm();
    if (cam_image) 
    {   
        SortRos::image_pub.publish(cam_image->toImageMsg());
        // cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
	    // cv::imshow("Display window", ImgCopy_);
	    // cv::waitKey(3);
    }

    cam_image = cv_bridge::toCvCopy(msg, msg->encoding);
    if (cam_image) 
    {
        ImgCopy_ = cam_image->image.clone();
    }

}


void SortRos::rectArrayCallback (const target_ros_msgs::BoundingBoxes::ConstPtr& data) 
{
    // flag = 1;

    clock_t start, finish;
    start = clock();

    t_stamp = data->header.stamp;
    t_box = t_stamp.toSec();
    d_t_ = t_box - last_t;
    last_t = t_box;
    // std::cout << d_t_ << std::endl;

    rects.clear();
    for(auto box : data->bounding_boxes) 
    {

        SortRect rect;

        rect.id = box.id;
        rect.centerX    = (float)(box.xmin + box.xmax)/2;
        rect.centerY    = (float)(box.ymin + box.ymax)/2;
        rect.width      = (float)(box.xmax - box.xmin);
        rect.height     = (float)(box.ymax - box.ymin);
        rect.distance   = (float)(box.distance); 

        rects.push_back(rect);

    }    

    //tracking algorithm
    TrackingAlgorithm();

    finish = clock();
    std::cout << "Tracking time: " << (double)(finish - start)/CLOCKS_PER_SEC * 1000 << "ms" << std::endl;

}


void SortRos::TrackingAlgorithm()
{
    // std::cout <<"flag"<< flag << std::endl;
    output = SortRos::s->update(rects, ImgCopy_, rotation_matrix, P); 
    // if(flag)
    // {
    //     output = SortRos::s->update(rects, ImgCopy_); 
    // }
    // else
    // {
    //     std::vector<SortRect> temp_rects;
    //     std::cout <<"!!!!!!" << std::endl;
    //     for(auto state : output)
    //     {
    //         SortRect temp_rect;
    //         temp_rect.fromTrackerState(state);
    //         temp_rects.push_back(temp_rect);
    //     }
    //     output = SortRos::s->update(temp_rects, ImgCopy_);
    // }

    int time_point = 1;

    visualization_msgs::MarkerArray markerArrayOutput;
    std::vector<Eigen::Vector3f> predict_points;

    obj_state_msgs::ObjectsStates states_output;
    states_output.header.stamp = t_stamp;
    states_output.header.frame_id = "map";

    // nav_msgs::Path ballpath;
    ballpath.header.stamp = t_stamp;
    ballpath.header.frame_id = "map";

    for(auto state : output) 
    {

        SortRect rect;
        rect.fromTrackerState(state, rotation_matrix, P);
        rect.id = state.id;

        p3.x = ((rect.centerX - rect.width/2) < 1) ? 1 : (rect.centerX - rect.width/2);
        p3.y = ((rect.centerY - rect.height/2) < 1) ? 1 : (rect.centerY - rect.height/2);
        p4.x = ((rect.centerX + rect.width/2) > ImgCopy_.cols) ? (ImgCopy_.cols - 1) : (rect.centerX + rect.width/2);
        p4.y = ((rect.centerY + rect.height/2) > ImgCopy_.rows) ? (ImgCopy_.rows - 1) : (rect.centerY + rect.height/2);

        cv::rectangle(cam_image->image, p3, p4, colorRectangle1, 3);
        cv::putText(cam_image->image, std::to_string(rect.id), p3, cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 255, 255), 2, 8, 0);

        //print state information
        std::cout << "ball id :" << rect.id << std::endl;
        std::cout << "position x :" << state.position(0)/1000 << std::endl;
        std::cout << "position y :" << state.position(1)/1000 << std::endl;
        std::cout << "position z :" << state.position(2)/1000 << std::endl;
        std::cout << "velocity x :" << state.velocity(0)/1000 << std::endl;
        std::cout << "velocity y :" << state.velocity(1)/1000 << std::endl;
        std::cout << "velocity z :" << state.velocity(2)/1000 << std::endl;
        std::cout << "acceleration x :" << state.acceleration(0)/1000 << std::endl;
        std::cout << "acceleration y :" << state.acceleration(1)/1000 << std::endl;
        std::cout << "acceleration z :" << state.acceleration(2)/1000 << std::endl;

        //obj msg
        obj_state_msgs::State state_msg;
        state_msg.position.x = state.position(0)/1000;
        state_msg.position.y = state.position(1)/1000;
        state_msg.position.z = state.position(2)/1000;
        state_msg.velocity.x = state.velocity(0)/1000;
        state_msg.velocity.y = state.velocity(1)/1000;
        state_msg.velocity.z = state.velocity(2)/1000;
        state_msg.acceleration.x = state.acceleration(0)/1000;
        state_msg.acceleration.y = state.acceleration(1)/1000;
        state_msg.acceleration.z = state.acceleration(2)/1000;
        states_output.states.push_back(state_msg);
        
        // the detection box
        visualization_msgs::Marker marker;

        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "camera_link";

        marker.frame_locked = true;
        marker.lifetime = ros::Duration(0);
        marker.ns = "bounding_box";
        marker.id = time_point;
        time_point += 1;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::SPHERE;
        
        marker.pose.position.x = state.position(0)/1000;
        marker.pose.position.y = state.position(1)/1000;
        marker.pose.position.z = state.position(2)/1000;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        markerArrayOutput.markers.push_back(marker);


        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.header.stamp = t_stamp;
        this_pose_stamped.header.frame_id = "map";
        this_pose_stamped.pose.position.x = state.position(0)/1000;
        this_pose_stamped.pose.position.y = state.position(1)/1000;
        this_pose_stamped.pose.position.z = state.position(2)/1000;
        this_pose_stamped.pose.orientation.x = 0.0;
        this_pose_stamped.pose.orientation.y = 0.0;
        this_pose_stamped.pose.orientation.z = 0.0;
        this_pose_stamped.pose.orientation.w = 1.0;
        ball_posestamp_pub.publish(this_pose_stamped);
        ballpath.poses.push_back(this_pose_stamped);

        // predict_points = trajectory.generator(worldstate);
        predict_points = TrajactoryGenerator(state);
        //the prediction box
        for(auto point : predict_points)
        {

            // visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "map";

            marker.frame_locked = true;
            marker.lifetime = ros::Duration(0);
            marker.ns = "bounding_box";
            marker.id = time_point;
            time_point += 1;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::SPHERE;
        
            marker.pose.position.x = point(0)/1000;
            marker.pose.position.y = point(1)/1000;
            marker.pose.position.z = point(2)/1000;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 0.3;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            markerArrayOutput.markers.push_back(marker);

        }
    }

    SortRos::mark_pub.publish(markerArrayOutput);
    SortRos::obj_pub.publish(states_output);
    SortRos::path_pub.publish(ballpath);

    // flag = 0;
}

std::vector<Eigen::Vector3f> SortRos::TrajactoryGenerator(TrackerState state)
{

    float delta_t = 0.2;
    float predict_x = state.position(0);
    float predict_y = state.position(1);
    float predict_z = state.position(2);

    Eigen::Vector3f prediction;

    std::vector<Eigen::Vector3f> predict_points;

    for(int i =1; i <= 15; ++i){

        predict_x = predict_x + state.velocity(0) * delta_t + 0.5 * state.acceleration(0) * delta_t * delta_t;
        predict_y = predict_y + state.velocity(1) * delta_t + 0.5 * state.acceleration(1) * delta_t * delta_t;
        predict_z = predict_z + state.velocity(2) * delta_t + 0.5 * state.acceleration(2) * delta_t * delta_t;

        prediction << predict_x, predict_y, predict_z;

        predict_points.push_back(prediction);

    }
    
    return predict_points;

}