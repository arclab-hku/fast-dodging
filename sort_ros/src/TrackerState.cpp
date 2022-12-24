#include "TrackerState.h"


cv::Mat TrackerState::toMat(void) {

    cv::Mat mat = cv::Mat::zeros(MEASURE_NUM, 1, CV_32F);
    mat.at<float>(0, 0) = position(0);
    mat.at<float>(1, 0) = position(1);
    mat.at<float>(2, 0) = position(2);  
    mat.at<float>(3, 0) = area;
    mat.at<float>(4, 0) = aspectRatio;

    return mat;
}

void TrackerState::fromMat(cv::Mat mat) {

    position(0)     = mat.at<float>(0, 0);
    position(1)     = mat.at<float>(1, 0);
    position(2)     = mat.at<float>(2, 0); 
    area            = mat.at<float>(3, 0);
    aspectRatio     = mat.at<float>(4, 0);
    velocity(0)     = mat.at<float>(5, 0);
    velocity(1)     = mat.at<float>(6, 0);
    velocity(2)     = mat.at<float>(7, 0);
    vs              = mat.at<float>(8, 0);
    vr              = mat.at<float>(9, 0);
    acceleration(0) = mat.at<float>(10, 0);
    acceleration(1) = mat.at<float>(11, 0);
    acceleration(2) = mat.at<float>(12, 0);
    
}
