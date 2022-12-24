#ifndef TRACKER_STATE_H
#define TRACKER_STATE_H

#define	MEASURE_NUM	5

#include "cv_bridge/cv_bridge.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>


struct TrackerState {

    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration;

	float area;
	float aspectRatio;
	float vs;
	float vr;

	int id;

	cv::Mat toMat(void);
	void fromMat(cv::Mat mat);

};


#endif