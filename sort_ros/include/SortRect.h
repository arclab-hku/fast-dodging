#ifndef SORT_RECT_H
#define SORT_RECT_H


#include "TrackerState.h"


struct SortRect {

    int id;
    float centerX;
    float centerY;
    float width;
    float height;
    float distance; 

    TrackerState toTrackerState(Eigen::Matrix3f rotation_matrix, Eigen::Vector3f T);
    void fromTrackerState(TrackerState state, Eigen::Matrix3f rotation_matrix, Eigen::Vector3f T);
};


#endif