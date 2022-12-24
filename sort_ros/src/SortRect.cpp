#include "SortRect.h"


Eigen::Matrix3f R;


TrackerState SortRect::toTrackerState(Eigen::Matrix3f rotation_matrix, Eigen::Vector3f T) {

    TrackerState state;
    
    float px = (centerX - 647.6) * distance/911.6;  //   x/z * fx + cx = u
    float py = (centerY - 369.1) * distance/910.8;  //   y/z * fy + cy = u
    float pz = distance;

    Eigen::Vector3f position;
    position << px, py, pz;
    R <<  0,  0, 1,
         -1,  0, 0,
          0, -1, 0;

    position = R * position;
    position = rotation_matrix * position;
    position = position + T * 1000;

    state.position = position;

    state.area = width * height;
    state.aspectRatio = width / height;
    
    return state;
}


void SortRect::fromTrackerState(TrackerState state, Eigen::Matrix3f rotation_matrix, Eigen::Vector3f T) {

    Eigen::Vector3f p;
    R <<  0,  0, 1,
         -1,  0, 0,
          0, -1, 0;
    
    // p = R.inverse() * state.position;

    p = rotation_matrix.inverse() * (state.position - T * 1000);
    p = R.inverse() * p;

    centerX = p(0) * 911.6 / p(2) + 647.6;
    centerY = p(1) * 910.8 / p(2) + 369.1;
    distance = p(2); 

    if(state.area > 0) {
        width = sqrt(state.area * state.aspectRatio);
        height = state.area / width;
    }
    else {
        width = 0;
        height = 0;
    }

}
