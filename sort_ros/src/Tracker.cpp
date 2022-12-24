#include "Tracker.h"


int Tracker::kf_count = 0;


Tracker::Tracker(TrackerState state) {

    kf_count++;
    // std::cout <<"kf count" << kf_count << std::endl;

    // Initialize variables
    m_time_since_update = 0;
    m_hits = 0;
    m_hit_streak = 0;
    m_id = kf_count;

    // Initialize kalman filter
    kf = cv::KalmanFilter(STATE_NUM, MEASURE_NUM, 0);

    kf.transitionMatrix = (cv::Mat_<float>(STATE_NUM, STATE_NUM) <<
    
        1, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0.5*dt*dt, 0, 0,
        0, 1, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0.5*dt*dt, 0,
        0, 0, 1, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0.5*dt*dt,
        0, 0, 0, 1, 0, 0, 0, 0, dt, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, dt, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, dt, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, dt, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, dt,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1

    );

    cv::setIdentity( kf.measurementMatrix );
    cv::setIdentity( kf.processNoiseCov,        cv::Scalar::all(10)   );
    cv::setIdentity( kf.measurementNoiseCov,    cv::Scalar::all(1)    );
    cv::setIdentity( kf.errorCovPost,           cv::Scalar::all(5000)      );

    // Initialize state vector
    kf.statePost.at<float>(0, 0) = state.position(0);
    kf.statePost.at<float>(1, 0) = state.position(1);
    kf.statePost.at<float>(2, 0) = state.position(2);  
    kf.statePost.at<float>(3, 0) = state.area;
    kf.statePost.at<float>(4, 0) = state.aspectRatio;
    
}


TrackerState Tracker::predict(void) {

    if (m_time_since_update > 0)
        m_hit_streak = 0;
    m_time_since_update += 1;

    TrackerState state;
    state.fromMat(kf.predict());

    if(state.area < 0) {
        state.area = 0;
    }
    
    return state;

}


void Tracker::update(TrackerState state) {

    m_time_since_update = 0;
    m_hits += 1;
    m_hit_streak += 1;

    cv::Mat measurement = cv::Mat::zeros(MEASURE_NUM, 1, CV_32F);
    measurement.at<float>(0, 0) = state.position(0);
    measurement.at<float>(1, 0) = state.position(1);
    measurement.at<float>(2, 0) = state.position(2);
    measurement.at<float>(3, 0) = state.area;
    measurement.at<float>(4, 0) = state.aspectRatio;

    kf.correct(measurement);

}


TrackerState Tracker::getState(void) {

    TrackerState state;
    state.fromMat(kf.statePost);

    return state;
}
