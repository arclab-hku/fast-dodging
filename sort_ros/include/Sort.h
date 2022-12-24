#ifndef SORT_H
#define SORT_H

#include "Tracker.h"
#include "Hungarian.h"
#include "SortRect.h"
#include <vector>
#include <set>



class Sort 
{

    public:

        Sort(int maxAge, int minHits, float iouThreshold);

        std::vector<TrackerState> update(std::vector<SortRect> rects, cv::Mat img, Eigen::Matrix3f rotation_matrix, Eigen::Vector3f P);


    private:

        HungarianAlgorithm HungAlgo;

        int maxAge;
        int minHits;
        float iouThreshold;

        std::vector<Tracker> trackers;

        float iou(SortRect rect1, SortRect rect2);
        float compareFeature(SortRect rect1, SortRect rect2, cv::Mat img);

};


#endif