#include "Sort.h"
#include <cfloat>


Sort::Sort(int maxAge = 5, int minHits = 5, float iouThreshold = 0.15) 
{

    this->maxAge = maxAge;
    this->minHits = minHits;
    this->iouThreshold = iouThreshold;

}


std::vector<TrackerState> Sort::update(std::vector<SortRect> detections, cv::Mat img, Eigen::Matrix3f rotation_matrix, Eigen::Vector3f P) 
{

    if(trackers.size() == 0) 
    {
        
        for (int i=0; i<detections.size(); ++i) 
        {

            TrackerState state = detections[i].toTrackerState(rotation_matrix, P);
            Tracker tracker = Tracker(state);

            trackers.push_back(tracker);
        }

        return std::vector<TrackerState>();
    }


    std::vector<SortRect> predictions;
    for(int i=0; i<trackers.size(); ++i) 
    {
        TrackerState state = trackers[i].predict();

        SortRect rect;
        rect.fromTrackerState(state, rotation_matrix, P);
        rect.id = 0;

        predictions.push_back(rect);
    }


    vector<vector<double>> iouMatrix;
	iouMatrix.resize(predictions.size(), vector<double>(detections.size(), 0));
    vector<vector<double>> histMatrix;
    histMatrix.resize(predictions.size(), vector<double>(detections.size(), 0));
    vector<vector<double>> threshMatrix;
    threshMatrix.resize(predictions.size(), vector<double>(detections.size(), 0));

    for(int i=0; i<predictions.size(); ++i) 
    {
        for(int j=0; j<detections.size(); ++j)
        {
            iouMatrix[i][j] = 1 - iou(predictions[i], detections[j]);
            histMatrix[i][j] = compareFeature(predictions[i], detections[j], img);
            threshMatrix[i][j] = 0.2*histMatrix[i][j] + 0.9*iouMatrix[i][j];
            // std::cout <<" thresh" << histMatrix[i][j] << std::endl;
            // std::cout << "iou " << iouMatrix[i][j] << std::endl;
        }
    }
        // for(int j=0; j<detections.size(); j++)
        //     iouMatrix[i][j] = 1 - iou(predictions[i], detections[j]);
    
    vector<int> assignment;

    HungAlgo.Solve(threshMatrix, assignment);


	set<int> allItems;
	set<int> matchedItems;
    set<int> unmatchedDetections;
    
    for (int i=0; i<detections.size(); ++i)
        allItems.insert(i);

    for (int i=0; i<predictions.size(); ++i)
        if (assignment[i] != -1)
            matchedItems.insert(assignment[i]);

    std::set_difference(
        allItems.begin(), allItems.end(),
        matchedItems.begin(), matchedItems.end(),
        insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin())
    );

    vector<std::pair<int, int>> matchedPairs;
    for(int i=0; i<assignment.size(); ++i) 
    {

        if(assignment[i] == -1) 
            continue;

        // std::cout << iouMatrix[i][assignment[i]] << std::endl;

        if(1 - threshMatrix[i][assignment[i]] < iouThreshold)
            unmatchedDetections.insert(assignment[i]);
        else
            matchedPairs.push_back(make_pair(i, assignment[i]));
    }
    


    for(auto pair : matchedPairs) 
    {

        int trackerIndex = pair.first;
        int detectionIndex = pair.second;

        TrackerState state = detections[detectionIndex].toTrackerState(rotation_matrix, P);

        trackers[trackerIndex].update(state);
    }


    
    for (auto detectionIndex : unmatchedDetections) 
    {

        TrackerState state = detections[detectionIndex].toTrackerState(rotation_matrix, P);

        Tracker tracker = Tracker(state);
        trackers.push_back(tracker);
    }

    std::vector<TrackerState> result;

    for (auto it = trackers.begin(); it != trackers.end();) 
    {
    
        if((*it).m_time_since_update > maxAge) 
        {

            it = trackers.erase(it);

        }
        else 
        {

            if( (*it).m_time_since_update < 1 && (*it).m_hit_streak >= minHits ) 
            {
                
                TrackerState state = (*it).getState();

                state.id = (*it).m_id;
                result.push_back(state);
                
            }

            it++;
        }

    }

    return result;

}


float Sort::iou(SortRect rect1, SortRect rect2) 
{

    float rect1_x1 = rect1.centerX - rect1.width/2;
    float rect1_y1 = rect1.centerY - rect1.height/2;
    float rect1_x2 = rect1.centerX + rect1.width/2;
    float rect1_y2 = rect1.centerY + rect1.height/2;

    float rect2_x1 = rect2.centerX - rect2.width/2;
    float rect2_y1 = rect2.centerY - rect2.height/2;
    float rect2_x2 = rect2.centerX + rect2.width/2;
    float rect2_y2 = rect2.centerY + rect2.height/2;


    float x1 = max(rect1_x1, rect2_x1);
    float y1 = max(rect1_y1, rect2_y1);
    float x2 = min(rect1_x2, rect2_x2);
    float y2 = min(rect1_y2, rect2_y2);

    float w = max(0.f, x2 - x1);
    float h = max(0.f, y2 - y1);

    float area1 = (rect1_x2 - rect1_x1) * (rect1_y2 - rect1_y1);
    float area2 = (rect2_x2 - rect2_x1) * (rect2_y2 - rect2_y1);
    float area3 = w * h;

    float iou = area3 / (area1 + area2 - area3 + DBL_EPSILON);

    return iou;
}

float Sort::compareFeature(SortRect rect1, SortRect rect2, cv::Mat img) 
{
    int rect1_x1 = int(rect1.centerX - rect1.width/2);
    int rect1_y1 = int(rect1.centerY - rect1.height/2);
    int rect1_w = int(rect1.width) + 1;
    int rect1_h = int(rect1.height) + 1;

    if( (rect1_x1 + rect1_w) < 1 || (rect1_y1 + rect1_h) < 1 || rect1_x1 > img.cols || rect1_y1 > img.rows)
    {
        return 0;
    }
    if(rect1_x1 < 1)
    {
        rect1_x1 = 1;
    }
    if(rect1_y1 < 1)
    {
        rect1_y1 = 1;
    }
    if( (rect1_x1 + rect1_w) > img.cols)
    {
        rect1_w = img.cols - rect1_x1 - 1;
    }
    if( (rect1_y1 + rect1_h) > img.rows)
    {
        rect1_h = img.rows - rect1_y1 -1;
    }  
    cv::Rect rect_pre(rect1_x1, rect1_y1, rect1_w, rect1_h);
    cv::Mat roi1 = img(rect_pre);

    int h_bins = 50; int s_bins = 60;     
	int histSize[] = { h_bins, s_bins };
    float h_ranges[] = { 0, 180 };     
	float s_ranges[] = { 0, 256 };
	const float* ranges[] = { h_ranges, s_ranges };
    int channels[] = { 0, 1 };

    cv::Mat hsvroi1;
    cvtColor(roi1, hsvroi1, CV_BGR2HSV);
    cv::MatND hist1;
    cv::calcHist(&hsvroi1, 1, channels, cv::Mat(), hist1, 2, histSize, ranges, true, false);

    int rect2_x1 = int(rect2.centerX - rect2.width/2);
    int rect2_y1 = int(rect2.centerY - rect2.height/2);
    int rect2_w = int(rect2.width);
    int rect2_h = int(rect2.height);

    cv::Rect rect_det(rect2_x1, rect2_y1, rect2_w, rect2_h);
    cv::Mat roi2 = img(rect_det);
    cv::Mat hsvroi2;
    cvtColor(roi2, hsvroi2, CV_BGR2HSV);
    cv::MatND hist2;
    cv::calcHist(&hsvroi2, 1, channels, cv::Mat(), hist2, 2, histSize, ranges, true, false);

    float diff = compareHist(hist1, hist2, CV_COMP_BHATTACHARYYA);
    
    return diff;
}