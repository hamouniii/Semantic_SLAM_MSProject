#ifndef _VISUAL_OBJECTCLASSIFIER_H
#define _VISUAL_OBJECTCLASSIFIER_H

#include <numeric>
#include "opencv2/opencv.hpp"
#include "CreadorMascaras.h"
#include "Hough.h"
#include "LineDetector.h"
#include "ColorImageProcessor.h"
#include "ShapeImageProcessor.h"
#include "PoseEstimator.h"
#include "xmlfilereader.h"


class VisualObjectClassifier
{
private:
    std::vector<float> average_width_height_object_ratio;
    unsigned int detection_counter;
    unsigned int no_detection_counter;
    float width_height_ratio_detection_threshold;
public:
    void ClassifyROIsBasedOnPose(Mat &I, std::vector<cv::RotatedRect> &rotated_rect_objects_detected, std::vector<int> &rotated_rect_objects_ids,
                                  std::vector<cv::Mat> &vector_Tmat, std::vector<cv::Mat> &vector_Rmat);
    std::vector<int> ClassifyROIs(std::vector<cv::RotatedRect> &rotated_rect_ROIs, string color);
    //std::vector<int> ClassifyROIs(std::vector<RotatedRect> &rotated_rect_ROIs, std::vector<float> &average_width_height_object_ratio, string color);
    VisualObjectClassifier(void);
    ~VisualObjectClassifier(void);
};


#endif // _VISUAL_OBJECTCLASSIFIER_H
