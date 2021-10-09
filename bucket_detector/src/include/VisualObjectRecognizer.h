#ifndef _VISUAL_OBJECTRECOGNIZER_H
#define _VISUAL_OBJECTRECOGNIZER_H

#include <numeric>
#include "opencv2/opencv.hpp"
#include "pugixml.hpp"
#include "VisualObjectDetector.h"
#include "VisualObjectClassifier.h"


class VisualObjectRecognizer
{
private:
    VisualObjectDetector visualObjectDetector;
    VisualObjectClassifier visualObjectClassifier;
    //int32_t perception_manager_mission_state;
    bool RECOGNIZE_RED_OBJECTS, RECOGNIZE_BLUE_OBJECTS, RECOGNIZE_BLACK_OBJECTS;
public:
    void init(const std::string configFile);
    bool readConfigs(const std::string configFile);
    void RecognizeObjects(Mat &I, std::vector<cv::RotatedRect> &rotated_rect_objects_detected, std::vector<int> &rotated_rect_objects_ids,
                         std::vector<cv::Mat> &vector_Tmat, std::vector<cv::Mat> &vector_Rmat);
    void RecognizeCylindricalObjectTopView(Mat &I, std::vector<RotatedRect> &rotated_rect_objects_detected, std::string object_type,
                                           std::vector<Mat> &vector_Tmat, std::vector<Mat> &vector_Rmat);
    //void setPerceptionManagerMissionState(int32_t mission_state){perception_manager_mission_state = mission_state;}
    VisualObjectRecognizer(void);
    ~VisualObjectRecognizer(void);
};


#endif // VISUAL_OBJECTRECOGNIZER_H
