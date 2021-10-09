
#ifndef _VISUAL_OBJECTDETECTOR_H_
#define _VISUAL_OBJECTDETECTOR_H_

#include <numeric>
#include "opencv2/opencv.hpp"
#include "CreadorMascaras.h"
#include "Hough.h"
#include "LineDetector.h"
#include "ColorImageProcessor.h"
#include "ShapeImageProcessor.h"
#include "PoseEstimator.h"
#include "xmlfilereader.h"



class VisualObjectDetector
{
private:
	ColorImageProcessor colorImageProcessor;
    ShapeImageProcessor shapeImageProcessor;
    PoseEstimator poseEstimator;
	//std::vector<cv::Point2f> corners_2D;
public:
    enum ContoursFilterType
    {
        AREA_FILTER             =1,
        AREA_AND_SHAPE_FILTER   =2,
        MAX_AREA_FILTER         =3,
    }contoursFilterType;

    enum DetectionFromTopMode
    {
        ELLIPSE_DETECTION       =1,
        BLOB_COLOR_DETECTION    =2,
    }detectionFromTopMode;

    void init(std::string configFile);
    bool readConfigs(std::string configFile);
    cv::RotatedRect DetectCylindricalObjectTopView(const Mat &I, const string object_color, const int detection_mode);
    cv::RotatedRect DetectObject(const Mat &I, const string object_color, const string object_type);
    std::vector<cv::RotatedRect> DetectObjects(const Mat &I, const string object_color);
    std::vector<std::vector<cv::Point> > DetectContoursInImage(const Mat &I, const std::string object_color, const int filter_mode);
    void EstimateObjectPose(const RotatedRect rotated_item, const string object_type, cv::Mat &Tvec, cv::Mat &Rvec);
	std::vector<cv::Point2f> DetectDoorCorners(cv::Mat &I);
	std::vector<cv::Point2f> DetectWindowCorners(cv::Mat &I);
    std::vector<cv::Point2f> DetectCornersBasedOnContours(const Mat &I, const Mat &I_binarized);
	cv::Mat DetectCornersBasedOnLines(cv::Mat &I, cv::Mat &I_edges);
	//std::vector<cv::Point2f> getObjectCorners(){return corners_2D;}
    inline cv::Mat getOrientation() const {return poseEstimator.getOrientation();}
    inline cv::Mat getTranslation() const {return poseEstimator.getTranslation();}
    VisualObjectDetector(void);
    ~VisualObjectDetector(void);
};

#endif //_VISUAL_OBJECTDETECTOR_H_
