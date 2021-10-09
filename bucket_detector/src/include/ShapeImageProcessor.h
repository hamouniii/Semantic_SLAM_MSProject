
#ifndef _SHAPEIMAGEPROCESSOR_H_
#define _SHAPEIMAGEPROCESSOR_H_

#include "opencv2/opencv.hpp"
#include "EllipseDetectorYaed.h"
#include "xmlfilereader.h"

class ShapeImageProcessor
{
private:
    double item_width, item_height;
    double bucket_width, bucket_height;
    CEllipseDetectorYaed ellipseDetector;
public:
    std::vector<cv::Point3f> get3DObjectCorners(std::string object_name);
    bool readConfigs(std::string configFile);
    cv::Mat detectEdges(cv::Mat &I);
    std::vector<double> computeHuMoments(std::vector<cv::Point> &contour);
    cv::Mat detectRectangles(cv::Mat &I);
    void findRectangles(const cv::Mat &image, std::vector<std::vector<cv::Point> >& rectangles);
    cv::Mat drawRectanglesMask(cv::Mat &image, const std::vector<std::vector<cv::Point> >& rectangles);
    cv::Mat drawRectanglesContour(cv::Mat &image, const std::vector<std::vector<cv::Point> >& rectangles);
    double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
    void setEllipseDetectorParameter(cv::Mat &I);
    cv::RotatedRect detectEllipseInObject(const cv::Mat &I, cv::RotatedRect rot_rect);
	ShapeImageProcessor(void);
	~ShapeImageProcessor(void);
};

#endif

