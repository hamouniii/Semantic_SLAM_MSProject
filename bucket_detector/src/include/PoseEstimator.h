#ifndef _POSEESTIMATOR_H_
#define _POSEESTIMATOR_H_

#include "opencv2/opencv.hpp"
//#include "opencv2/nonfree/features2d.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
#include <stdio.h>
#include "xmlfilereader.h"


struct connections{
  cv::Point2f o;
  cv::Point2f d0;
  cv::Point2f d1;
  float angle;
  bool connected;
};

class PoseEstimator
{
private:
	std::vector<cv::Point3f> pose;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Mat Rvec;
	cv::Mat Tvec;
public:
    void init(std::string configFile);
    bool readConfigs(std::string configFile);
	void EstimatePoseFromPNP(std::vector<cv::Point3f> points_3D, std::vector<cv::Point2f> points_2D);
	std::vector<cv::Point2f> matchOBjectToImageRansac(std::vector<cv::Point2f> image_not_ordered);
	void ReadCameraParameters(int parameters_type);
	void Project3Dpoints(cv::Mat &I, std::string image_name, std::vector<cv::Point3f> &points_3D_for_project, cv::Mat &rotMat, cv::Mat &transMat);
    inline std::vector<cv::Point3f> getPose(){return pose;}
    inline cv::Mat getOrientation() const {return Rvec;}
    inline cv::Mat getTranslation() const {return Tvec;}
	PoseEstimator(void);
	~PoseEstimator(void);
};

#endif

