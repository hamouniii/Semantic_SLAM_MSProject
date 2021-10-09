#ifndef _HOUGH_H_
#define _HOUGH_H_

#include <opencv2/opencv.hpp>
using namespace cv;

class Hough
{
private:
    std::vector<cv::Vec2f> lines_hough;
	cv::Mat Hough_Space;
	int rho_min,rho_max;
	int theta_min,theta_max;
public:
	Hough(void);
	~Hough(void);
	Mat ComputeHoughSpace(cv::Mat &I_bordes,double range_theta[]);
	std::vector<cv::Point> FindMaximum_InHoughSpace(cv::Mat &E_Hough,int num_picos,int umbral);
	Mat ComputeHoughLines(cv::Mat &I,cv::Mat &E_Hough,std::vector<cv::Point> rho_theta);
};

#endif

