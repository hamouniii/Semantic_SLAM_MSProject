//#pragma once
//#include "CreadorMascaras.h"
//#include "Hough.h"

#ifndef _LINEDETECTOR_H_
#define _LINEDETECTOR_H_

#include "CreadorMascaras.h"
#include "Hough.h"
#include <opencv2/opencv.hpp>

class LineDetector
{
private:	
	Hough hough;
	CreadorMascaras maskCreator;

	double range_theta[2];
	int minVotes;
	cv::Mat I_vertical_edges_thr;
	cv::Mat I_vertical_lines;
public:
	std::vector<cv::Point> DetectHoughLines(cv::Mat &frame);
	//std::vector<cv::Point> DetectHoughLines(cv::Mat &frame,double range_theta[],int minVotes);
    std::vector<std::vector<cv::Point> > DetectVerticalContours(cv::Mat &result_contours_columnas_torre);

    void SetMinVote(int minv){minVotes = minv;}
    void SetRangeTheta(double r_theta[]){range_theta[0]=r_theta[0];range_theta[1]=r_theta[1];}
	LineDetector(void);
	~LineDetector(void);
};

#endif

