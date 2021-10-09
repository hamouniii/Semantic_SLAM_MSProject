
#ifndef _COLORIMAGEPROCESSOR_H_
#define _COLORIMAGEPROCESSOR_H_

#include "opencv2/opencv.hpp"
//#include "opencv2/nonfree/features2d.hpp"

class ColorImageProcessor
{
private:
    cv::Mat I_definitive_mask;
public:
    cv::Mat ProcessRedColor(const cv::Mat &I);
    cv::Mat ProcessBlueColor(const cv::Mat &I);
    cv::Mat ProcessWhiteColor(const cv::Mat &I);
    cv::Mat ProcessBlackColor(const cv::Mat &I);
    cv::Mat ProcessInOpponentColorSpace(const cv::Mat &I);
    cv::Mat ProcessInRGBColorSpace(const cv::Mat &I);
    cv::Mat ProcessInLabColorSpace(const cv::Mat &I);
    cv::Mat ProcessInHLSColorSpace(const cv::Mat &I);
    inline cv::Mat getProcessedMask() const {return I_definitive_mask;}

	ColorImageProcessor(void);
	~ColorImageProcessor(void);
};

#endif

