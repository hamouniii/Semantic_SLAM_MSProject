#include "ColorImageProcessor.h"

#define IMSHOW_MODE 0
#define COLOR_SPACE_CONSENSUS 1
#define IMAV_CONFIGURATION_BEIJING 0

ColorImageProcessor::ColorImageProcessor(void)
{
}


cv::Mat ColorImageProcessor::ProcessInOpponentColorSpace(const cv::Mat &I)
{
    std::vector<cv::Mat> canales_rgb;
    std::vector<cv::Mat> canales_op;
    cv::Mat I_O1_thresholded, I_O2_thresholded, I_O3_thresholded;

    cv::split(I, canales_rgb);
    cv::Mat I_B = canales_rgb[0];
    cv::Mat I_G = canales_rgb[1];
    cv::Mat I_R = canales_rgb[2];
    cv::Mat I_OP_result;

    cv::Mat I_O1 = (I_R - I_G)/(double)cv::sqrt(2.0);
    cv::Mat I_O2 = (I_R + I_G - 2*I_B)/(double)cv::sqrt(6.0);
    cv::Mat I_O3 = (I_R + I_G + I_B)/(double)cv::sqrt(3.0);
    cv::Mat I_O4 = (I_B - I_G)/(double)cv::sqrt(2.0);
    canales_op.push_back(I_O1);
    canales_op.push_back(I_O2);
    canales_op.push_back(I_O3);
    //imshow("I_O1", I_O1);
    //imshow("I_O2", I_O2);
    //imshow("I_O3", I_O3);
    //imshow("I_O4", I_O4);

    cv::threshold(I_O1, I_O1_thresholded, 20, 255, cv::THRESH_BINARY_INV);
    cv::threshold(I_O2, I_O2_thresholded, 50, 255, cv::THRESH_BINARY_INV);
    cv::threshold(I_O3, I_O3_thresholded, 20, 255, cv::THRESH_BINARY_INV);
    //cv::imshow("I channel O1 (Lab) Thresholded", I_O1_thresholded);
    //cv::imshow("I channel O2 (Lab) Thresholded", I_O2_thresholded);
    //cv::imshow("I channel O3 (Lab) Thresholded", I_O3_thresholded);


    cv::Mat I_op;
    cv::merge(canales_op, I_op);
    return I_OP_result;
}


cv::Mat ColorImageProcessor::ProcessRedColor(const cv::Mat &I)
{
    //	std::vector<cv::Mat> canales_rgb;

    //	cv::split(I, canales_rgb);
    //	cv::Mat I_B = canales_rgb[0];
    //	cv::Mat I_G = canales_rgb[1];
    //	cv::Mat I_R = canales_rgb[2];
    cv::Mat I_gray;
    cv::cvtColor(I, I_gray, CV_BGR2GRAY);


    cv::Mat3b I_hsv;
    cv::cvtColor(I, I_hsv, CV_BGR2HSV);
    cv::Mat1b mask1_hsv, mask2_hsv;
    //Simulation

    cv::inRange(I_hsv, cv::Scalar(0, 70, 20), cv::Scalar(10, 255, 255), mask1_hsv);
    cv::inRange(I_hsv, cv::Scalar(170, 70, 20), cv::Scalar(180, 255, 255), mask2_hsv);

    //Real
    //    cv::inRange(I_hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1_hsv);
    //    cv::inRange(I_hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask2_hsv);
    cv::Mat1b I_mask_hsv = mask1_hsv | mask2_hsv;



    cv::Mat3b I_hls;
    cv::cvtColor(I, I_hls, CV_BGR2HLS);
    cv::Mat1b mask1_hls, mask2_hls;
    //Simulation
    cv::inRange(I_hls, cv::Scalar(0, 20, 70), cv::Scalar(10, 200, 255), mask1_hls);
    cv::inRange(I_hls, cv::Scalar(170, 20, 70), cv::Scalar(180, 200, 255), mask2_hls);
    //Real
    //    cv::inRange(I_hls, cv::Scalar(0, 50, 70), cv::Scalar(10, 200, 255), mask1_hls);
    //    cv::inRange(I_hls, cv::Scalar(170, 50, 70), cv::Scalar(180, 200, 255), mask2_hls);
    cv::Mat1b I_mask_hls = mask1_hls | mask2_hls;



    cv::Mat3b I_Lab;
    cv::cvtColor(I, I_Lab, CV_BGR2Lab);
    cv::Mat1b I_mask_lab;
    //cv::inRange(I_Lab, cv::Scalar(20, 160, 100), cv::Scalar(220, 255, 255), I_mask_lab); //Simulation
    cv::inRange(I_Lab, cv::Scalar(20, 140, 90), cv::Scalar(220, 255, 255), I_mask_lab); //Real


    cv::bitwise_and(I_mask_hsv, I_mask_hls, I_definitive_mask);



    if(IMSHOW_MODE)
    {
        cv::imshow("I Original", I);
        cv::imshow("I HSV for RED Thresholded", I_mask_hsv);
        cv::imshow("I HLS for RED Thresholded", I_mask_hls);
        cv::imshow("I Lab for RED Thresholded", I_mask_lab);
        //cv::imshow("I channel O1 (Opponent) Thresholded", I_O1_thresholded);
        //cv::imshow("I channel H MASK", I_mask_hsv);
        cv::imshow("DEFINITIVE MASK RED", I_definitive_mask);
        //cv::imshow("I_canny",I_canny);
        //cv::imshow("I_canny OBJECT", I_canny_object);
        cv::waitKey(1);
    }


    return I_definitive_mask;




    if(IMAV_CONFIGURATION_BEIJING)
    {

        //////////////////////////////
        /// Configuration for IMAV Beijing
        ///////////////////////
        cv::Mat I_gray;
        cv::cvtColor(I, I_gray, CV_BGR2GRAY);


        cv::Mat3b I_hsv;
        cv::cvtColor(I, I_hsv, CV_BGR2HSV);
        cv::Mat1b mask1_hsv, mask2_hsv;
        //Simulation
        cv::inRange(I_hsv, cv::Scalar(0, 70, 15), cv::Scalar(10, 255, 200), mask1_hsv);
        cv::inRange(I_hsv, cv::Scalar(170, 70, 15), cv::Scalar(180, 255, 200), mask2_hsv);
        //Real
        //    cv::inRange(I_hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1_hsv);
        //    cv::inRange(I_hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask2_hsv);
        cv::Mat1b I_mask_hsv = mask1_hsv | mask2_hsv;



        cv::Mat3b I_hls;
        cv::cvtColor(I, I_hls, CV_BGR2HLS);
        cv::Mat1b mask1_hls, mask2_hls;
        //Simulation
        cv::inRange(I_hls, cv::Scalar(0, 20, 70), cv::Scalar(10, 200, 255), mask1_hls);
        cv::inRange(I_hls, cv::Scalar(170, 20, 70), cv::Scalar(180, 200, 255), mask2_hls);
        //Real
        //    cv::inRange(I_hls, cv::Scalar(0, 50, 70), cv::Scalar(10, 200, 255), mask1_hls);
        //    cv::inRange(I_hls, cv::Scalar(170, 50, 70), cv::Scalar(180, 200, 255), mask2_hls);
        cv::Mat1b I_mask_hls = mask1_hls | mask2_hls;



        cv::Mat3b I_Lab;
        cv::cvtColor(I, I_Lab, CV_BGR2Lab);
        cv::Mat1b I_mask_lab;
        //cv::inRange(I_Lab, cv::Scalar(20, 160, 100), cv::Scalar(220, 255, 255), I_mask_lab); //Simulation
        cv::inRange(I_Lab, cv::Scalar(20, 140, 90), cv::Scalar(220, 255, 255), I_mask_lab); //Real


        cv::bitwise_and(I_mask_hsv, I_mask_hls, I_definitive_mask);



        if(IMSHOW_MODE)
        {
            cv::imshow("I Original", I);
            cv::imshow("I HSV for RED Thresholded", I_mask_hsv);
            cv::imshow("I HLS for RED Thresholded", I_mask_hls);
            cv::imshow("I Lab for RED Thresholded", I_mask_lab);
            //cv::imshow("I channel O1 (Opponent) Thresholded", I_O1_thresholded);
            //cv::imshow("I channel H MASK", I_mask_hsv);
            cv::imshow("DEFINITIVE MASK RED", I_definitive_mask);
            //cv::imshow("I_canny",I_canny);
            //cv::imshow("I_canny OBJECT", I_canny_object);
            cv::waitKey(1);
        }


        return I_definitive_mask;
    }

}

cv::Mat ColorImageProcessor::ProcessBlueColor(const cv::Mat &I)
{
    //	std::vector<cv::Mat> canales_rgb;

    //	cv::split(I, canales_rgb);
    //	cv::Mat I_B = canales_rgb[0];
    //	cv::Mat I_G = canales_rgb[1];
    //	cv::Mat I_R = canales_rgb[2];


    cv::Mat3b I_hsv;
    cv::cvtColor(I, I_hsv, CV_BGR2HSV);
    cv::Mat1b I_mask_hsv;
    //cv::inRange(I_hsv, cv::Scalar(110, 70, 50), cv::Scalar(130, 255, 255), I_mask_hsv); //Simulation
    cv::inRange(I_hsv, cv::Scalar(90, 70, 50), cv::Scalar(140, 255, 255), I_mask_hsv); //Real
    //cv::inRange(I_hsv, cv::Scalar(90, 100, 50), cv::Scalar(140, 255, 255), I_mask_hsv); //Real


    cv::Mat3b I_hls;
    cv::cvtColor(I, I_hls, CV_BGR2HLS);
    cv::Mat1b I_mask_hls;
    cv::inRange(I_hls, cv::Scalar(80, 50, 50), cv::Scalar(150, 255, 255), I_mask_hls);


    cv::Mat3b I_Lab;
    cv::cvtColor(I, I_Lab, CV_BGR2Lab);
    cv::Mat1b I_mask_lab;
    cv::inRange(I_Lab, cv::Scalar(0, 20, 0), cv::Scalar(200, 220, 115), I_mask_lab);
    //cv::inRange(I_Lab, cv::Scalar(20, 20, 0), cv::Scalar(200, 200, 115), I_mask_lab);



    cv::bitwise_and(I_mask_hsv, I_mask_hls, I_definitive_mask);
    //cv::bitwise_and(I_mask_hsv, I_mask_lab, I_definitive_mask);

    if(IMSHOW_MODE)
    {
        //std::cout<<"Image Size: "<<"["<<I.rows<<" ; "<<I.cols<<"]"<<std::endl;
        cv::imshow("I Original", I);
        cv::imshow("I HSV for BLUE Thresholded", I_mask_hsv);
        cv::imshow("I HLS for BLUE Thresholded", I_mask_hls);
        cv::imshow("I Lab for BLUE Thresholded", I_mask_lab);
        cv::imshow("DEFINITIVE MASK BLUE", I_definitive_mask);
        cv::waitKey(1);
    }

    //return I_definitive_mask;
    return I_mask_hsv;





    if(IMAV_CONFIGURATION_BEIJING)
    {
        //////////////////////////////
        /// Configuration for IMAV Beijing
        ///////////////////////
        cv::Mat3b I_hsv;
        cv::cvtColor(I, I_hsv, CV_BGR2HSV);
        cv::Mat1b I_mask_hsv;
        //cv::inRange(I_hsv, cv::Scalar(110, 70, 50), cv::Scalar(130, 255, 255), I_mask_hsv); //Simulation
        //cv::inRange(I_hsv, cv::Scalar(90, 70, 50), cv::Scalar(140, 255, 255), I_mask_hsv); //Real
        cv::inRange(I_hsv, cv::Scalar(90, 70, 50), cv::Scalar(140, 255, 255), I_mask_hsv); //Real


        cv::Mat3b I_hls;
        cv::cvtColor(I, I_hls, CV_BGR2HLS);
        cv::Mat1b I_mask_hls;
        cv::inRange(I_hls, cv::Scalar(90, 50, 70), cv::Scalar(150, 200, 255), I_mask_hls);


        cv::Mat3b I_Lab;
        cv::cvtColor(I, I_Lab, CV_BGR2Lab);
        cv::Mat1b I_mask_lab;
        cv::inRange(I_Lab, cv::Scalar(0, 20, 0), cv::Scalar(200, 220, 115), I_mask_lab);
        //cv::inRange(I_Lab, cv::Scalar(20, 20, 0), cv::Scalar(200, 200, 115), I_mask_lab);



        //cv::bitwise_and(I_mask_hsv, I_mask_hls, I_definitive_mask);
        cv::bitwise_and(I_mask_hsv, I_mask_lab, I_definitive_mask);

        if(IMSHOW_MODE)
        {
            //std::cout<<"Image Size: "<<"["<<I.rows<<" ; "<<I.cols<<"]"<<std::endl;
            cv::imshow("I Original", I);
            cv::imshow("I HSV for BLUE Thresholded", I_mask_hsv);
            cv::imshow("I HLS for BLUE Thresholded", I_mask_hls);
            cv::imshow("I Lab for BLUE Thresholded", I_mask_lab);
            cv::imshow("DEFINITIVE MASK BLUE", I_definitive_mask);
            cv::waitKey(1);
        }

        //return I_definitive_mask;
        return I_mask_lab;
    }
}


cv::Mat ColorImageProcessor::ProcessWhiteColor(const cv::Mat &I)
{
    cv::Mat I_gray, I_gray_thresholded;
    cv::cvtColor(I, I_gray, CV_BGR2GRAY);

    cv::threshold(I_gray, I_gray_thresholded, 0, 20, cv::THRESH_BINARY);
    cv::imshow("I_gray", I_gray);
    cv::imshow("I_gray Trhesholded", I_gray_thresholded);


    cv::Mat1b I_mask_rgb;
    cv::inRange(I, cv::Scalar(0, 0, 0), cv::Scalar(20, 20, 20), I_mask_rgb);
    cv::imshow("I RGB for BLACK Thresholded", I_mask_rgb);


    cv::Mat3b I_hsv;
    cv::cvtColor(I, I_hsv, CV_BGR2HSV);
    cv::Mat1b I_mask_hsv;
    cv::inRange(I_hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 30), I_mask_hsv);
    cv::imshow("I HSV for WHITE Thresholded", I_mask_hsv);
    cv::waitKey(1);

    //    cv::Mat3b I_hls;
    //    cv::cvtColor(I, I_hls, CV_BGR2HLS);
    //    cv::Mat1b I_mask_hls;
    //    cv::inRange(I_hls, cv::Scalar(0, 210, 0), cv::Scalar(255, 255, 255), I_mask_hls);
    //    cv::imshow("I HLS for WHITE Thresholded", I_mask_hls);
    //    cv::waitKey(1);

    return I_mask_rgb;
}

cv::Mat ColorImageProcessor::ProcessBlackColor(const cv::Mat &I)
{
    //    cv::Mat I_gray, I_gray_thresholded;
    //    cv::cvtColor(I, I_gray, CV_BGR2GRAY);

    //    cv::erode(I, I, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    //    cv::dilate(I, I, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

    //    //morphological closing fill in small holes of the foreground)
    //    cv::dilate(I, I, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));
    //    cv::erode(I, I, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5)));

    cv::Mat1b I_mask_rgb;
    cv::inRange(I, cv::Scalar(0, 0, 0), cv::Scalar(40, 40, 40), I_mask_rgb);

    cv::Mat3b I_hsv;
    cv::cvtColor(I, I_hsv, CV_BGR2HSV);
    cv::Mat1b I_mask_hsv;
    cv::inRange(I_hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 40), I_mask_hsv);

    cv::Mat3b I_hls;
    cv::cvtColor(I, I_hls, CV_BGR2HLS);
    cv::Mat1b I_mask_hls;
    cv::inRange(I_hls, cv::Scalar(0, 0, 0), cv::Scalar(60, 60, 60), I_mask_hls);


    cv::bitwise_and(I_mask_rgb, I_mask_hsv,I_definitive_mask);
    //cv::bitwise_and(I_mask_hsv, I_mask_lab, I_definitive_mask);

    if(IMSHOW_MODE)
    {
        //std::cout<<"Image Size: "<<"["<<I.rows<<" ; "<<I.cols<<"]"<<std::endl;
        cv::imshow("I Original", I);
        cv::imshow("I HSV for BLACK Thresholded", I_mask_hsv);
        cv::imshow("I rgb for BLACK Thresholded", I_mask_rgb);
        cv::imshow("I hls for BLACK Thresholded", I_mask_hls);
        cv::imshow("DEFINITIVE MASK BLACK", I_definitive_mask);
        cv::waitKey(1);
    }

    //    cv::Mat3b I_hls;
    //    cv::cvtColor(I, I_hls, CV_BGR2HLS);
    //    cv::Mat1b I_mask_hls;
    //    cv::inRange(I_hls, cv::Scalar(0, 210, 0), cv::Scalar(255, 255, 255), I_mask_hls);
    //    cv::imshow("I HLS for WHITE Thresholded", I_mask_hls);
    //    cv::waitKey(1);

    return I_definitive_mask;
}



cv::Mat ColorImageProcessor::ProcessInRGBColorSpace(const cv::Mat &I)
{
    cv::Mat I_hsv;
    cv::Mat I_R_thresholded, I_G_thresholded, I_B_thresholded;
    cv::Mat I_S_thresholded;
    cv::Mat I_and, I_xor_RandB;
    cv::Mat I_G_thresholded_dilated;
    cv::Mat I_color_result;

    cv::cvtColor(I, I_hsv, CV_BGR2HSV);
    std::vector<cv::Mat> canales_rgb;
    std::vector<cv::Mat> canales_hsv;
    cv::split(I, canales_rgb);
    cv::split(I_hsv, canales_hsv);
    cv::Mat I_B = canales_rgb[0];
    cv::Mat I_G = canales_rgb[1];
    cv::Mat I_R = canales_rgb[2];
    cv::Mat I_H = canales_hsv[0];
    cv::Mat I_S = canales_hsv[1];
    cv::Mat I_V = canales_hsv[2];
    //cv::equalizeHist(I_V,I_V_eq);
    //imshow("I_R",I_R);
    //imshow("I_G",I_G);
    //imshow("I_B",I_B);
    //imshow("I_H",I_H);
    //imshow("I_S",I_S);
    //imshow("I_V",I_V);


    cv::threshold(I_R, I_R_thresholded, 20, 255, cv::THRESH_BINARY_INV);
    cv::threshold(I_G, I_G_thresholded, 50, 255, cv::THRESH_BINARY_INV);
    cv::threshold(I_B, I_B_thresholded, 20, 255, cv::THRESH_BINARY_INV);
    cv::threshold(I_S, I_S_thresholded, 200, 255, cv::THRESH_BINARY);
    cv::bitwise_xor(I_R_thresholded, I_B_thresholded, I_xor_RandB);
    cv::bitwise_and(I_S_thresholded, I_xor_RandB, I_color_result);

    cv::Mat element9(3,3,CV_8U,cv::Scalar(1));
    cv::dilate(I_G_thresholded, I_G_thresholded_dilated, element9);
    cv::morphologyEx(I_color_result, I_color_result, cv::MORPH_OPEN, cv::Mat(2,2, CV_8U, cv::Scalar(1)));
    //cv::dilate(I_xor_RandB, I_xor_RandB, element9);

    //cv::imshow("I channel R (RGB) Thresholded", I_R_thresholded);
    //cv::imshow("I channel G (RGB) Thresholded", I_G_thresholded);
    //cv::imshow("I channel B (RGB) Thresholded", I_B_thresholded);
    //cv::imshow("I channel S (HSV) Thresholded", I_S_thresholded);
    //cv::imshow("I AND", I_and);
    //cv::imshow("I XOR channel R and B", I_xor_RandB);
    cv::imshow("I COLOR RESULT", I_color_result);

    return I_color_result;
}

cv::Mat ColorImageProcessor::ProcessInLabColorSpace(const cv::Mat &I)
{
    cv::Mat I_Lab;
    cv::Mat I_L_thresholded, I_a_thresholded, I_b_thresholded;
    cv::Mat I_color_result;

    cv::cvtColor(I, I_Lab, CV_BGR2Lab);
    std::vector<cv::Mat> canales_Lab;
    cv::split(I_Lab, canales_Lab);
    cv::Mat I_L = canales_Lab[0];
    cv::Mat I_a = canales_Lab[1];
    cv::Mat I_b = canales_Lab[2];
    //cv::equalizeHist(I_V,I_V_eq);
    //imshow("I_L (Lab)",I_L);
    //imshow("I_a (Lab)",I_a);
    //imshow("I_b (Lab)",I_b);

    cv::threshold(I_L, I_L_thresholded, 20, 255, cv::THRESH_BINARY_INV);
    cv::threshold(I_a, I_a_thresholded, 50, 255, cv::THRESH_BINARY_INV);
    cv::threshold(I_b, I_b_thresholded, 100, 255, cv::THRESH_BINARY_INV);
    //cv::imshow("I channel L (Lab) Thresholded", I_L_thresholded);
    //cv::imshow("I channel a (Lab) Thresholded", I_a_thresholded);
    //cv::imshow("I channel b (Lab) Thresholded", I_b_thresholded);

    return I_color_result;
}

cv::Mat ColorImageProcessor::ProcessInHLSColorSpace(const cv::Mat &I)
{
    cv::Mat I_hls;
    cv::Mat I_H_thresholded, I_L_thresholded, I_S_thresholded;
    cv::Mat I_color_result;

    cv::cvtColor(I, I_hls, CV_BGR2HSV);
    std::vector<cv::Mat> canales_hls;
    cv::split(I_hls, canales_hls);
    cv::Mat I_H = canales_hls[0];
    cv::Mat I_L = canales_hls[1];
    cv::Mat I_S = canales_hls[2];
    //cv::equalizeHist(I_V,I_V_eq);
    //imshow("I_H (HLS)",I_H);
    //imshow("I_L (HLS)",I_L);
    //imshow("I_S (HLS)",I_S);

    cv::threshold(I_H, I_H_thresholded, 20, 255, cv::THRESH_BINARY_INV);
    cv::threshold(I_L, I_L_thresholded, 50, 255, cv::THRESH_BINARY_INV);
    cv::threshold(I_S, I_S_thresholded, 20, 255, cv::THRESH_BINARY_INV);
    //cv::imshow("I channel H (HLS) Thresholded", I_H_thresholded);
    //cv::imshow("I channel L (HLS) Thresholded", I_L_thresholded);
    //cv::imshow("I channel S (HLS) Thresholded", I_S_thresholded);

    return I_color_result;
}


ColorImageProcessor::~ColorImageProcessor(void)
{
}
