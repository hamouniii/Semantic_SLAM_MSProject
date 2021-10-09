#include <iostream>
#include <string>
#include <vector>


#include "ros/ros.h"
#include "ObjectDetectorROSModule.h"

using namespace std;



int main(int argc, char **argv)
{
    cv::VideoCapture cap;
    cap.open(1);
    if(!cap.isOpened())
        return -1;

    ros::init(argc, argv, "Object_Detector");
    ros::NodeHandle n;


    ObjectDetectorROSModule MyObjectDetectorROSModule;
    MyObjectDetectorROSModule.open(n);


    while(1)
    {
        cv::Mat I;
        cap>>I;
        if(I.empty())
        {
            printf("!!! cvCaptureFromAVI failed (file not found?)\n");
            return -1;
        }
        MyObjectDetectorROSModule.Detect(I);

//        while(ros::ok())
//        {
//            ros::spin();
//        }
        char tecla = cv::waitKey(10);
        if(tecla==27)
            break;
    }
    return 0;
}





