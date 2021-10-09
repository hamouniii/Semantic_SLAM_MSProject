#include <iostream>
#include <string>
#include <vector>


#include "ros/ros.h"
#include "VisualObjectRecognizerROSModule.h"

using namespace std;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Object_Recognizer");
    ros::NodeHandle nh;


    VisualObjectRecognizerROSModule MyVisualObjectRecognizerROSModule;
    MyVisualObjectRecognizerROSModule.open(nh);

    ros::spin();

    return 0;
}





