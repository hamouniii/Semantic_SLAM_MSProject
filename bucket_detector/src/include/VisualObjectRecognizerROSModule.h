
#ifndef _OBJECT_DETECTOR_ROS_MODULE_H
#define _OBJECT_DETECTOR_ROS_MODULE_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "VisualObjectDetector.h"
#include "VisualObjectRecognizer.h"

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "opencv_apps/RotatedRect.h"
#include "opencv_apps/RotatedRectStamped.h"
#include "opencv_apps/Rect.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "ShapeColor_ObjectDetection/DetectedObjects.h"
#include "ShapeColor_ObjectDetection/ObjectInfo.h"

class VisualObjectRecognizerROSModule
{
private:
    VisualObjectDetector visualObjectDetector;
    VisualObjectRecognizer visualObjectRecognizer;
    cv_bridge::CvImagePtr cvFrontImage;
    cv_bridge::CvImagePtr cvBottomImage;
    cv::Mat  front_camera_image;
    cv::Mat bottom_camera_image;
    int32_t perception_manager_mission_state;
    int num_images_recorded;

    //Topic Names
    std::string object_recognized_front_camera_topic_name_;
    std::string front_camera_image_topic_name_;
    std::string object_recognized_bbs_;
    std::string config_file_;

    //ROS Publishers
    ros::Publisher visual_object_recognized_front_image_pub;
    ros::Publisher visual_object_recognized_bbs_pub;

    //ROS Subscribers
    ros::Subscriber drone_image_front_camera_subs;

    //ROS Messages
    sensor_msgs::Image image_from_front_camera;


public:
    VisualObjectRecognizerROSModule();
    ~VisualObjectRecognizerROSModule();

    void droneImageFromFrontCameraCallback(const sensor_msgs::Image &msg);
    void Detect(const Mat &I);
    void open(ros::NodeHandle & nIn);
    void init();
    bool ReadConfigs(std::string configFile);
    void ReadParameters();

};


#endif
