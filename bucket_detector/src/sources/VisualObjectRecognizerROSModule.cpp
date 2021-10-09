#include "VisualObjectRecognizerROSModule.h"
using namespace std;

VisualObjectRecognizerROSModule::VisualObjectRecognizerROSModule()
{
    num_images_recorded = 0;
}


VisualObjectRecognizerROSModule::~VisualObjectRecognizerROSModule()
{

}

void VisualObjectRecognizerROSModule::init()
{
    ReadParameters();
    ReadConfigs(config_file_);
    visualObjectRecognizer.init(config_file_);
    return;
}

void VisualObjectRecognizerROSModule::open(ros::NodeHandle & nIn)
{
    init();

    //object_type_publ = nIn.advertise<std_msgs::String>("ROIObjectType",1, true);
    visual_object_recognized_front_image_pub = nIn.advertise<sensor_msgs::Image>(object_recognized_front_camera_topic_name_, 1, this);
    visual_object_recognized_bbs_pub         = nIn.advertise<ShapeColor_ObjectDetection::DetectedObjects>(object_recognized_bbs_, 1);

    //Real Flights
    drone_image_front_camera_subs = nIn.subscribe("/camera/color/image_raw", 1, &VisualObjectRecognizerROSModule::droneImageFromFrontCameraCallback, this);

}

void VisualObjectRecognizerROSModule::ReadParameters()
{
    // Topic names
    ros::param::get("~front_camera_image_topic_name", front_camera_image_topic_name_);
    if ( front_camera_image_topic_name_.length() == 0)
    {
        front_camera_image_topic_name_="/camera_cam/image_raw";
    }
    std::cout<<"front_camera_image_topic_name = "<<front_camera_image_topic_name_<<std::endl;

    ros::param::get("~object_recognized_front_camera_topic_name", object_recognized_front_camera_topic_name_);
    if ( object_recognized_front_camera_topic_name_.length() == 0)
    {
        object_recognized_front_camera_topic_name_="/image_processed/object_recognized_front_camera";
    }
    std::cout<<"object_recognized_front_camera_topic_name = "<<object_recognized_front_camera_topic_name_<<std::endl;

    ros::param::get("~object_recognized_bbs", object_recognized_bbs_);
    if ( object_recognized_bbs_.length() == 0)
    {
        object_recognized_bbs_="/image_processed/bounding_boxes";
    }
    std::cout<<"object_recognized_bbs_ = "<<object_recognized_bbs_<<std::endl;

    ros::param::get("~config_file",config_file_);
    std::cout << "config_file " << config_file_ << std::endl;

    return;
}

bool VisualObjectRecognizerROSModule::ReadConfigs(std::string configFile)
{
    pugi::xml_document doc;

    std::ifstream nameFile(configFile.c_str());
    pugi::xml_parse_result result = doc.load(nameFile);

    if(!result)
    {
        cout << "ERROR: Could not load the file: " << result.description() << endl;
        return 0;
    }

    pugi::xml_node Configuration = doc.child("ShapeColor_ObjectDetector_config");
    pugi::xml_node nodeObjectRecognizer = Configuration.child("VisualObjectRecognizer");
    std::string readingValue;

    readingValue = nodeObjectRecognizer.child_value("perception_manager_mission_state");
    perception_manager_mission_state = atoi(readingValue.c_str());
    //cout<<"perception_manager_mission_state: "<<perception_manager_mission_state<<endl;

    return true;
}


void VisualObjectRecognizerROSModule::droneImageFromFrontCameraCallback(const sensor_msgs::Image &msg)
{

    cv_bridge::CvImagePtr cvObject;
    try
    {
        cvFrontImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvObject = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {

        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    front_camera_image = cvFrontImage->image;
    //cv::imshow("Front Camera Image", front_camera_image);
    //cv::waitKey(5);

    cv::Mat I_detection;
    front_camera_image.copyTo(I_detection);

    std::vector<cv::Mat> vector_Tmat;
    std::vector<cv::Mat> vector_Rmat;
    std::vector<cv::RotatedRect> rotated_rect_objects_detected;
    std::vector<int> rotated_rect_objects_ids;
    //ros::Time time_t1 = ros::Time::now();
    visualObjectRecognizer.RecognizeObjects(I_detection, rotated_rect_objects_detected, rotated_rect_objects_ids, vector_Tmat, vector_Rmat);
    //ros::Time time_t2 = ros::Time::now();
    //std::cout << "time taken for detection " << time_t2 - time_t1 << std::endl;

    cvObject->image = I_detection;
    ShapeColor_ObjectDetection::DetectedObjects detected_objs_info;

    //**********   ROS Communication ************
    if(rotated_rect_objects_detected.size() > 0)
    {
        for(int i=0;i< rotated_rect_objects_detected.size();i++)
        {
            std::string object_name;
            if((rotated_rect_objects_ids[i] == 0)||(rotated_rect_objects_ids[i] == 2))
                object_name = "item";
            else
                object_name = "bucket";

            ShapeColor_ObjectDetection::ObjectInfo detected_obj_info;

            detected_obj_info.prob      = 0.989;
            detected_obj_info.type      = object_name;
            detected_obj_info.tl_x      = rotated_rect_objects_detected[i].center.x - rotated_rect_objects_detected[i].size.width/2;
            detected_obj_info.tl_y      = rotated_rect_objects_detected[i].center.y - rotated_rect_objects_detected[i].size.height/2;
            detected_obj_info.width     = rotated_rect_objects_detected[i].size.width;
            detected_obj_info.height    = rotated_rect_objects_detected[i].size.height;

            detected_objs_info.objects.push_back(detected_obj_info);
        }

        visual_object_recognized_bbs_pub.publish(detected_objs_info);
    }

    visual_object_recognized_front_image_pub.publish(cvObject->toImageMsg());

    return;
}


void VisualObjectRecognizerROSModule::Detect(const Mat &I)
{
    cv::Mat I_detection;
    I.copyTo(I_detection);

    cv::RotatedRect rotated_item;
    opencv_apps::RotatedRect item_rotated_msg;

    rotated_item = visualObjectDetector.DetectObject(I_detection, "Red", "Item");
    item_rotated_msg.angle = rotated_item.angle;
    item_rotated_msg.center.x = rotated_item.center.x;
    item_rotated_msg.center.y = rotated_item.center.y;
    item_rotated_msg.size.width = rotated_item.size.width;
    item_rotated_msg.size.height = rotated_item.size.height;

    //object_ROI_publ.publish(item_rotated_msg);

    //std::string type = "Bucket";
    std::string type = "Item";
    std_msgs::String TypeObject_msg;
    TypeObject_msg.data = type;
    //object_type_publ.publish(TypeObject_msg);

}
