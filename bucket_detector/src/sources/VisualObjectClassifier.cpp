#include "VisualObjectClassifier.h"


VisualObjectClassifier::VisualObjectClassifier(void)
{
    detection_counter = 0;
    no_detection_counter = 0;
    width_height_ratio_detection_threshold = 1.07;
}


VisualObjectClassifier::~VisualObjectClassifier(void)
{
}



std::vector<int> VisualObjectClassifier::ClassifyROIs(std::vector<RotatedRect> &rotated_rect_ROIs, std::string color)
{
    std::vector<int> objects_detected_ids;

    if(rotated_rect_ROIs.size())
    {
        no_detection_counter = 0;
        detection_counter++;
    }
    else
    {
        detection_counter = 0;
        no_detection_counter++;
        if(no_detection_counter > 50)
            average_width_height_object_ratio.clear();
    }

    for(int i=0; i<rotated_rect_ROIs.size();i++)
    {
        cv::RotatedRect rot_box = rotated_rect_ROIs[i];
        float rot_item_width = 0.0;
        float rot_item_height = 0.0;
        if(rot_box.size.width <= rot_box.size.height)
        {
            rot_item_width = (float)rot_box.size.width;
            rot_item_height = (float)rot_box.size.height;
        }
        else
        {
            rot_item_width = (float)rot_box.size.height;
            rot_item_height = (float)rot_box.size.width;
        }
        float rot_box_width_height_ratio = rot_item_width/rot_item_height;

        average_width_height_object_ratio.push_back(rot_box_width_height_ratio);
        float sum = std::accumulate(average_width_height_object_ratio.begin(), average_width_height_object_ratio.end(), 0.0);
        float mean_aspect_ratio = sum / average_width_height_object_ratio.size();
        //cout<<"Mean Ratio (Width/Height) of accumulated detections: "<<mean_aspect_ratio<<endl;


        if((mean_aspect_ratio > width_height_ratio_detection_threshold) && (color == "Red"))
        {
            objects_detected_ids.push_back(0); //ItemA
        }

        else if((mean_aspect_ratio <= width_height_ratio_detection_threshold) && (color == "Red"))
        {
            objects_detected_ids.push_back(1); //BucketA
        }

        else if((mean_aspect_ratio > width_height_ratio_detection_threshold) && (color == "Blue"))
        {
            objects_detected_ids.push_back(2); //ItemB
        }

        else if((mean_aspect_ratio <= width_height_ratio_detection_threshold) && (color == "Blue"))
        {
            objects_detected_ids.push_back(3); //BucketB
        }
        else if((mean_aspect_ratio <= width_height_ratio_detection_threshold) && (color == "Black"))
        {
            objects_detected_ids.push_back(4); //BlackBucket
        }


//        if((rot_box_width_height_ratio > width_height_ratio_detection_threshold) && (color == "Red"))
//            objects_detected_ids.push_back(0); //ItemA

//        if((rot_box_width_height_ratio < width_height_ratio_detection_threshold) && (color == "Red"))
//            objects_detected_ids.push_back(1); //BucketA

//        if((rot_box_width_height_ratio > width_height_ratio_detection_threshold) && (color == "Blue"))
//            objects_detected_ids.push_back(2); //ItemB

//        if((rot_box_width_height_ratio < width_height_ratio_detection_threshold) && (color == "Blue"))
//            objects_detected_ids.push_back(3); //BucketB

    }

    return objects_detected_ids;

}

void VisualObjectClassifier::ClassifyROIsBasedOnPose(Mat &I, std::vector<RotatedRect> &rotated_rect_objects_detected, std::vector<int> &rotated_rect_objects_ids, std::vector<Mat> &vector_Tmat, std::vector<Mat> &vector_Rmat)
{

   //TODO: Project 3D object points to the Image plane and check the detection
}


