#include "VisualObjectRecognizer.h"

//#define RECOGNIZE_RED_OBJECTS 1
//#define RECOGNIZE_BLUE_OBJECTS 1
#define CLASSIFY_OBJECTS_BASED_ON_HEIGHT 1
#define IMSHOW_MODE 1
#define DRAW_WITH_TEXT 0


VisualObjectRecognizer::VisualObjectRecognizer(void)
{
    RECOGNIZE_RED_OBJECTS = 0;
    RECOGNIZE_BLUE_OBJECTS = 0;
}


VisualObjectRecognizer::~VisualObjectRecognizer(void)
{
}


void VisualObjectRecognizer::init(const std::string configFile)
{
    readConfigs(configFile);
    visualObjectDetector.init(configFile);
}

bool VisualObjectRecognizer::readConfigs(const string configFile)
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


    readingValue = nodeObjectRecognizer.child_value("recognize_color");
    if(readingValue == "RED")
        RECOGNIZE_RED_OBJECTS = 1;
    else if(readingValue == "BLUE")
        RECOGNIZE_BLUE_OBJECTS = 1;
    else if(readingValue == "REDandBLUE")
    {
        RECOGNIZE_RED_OBJECTS = 1;
        RECOGNIZE_BLUE_OBJECTS = 1;
    }
    else if(readingValue == "BLUEandBLACK")
    {
        RECOGNIZE_BLUE_OBJECTS = 1;
        RECOGNIZE_BLACK_OBJECTS = 1;
    }




    return true;
}

void VisualObjectRecognizer::RecognizeObjects(Mat &I, std::vector<RotatedRect> &rotated_rect_objects_detected, std::vector<int> &rotated_rect_objects_ids,
                                              std::vector<Mat> &vector_Tmat, std::vector<Mat> &vector_Rmat)
{
    cv::Mat I_detection;
    I.copyTo(I_detection);

    //    rotated_rect_objects_detected = visualObjectDetector.DetectObjects(I_detection, "White");


    //******** Recognize Stage (DETECTION + CLASSIFICATION) ********
    if(RECOGNIZE_RED_OBJECTS)
    {
        std::vector<cv::RotatedRect> ROIs_red = visualObjectDetector.DetectObjects(I_detection, "Red");
        std::vector<int> ids_ROIs_red = visualObjectClassifier.ClassifyROIs(ROIs_red, "Red");

        rotated_rect_objects_ids.insert(rotated_rect_objects_ids.end(), ids_ROIs_red.begin(), ids_ROIs_red.end());
        rotated_rect_objects_detected.insert(rotated_rect_objects_detected.end(), ROIs_red.begin(), ROIs_red.end());
    }

    if(RECOGNIZE_BLUE_OBJECTS)
    {
        std::vector<cv::RotatedRect> ROIs_blue = visualObjectDetector.DetectObjects(I_detection, "Blue");
        std::vector<int> ids_ROIs_blue = visualObjectClassifier.ClassifyROIs(ROIs_blue, "Blue");

        rotated_rect_objects_ids.insert(rotated_rect_objects_ids.end(), ids_ROIs_blue.begin(), ids_ROIs_blue.end());
        rotated_rect_objects_detected.insert(rotated_rect_objects_detected.end(), ROIs_blue.begin(), ROIs_blue.end());
    }

    if(RECOGNIZE_BLACK_OBJECTS)
    {
        std::vector<cv::RotatedRect> ROIs_black = visualObjectDetector.DetectObjects(I_detection, "Black");
        std::vector<int> ids_ROIs_black = visualObjectClassifier.ClassifyROIs(ROIs_black, "Black");

        rotated_rect_objects_ids.insert(rotated_rect_objects_ids.end(), ids_ROIs_black.begin(), ids_ROIs_black.end());
        rotated_rect_objects_detected.insert(rotated_rect_objects_detected.end(), ROIs_black.begin(), ROIs_black.end());
    }



    //    //******** Pose Estimation for every Recognized Object ********
    //    for(int i=0;i<rotated_rect_objects_detected.size();i++)
    //    {
    //        cv::Mat Tmat, Rvec, Rmat;
    //        std::string object_name;
    //        if((rotated_rect_objects_ids[i] == 0)||(rotated_rect_objects_ids[i] == 2))
    //            object_name = "Item";
    //        else
    //            object_name = "Bucket";
    //        visualObjectDetector.EstimateObjectPose(rotated_rect_objects_detected[i], object_name, Tmat, Rvec);
    //        cv::Rodrigues(Rvec, Rmat);
    //        vector_Tmat.push_back(Tmat.clone());
    //        vector_Rmat.push_back(Rmat.clone());
    //    }


    char corner_id[10];
    cv::Scalar text_color = cv::Scalar(0, 0, 0);
    int font_type = cv::FONT_HERSHEY_SIMPLEX;
    int font_thickness = 2;
    double font_height = 0.95;
    //    cv::Scalar font_text_color1_bucketA = cv::Scalar(0, 0, 255);
    //    cv::Scalar font_text_color1_bucketB = cv::Scalar(255, 0, 0);
    cv::Scalar font_text_color1_bucketA = cv::Scalar(0, 255, 255);
    cv::Scalar font_text_color1_bucketB = cv::Scalar(0, 255, 255);
    cv::Scalar font_text_color2 = cv::Scalar(0, 255, 255);
    //cv::Scalar font_text_color2 = cv::Scalar(255, 255, 255);
    int pos_text_ref_x = 10;
    int pos_text_ref_y = 50;
    if(IMSHOW_MODE)
    {
        //********** VISUALIZATION Of the Detections and Classifications of Objects **********
        for(int i = 0;i<rotated_rect_objects_detected.size();i++)
        {
            cv::Rect bounding_rect = rotated_rect_objects_detected[i].boundingRect();
            std::vector<cv::Point2f> item_corners_2D;
            cv::Point2f corner_points[4];
            rotated_rect_objects_detected[i].points(corner_points);
            for( int j = 0; j < 4; j++)
            {
                cv::line(I, corner_points[j], corner_points[(j+1)%4], cv::Scalar(0, 255, 0), 3, 8);
                item_corners_2D.push_back(corner_points[j]);
            }
            if(rotated_rect_objects_ids[i] == 0)
            {
                pos_text_ref_x = bounding_rect.tl().x;
                pos_text_ref_y = bounding_rect.tl().y-100;
                cv::putText(I, "ItemA", cv::Point(bounding_rect.br().x, bounding_rect.br().y), font_type, font_height, text_color, 2);
                if(DRAW_WITH_TEXT)
                {
                    //cv::putText(I, "Pos.ItemA", cv::Point(pos_text_ref_x, pos_text_ref_y-30), font_type, font_height, font_text_color1_bucketA, 2);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(0,0));
                    cv::putText(I, "x: ", cv::Point(pos_text_ref_x, pos_text_ref_y), font_type, font_height, font_text_color1_bucketA, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y), font_type, font_height, font_text_color2, font_thickness);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(1,0));
                    cv::putText(I, "y: ", cv::Point(pos_text_ref_x, pos_text_ref_y+30), font_type, font_height, font_text_color1_bucketA, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y+30), font_type, font_height, font_text_color2, font_thickness);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(2,0));
                    cv::putText(I, "z: ", cv::Point(pos_text_ref_x, pos_text_ref_y+60), font_type, font_height, font_text_color1_bucketA, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y+60), font_type, font_height, font_text_color2, font_thickness);
                }
            }
            else if(rotated_rect_objects_ids[i] == 1)
            {
                //pos_text_ref_x = corner_points[0].x;
                //pos_text_ref_y = corner_points[0].y -160;
                pos_text_ref_x = bounding_rect.tl().x;
                pos_text_ref_y = bounding_rect.tl().y-100;
                cv::putText(I, "BucketA", cv::Point(bounding_rect.br().x, bounding_rect.br().y), font_type, font_height, text_color, 2);
                if(DRAW_WITH_TEXT)
                {
                    //cv::putText(I, "Pos.BucketA", cv::Point(pos_text_ref_x, pos_text_ref_y-30), font_type, font_height, font_text_color1_bucketA, 2);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(0,0));
                    cv::putText(I, "x: ", cv::Point(pos_text_ref_x, pos_text_ref_y), font_type, font_height, font_text_color1_bucketA, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y), font_type, font_height, font_text_color2, font_thickness);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(1,0));
                    cv::putText(I, "y: ", cv::Point(pos_text_ref_x, pos_text_ref_y+30), font_type, font_height, font_text_color1_bucketA, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y+30), font_type, font_height, font_text_color2, font_thickness);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(2,0));
                    cv::putText(I, "z: ", cv::Point(pos_text_ref_x, pos_text_ref_y+60), font_type, font_height, font_text_color1_bucketA, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y+60), font_type, font_height, font_text_color2, font_thickness);
                }
            }
            else if(rotated_rect_objects_ids[i] == 2)
            {
                pos_text_ref_x = bounding_rect.tl().x;
                pos_text_ref_y = bounding_rect.tl().y-100;
                cv::putText(I, "ItemB", cv::Point(bounding_rect.br().x, bounding_rect.br().y), font_type, font_height, text_color, 2);
                if(DRAW_WITH_TEXT)
                {
                    //cv::putText(I, "Pos.ItemB", cv::Point(pos_text_ref_x, pos_text_ref_y-30), font_type, font_height, font_text_color1_bucketB, 2);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(0,0));
                    cv::putText(I, "x: ", cv::Point(pos_text_ref_x, pos_text_ref_y), font_type, font_height, font_text_color1_bucketB, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y), font_type, font_height, font_text_color2, font_thickness);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(1,0));
                    cv::putText(I, "y: ", cv::Point(pos_text_ref_x, pos_text_ref_y+30), font_type, font_height, font_text_color1_bucketB, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y+30), font_type, font_height, font_text_color2, font_thickness);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(2,0));
                    cv::putText(I, "z: ", cv::Point(pos_text_ref_x, pos_text_ref_y+60), font_type, font_height, font_text_color1_bucketB, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y+60), font_type, font_height, font_text_color2, font_thickness);
                }
            }
            else if(rotated_rect_objects_ids[i] == 3)
            {
                pos_text_ref_x = bounding_rect.tl().x;
                pos_text_ref_y = bounding_rect.tl().y-100;
                cv::putText(I, "BucketB", cv::Point(bounding_rect.br().x, bounding_rect.br().y), font_type, font_height, text_color, 2);
                if(DRAW_WITH_TEXT)
                {
                    //cv::putText(I, "Pos.BucketB", cv::Point(pos_text_ref_x, pos_text_ref_y-30), font_type, font_height, font_text_color1_bucketB, 2);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(0,0));
                    cv::putText(I, "x: ", cv::Point(pos_text_ref_x, pos_text_ref_y), font_type, font_height, font_text_color1_bucketB, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y), font_type, font_height, font_text_color2, font_thickness);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(1,0));
                    cv::putText(I, "y: ", cv::Point(pos_text_ref_x, pos_text_ref_y+30), font_type, font_height, font_text_color1_bucketB, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y+30), font_type, font_height, font_text_color2, font_thickness);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(2,0));
                    cv::putText(I, "z: ", cv::Point(pos_text_ref_x, pos_text_ref_y+60), font_type, font_height, font_text_color1_bucketB, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y+60), font_type, font_height, font_text_color2, font_thickness);
                }

            }
            else if(rotated_rect_objects_ids[i] == 4)
            {
                pos_text_ref_x = bounding_rect.tl().x;
                pos_text_ref_y = bounding_rect.tl().y-100;
                cv::putText(I, "BlackBucket", cv::Point(bounding_rect.br().x, bounding_rect.br().y), font_type, font_height, text_color, 2);
                if(DRAW_WITH_TEXT)
                {
                    //cv::putText(I, "Pos.BucketB", cv::Point(pos_text_ref_x, pos_text_ref_y-30), font_type, font_height, font_text_color1_bucketB, 2);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(0,0));
                    cv::putText(I, "x: ", cv::Point(pos_text_ref_x, pos_text_ref_y), font_type, font_height, font_text_color1_bucketB, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y), font_type, font_height, font_text_color2, font_thickness);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(1,0));
                    cv::putText(I, "y: ", cv::Point(pos_text_ref_x, pos_text_ref_y+30), font_type, font_height, font_text_color1_bucketB, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y+30), font_type, font_height, font_text_color2, font_thickness);
                    sprintf(corner_id,"%.3f",vector_Tmat[i].at<double>(2,0));
                    cv::putText(I, "z: ", cv::Point(pos_text_ref_x, pos_text_ref_y+60), font_type, font_height, font_text_color1_bucketB, font_thickness);
                    cv::putText(I, corner_id, cv::Point(pos_text_ref_x+30, pos_text_ref_y+60), font_type, font_height, font_text_color2, font_thickness);
                }

            }

        }
    }

}

void VisualObjectRecognizer::RecognizeCylindricalObjectTopView(Mat &I, std::vector<RotatedRect> &rotated_rect_objects_detected, std::string object_type,
                                                               std::vector<Mat> &vector_Tmat, std::vector<Mat> &vector_Rmat)
{
    cv::RotatedRect ROI_red;
    cv::RotatedRect ROI_blue;

    ROI_blue = visualObjectDetector.DetectCylindricalObjectTopView(I, object_type, VisualObjectDetector::DetectionFromTopMode::BLOB_COLOR_DETECTION);

    if(ROI_red.size.width > 0 && ROI_red.size.height > 0)
        rotated_rect_objects_detected.push_back(ROI_red);
    if(ROI_blue.size.width > 0 && ROI_blue.size.height > 0)
        rotated_rect_objects_detected.push_back(ROI_blue);

    //******** Pose Estimation for the Recognized Object ********
    for(unsigned int i=0;i<rotated_rect_objects_detected.size();i++)
    {
        cv::Mat Tmat, Rvec, Rmat;
        //visualObjectDetector.EstimateObjectPose(rotated_rect_objects_detected[i], "Bucket", Tmat, Rvec);
        //cv::Rodrigues(Rvec, Rmat);
        //vector_Tmat.push_back(Tmat.clone());
        //vector_Rmat.push_back(Rmat.clone());
        //std::cout<<"Translation Matrix: "<<std::endl<<Tmat<<std::endl<<std::endl;
        //std::cout<<"Rotation Matrix: "<<std::endl<<Rvec<<std::endl<<std::endl;
    }


    if(IMSHOW_MODE)
    {
        //********** VISUALIZATION Of the Detections and Classifications of Objects **********
        for(int i = 0;i<rotated_rect_objects_detected.size();i++)
        {
            cv::Point2f corner_points[4];
            rotated_rect_objects_detected[i].points(corner_points);
            for(unsigned int j = 0; j < 4; j++)
            {
                cv::line(I, corner_points[j], corner_points[(j+1)%4], cv::Scalar(0, 255, 0), 5, 8);
            }

        }
    }
}


