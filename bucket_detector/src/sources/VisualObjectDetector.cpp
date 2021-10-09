#include "VisualObjectDetector.h"

#define IMSHOW_MODE 0

VisualObjectDetector::VisualObjectDetector(void)
{
}


VisualObjectDetector::~VisualObjectDetector(void)
{
}

void VisualObjectDetector::init(string configFile)
{
    poseEstimator.readConfigs(configFile);
    shapeImageProcessor.readConfigs(configFile);
    //readConfigs(configFile);
}

bool VisualObjectDetector::readConfigs(std::string configFile)
{

}

std::vector<cv::Point2f> VisualObjectDetector::DetectCornersBasedOnContours(const cv::Mat &I, const cv::Mat &I_binarized)
{
    //Search for CONTOURS
    cv::Mat I_contours, I_ROI_boxes, I_ROIs_rotated, I_ROIs_corners, I_thresh;
    I.copyTo(I_contours);
    I.copyTo(I_ROI_boxes);
    I.copyTo(I_ROIs_rotated);
    I.copyTo(I_ROIs_corners);
    I_binarized.copyTo(I_thresh);
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Point2f> corners;

    //cv::findContours(I_color_result, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    cv::findContours(I_thresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);


    std::vector<std::vector<cv::Point> >::iterator itc = contours.begin();
    std::vector<cv::Rect> box_contours(contours.size());
    int size_MaxContour=0;
    //cout<<"**** Area de los Contornos ****"<<endl;
    for(int i=0;i<contours.size();i++)
    {
        box_contours[i] = cv::boundingRect(cv::Mat(contours[i]));
        if(box_contours[i].area()>size_MaxContour)
            size_MaxContour = box_contours[i].area();
        //cout<<"contorno i: "<<box_contours[i].area()<<endl;
    }
    //cout<<"contorno maximo:"<<endl<<size_MaxContour<<endl;

    itc = contours.begin();
    while(itc!=contours.end())
    {
        Rect box = cv::boundingRect(cv::Mat(*itc));
        if(box.area()<size_MaxContour/4)
            itc = contours.erase(itc);
        else
            ++itc;
    }


    vector<RotatedRect> minRect(contours.size());
    box_contours.clear();
    int offset = 10;
    for(int i=0;i<contours.size();i++)
    {
        minRect[i] = minAreaRect( Mat(contours[i]));
        //box_contours.push_back(cv::boundingRect(cv::Mat(contours[i])));
        //cv::rectangle(I_ROI_boxes, box_contours[i], cv::Scalar(0, 255, 0),2);

        Point2f rect_points[4];
        minRect[i].points(rect_points);
        for( int j = 0; j < 4; j++ )
        {
            //cv::circle(I_ROIs_rotated, rect_points[j], 3, cv::Scalar(255, 255, 255), 2);
            //cv::line(I_ROIs_rotated, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0, 255, 0), 2, 8);

            if(((rect_points[j].x) > offset && (rect_points[j].x < I_ROIs_corners.cols - offset)) && ((rect_points[j].y > offset) && (rect_points[j].y < I_ROIs_corners.rows - offset)))
            {
                cv::circle(I_ROIs_corners, rect_points[j], 3, cv::Scalar(255, 255, 255), 2);
                corners.push_back(rect_points[j]);
            }
        }
    }
    //cv::imshow("I ROI BOXES", I_ROI_boxes);
    //cv::imshow("I ROTATED ROIs", I_ROIs_rotated);
    //cv::imshow("I ROTATED CORNERS", I_ROIs_corners);
    //writer.write(I_ROIs_corners);

    return corners;
}

cv::Mat VisualObjectDetector::DetectCornersBasedOnLines(cv::Mat &I, cv::Mat &I_edges)
{
    LineDetector lineDetector;
    CreadorMascaras maskCreator;
    Hough hough;

    cv::Mat I_and_edges_color;
    I_edges.copyTo(I_and_edges_color);
    cv::Mat frame_for_hough;
    cv::Mat Hough_Space_v, Hough_Space_h;
    std::vector<cv::Point> rho_theta_v, rho_theta_h;
    int minVotes = 25;
    double range_theta_v[2], range_theta_h[2];
    range_theta_v[0] = -5;
    range_theta_v[1] = 5;

    range_theta_h[0] = 89;
    range_theta_h[1] = 91;
    I.copyTo(frame_for_hough);

    //Computing the Hough Space (convert to parametric coordinates), Non maxima
    //supression and computing the vertical lines detected in the Hough Space
    Hough_Space_v = hough.ComputeHoughSpace(I_and_edges_color, range_theta_v);
    rho_theta_v = hough.FindMaximum_InHoughSpace(Hough_Space_v, 8, minVotes);
    //std::cout<<"Rho Theta:"<<std::endl;
    //std::cout<<rho_theta<<std::endl;
    cv::Mat I_lineas_v = hough.ComputeHoughLines(frame_for_hough, Hough_Space_v, rho_theta_v);
    //imshow("I_linea_v",I_lineas_v);

    Hough_Space_h = hough.ComputeHoughSpace(I_and_edges_color, range_theta_h);
    rho_theta_h = hough.FindMaximum_InHoughSpace(Hough_Space_h, 8, minVotes);
    //std::cout<<"Rho Theta:"<<std::endl;
    //std::cout<<rho_theta<<std::endl;
    cv::Mat I_lineas_h = hough.ComputeHoughLines(frame_for_hough, Hough_Space_h, rho_theta_h);
    //imshow("I_linea_h",I_lineas_h);


    cv::Mat I_AND_lineas;
    cv::bitwise_and(I_lineas_v, I_lineas_h, I_AND_lineas);
    //cv::imshow("I AND Lineas", I_AND_lineas);

    return I_AND_lineas;
}

std::vector<cv::Point2f> VisualObjectDetector::DetectDoorCorners(cv::Mat &I)
{
    std::vector<cv::Point2f> corners_2D;
    cv::Mat I_corners;
    I.copyTo(I_corners);
    cv::Mat I_binarized = colorImageProcessor.ProcessRedColor(I);
    corners_2D = DetectCornersBasedOnContours(I, I_binarized);
    for(int i=0;i<corners_2D.size();i++)
    {
        cv::circle(I_corners, corners_2D[i], 3, cv::Scalar(255, 255, 255), 2);
    }
    imshow("Door Corners",I_corners);

    return corners_2D;
}

std::vector<cv::Point2f> VisualObjectDetector::DetectWindowCorners(cv::Mat &I)
{
    std::vector<cv::Point2f> corners_2D;
    cv::Mat I_corners;
    I.copyTo(I_corners);
    cv::Mat I_color_mask = colorImageProcessor.ProcessBlueColor(I);
    cv::Mat I_shape_mask = shapeImageProcessor.detectRectangles(I);
    cv::Mat I_final_mask;
    cv::bitwise_and(I_color_mask, I_shape_mask, I_final_mask);
    imshow("AND Color (Blue) and Shape (Rectangle)",I_final_mask);

    corners_2D = DetectCornersBasedOnContours(I, I_final_mask);
    for(int i=0;i<corners_2D.size();i++)
    {
        cv::circle(I_corners, corners_2D[i], 3, cv::Scalar(255, 255, 255), 2);
    }
    imshow("Window Corners",I_corners);

    return corners_2D;
}



std::vector<std::vector<cv::Point> > VisualObjectDetector::DetectContoursInImage(const Mat &I, const std::string object_color, const int filter_mode)
{
    cv::Mat I_thresh, I_ROI_boxes, I_gray;
    I.copyTo(I_thresh);
    I.copyTo(I_ROI_boxes);
    cv::cvtColor(I, I_gray, CV_RGB2GRAY);
    std::vector<std::vector<cv::Point> > contours;

    int LARGEST_AREA_IN_IMAGE = (I.rows/3) * (I.cols/3);
    int SMALLEST_AREA_IN_IMAGE = (I.rows/10) * (I.cols/10);



    cv::Mat I_binarized;
    if(object_color == "Red")
        I_binarized = colorImageProcessor.ProcessRedColor(I);
    else if(object_color == "Blue")
        I_binarized = colorImageProcessor.ProcessBlueColor(I);
    else if(object_color == "White")
        I_binarized = colorImageProcessor.ProcessWhiteColor(I);
    //    else if(object_color == "Black")
    //        I_binarized = colorImageProcessor.ProcessBlackColor(I);


    //    cv::Mat I_edges;
    //    I_edges = shapeImageProcessor.detectEdges(I);

    //    //AND between Processed Image based on COLOR and based on EDGES
    //    cv::Mat I_mask_color_shape;
    //    cv::bitwise_and(I_binarized, I_edges, I_mask_color_shape);

    if(IMSHOW_MODE)
    {
        cv::imshow("I BINARIZED (color processed)", I_binarized);
        //        cv::imshow("I Edges (Canny + Dilate)", I_edges);
        //        cv::imshow("I AND Color/Shape", I_mask_color_shape);
        cv::waitKey(1);
    }


    cv::findContours(I_binarized, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);



    //Filtering Stage for Deleting Not desired contours
    int size_MaxContour=0;
    if(filter_mode == ContoursFilterType::MAX_AREA_FILTER)
    {
        SMALLEST_AREA_IN_IMAGE = (I.rows/20) * (I.cols/20);
        std::vector<cv::Rect> box_contours(contours.size());
        //cout<<"**** Area de los Contornos ****"<<endl;
        for(int i=0;i<contours.size();i++)
        {
            box_contours[i] = cv::boundingRect(cv::Mat(contours[i]));
            if(box_contours[i].area() > size_MaxContour)
                size_MaxContour = box_contours[i].area();
            //cout<<"contorno i: "<<box_contours[i].area()<<endl;
        }
        //cout<<"contorno maximo:"<<endl<<size_MaxContour<<endl;
    }


    std::vector<std::vector<cv::Point> >::iterator itc = contours.begin();
    float rot_item_width = 0.0;
    float rot_item_height = 0.0;
    while(itc != contours.end())
    {
        cv::Rect box = cv::boundingRect(cv::Mat(*itc));
        cv::RotatedRect rot_box = cv::minAreaRect(cv::Mat(*itc));
        rot_item_width = (float)rot_box.size.width;
        rot_item_height = (float)rot_box.size.height;
        float box_width_height_ratio = (float)box.width/(float)box.height;
        float rot_box_width_height_ratio = rot_item_width/rot_item_height;
        float rot_box_area = (float)rot_box.size.width * (float)rot_box.size.height;

        //        cout<<"Width: "<< box.width<<endl;
        //        cout<<"Height: "<< box.height<<endl;
        //        cout<<"Rotated Ratio (Width/Height): "<< rot_box_width_height_ratio <<endl;
        //        cout<<"Ratio (Width/Height): "<< box_width_height_ratio <<endl;
        //        cout<<"Area of the Contour: "<<box.area()<<endl<<endl;


        //Count the Number of White pixels
        int non_zero_pixels = cv::countNonZero(I_binarized(box));
        float percentage_white_pixels = (float)non_zero_pixels/(float)box.area();
        float percentage_white_pixels_rot = (float)non_zero_pixels/(float)rot_box_area;


        int image_center_x = I.cols/2;
        int image_center_y = I.rows/2;
        int offset_center_image_x = 250;
        int offset_center_image_y = 180;
        bool position_filtering_condition = true;
        bool contours_filtering_condition;
        switch(filter_mode)
        {
        case ContoursFilterType::AREA_FILTER:
            contours_filtering_condition = rot_box_area < SMALLEST_AREA_IN_IMAGE;
            break;
        case ContoursFilterType::AREA_AND_SHAPE_FILTER:
            //SMALLEST_AREA_IN_IMAGE = (I.rows/20) * (I.cols/20);
            SMALLEST_AREA_IN_IMAGE = 500;
            contours_filtering_condition = (box_width_height_ratio < 0.5) || (box_width_height_ratio > 1.2)
                    || (rot_box_area < SMALLEST_AREA_IN_IMAGE) || (rot_box_area > LARGEST_AREA_IN_IMAGE) || (percentage_white_pixels_rot < 0.6);
            contours_filtering_condition = contours_filtering_condition
                    || (rot_box_width_height_ratio < 0.5)|| (rot_box_width_height_ratio > 2.5);
            break;
        case ContoursFilterType::MAX_AREA_FILTER:
            contours_filtering_condition = (box.area() < (size_MaxContour * 0.95)) ||  (rot_box_area < SMALLEST_AREA_IN_IMAGE);
            break;
        default:
            SMALLEST_AREA_IN_IMAGE = (I.rows/20) * (I.cols/20);
            //contours_filtering_condition = (rot_box_width_height_ratio < 0.7)|| (rot_box_width_height_ratio > 1.5) ||
            //(rot_box_area < SMALLEST_AREA_IN_IMAGE) || (percentage_white_pixels_rot < 0.1);
            contours_filtering_condition = rot_box_area < SMALLEST_AREA_IN_IMAGE;
            //                if(((rot_box.center.x > (image_center_x - offset_center_image_x)) && (rot_box.center.x < (image_center_x + offset_center_image_x))) &&
            //                        ((rot_box.center.y  > (image_center_y - offset_center_image_y)) && (rot_box.center.y < (image_center_y + offset_center_image_y))))
            //                {
            //                    position_filtering_condition = false;
            //                    //cout<<"Inside Image Center!"<<endl;
            //                    //cout<<"Image Size: "<<I.rows<<" ; "<<I.cols<<endl;
            //                }
            //                cv::Point p1(image_center_x - offset_center_image_x, image_center_y - offset_center_image_y);
            //                cv::Point p2(image_center_x + offset_center_image_x, image_center_y + offset_center_image_y);
            //                cv::Mat I_debug = I.clone();
            //                cv::rectangle(I_debug, p1, p2, cv::Scalar(255,0,0),2);
            //                imshow("Rectangle For Filtering in POSITION", I_debug);
            //                cv::waitKey(1);
            //                contours_filtering_condition = contours_filtering_condition || position_filtering_condition;
            break;
        }

        if(contours_filtering_condition)
            itc = contours.erase(itc);
        else
        {
            //            cout<<"Rot Box Width: "<< rot_box.size.width<<endl;
            //            cout<<"Rot Box Height: "<< rot_box.size.height<<endl;
            //            cout<<"Rot Box Ratio (Width/Height): "<< rot_box_width_height_ratio <<endl;
            //            cout<<"Non-Zero Pixels: "<<non_zero_pixels<<endl;
            //            cout<<"PERCENTAGE Non-Zero Pixels: "<<percentage_white_pixels<<endl<<endl;
            ++itc;
        }
    }

    return contours;
}

void VisualObjectDetector::EstimateObjectPose(const cv::RotatedRect rotated_item, const std::string object_type, cv::Mat &Tvec, cv::Mat &Rvec)
{
    std::vector<cv::Point2f> item_corners_2D;
    cv::Point2f corner_points[4];
    rotated_item.points(corner_points);

    for( int j = 0; j < 4; j++)
        item_corners_2D.push_back(corner_points[j]);

    std::vector<cv::Point3f> item_corners_3D;
    if(object_type == "Item")
        item_corners_3D = shapeImageProcessor.get3DObjectCorners("Item");
    else if(object_type == "Bucket")
        item_corners_3D = shapeImageProcessor.get3DObjectCorners("Bucket");
    poseEstimator.EstimatePoseFromPNP(item_corners_3D, item_corners_2D);
    Tvec = poseEstimator.getTranslation();
    Rvec = poseEstimator.getOrientation();

}

cv::RotatedRect VisualObjectDetector::DetectObject(const cv::Mat &I, const std::string object_color, const string object_type)
{
    cv::Mat I_for_contours, I_ROI_boxes;
    I.copyTo(I_ROI_boxes);
    I.copyTo(I_for_contours);
    std::vector<std::vector<cv::Point> > contours = DetectContoursInImage(I_for_contours, object_color, AREA_AND_SHAPE_FILTER);


    RotatedRect rotated_item;
    for(int i=0;i<contours.size();i++)
    {
        rotated_item = cv::minAreaRect(cv::Mat(contours[i]));
        cv::Point2f corner_points[4];
        rotated_item.points(corner_points);

        //        float rot_box_width_height_ratio = (float)rotated_item.size.width/(float)rotated_item.size.height;
        //        cout<<"Rot Box Width: "<< rotated_item.size.width<<endl;
        //        cout<<"Rot Box Height: "<< rotated_item.size.height<<endl;
        //        cout<<"Rot Box Ratio (Width/Height): "<< rot_box_width_height_ratio <<endl;
    }


    //if(contours.size())
    //{
    //    cv::Mat Tmat,Rmat;
    //    EstimateObjectPose(rotated_item, object_type, Tmat, Rmat);
    //cout<<"x: "<<Tmat.at<double>(0,0)<<" ; "<<"y: "<<Tmat.at<double>(1,0)<<"z: "<<Tmat.at<double>(2,0)<<endl;
    //}

    return rotated_item;
}


cv::RotatedRect VisualObjectDetector::DetectCylindricalObjectTopView(const Mat &I, const string object_color, const int detection_mode)
{
    //Step 1.- Compute the Rotated Rect of the object_color;
    //Step 2.- Detect Ellipses in the Image.
    //Step 3.- Ellipses Filtering. Take the best Ellipse inside the computed Rotated Rect.
    //Step 4.- Detect Contours in the Image of the Ellipse with black background.
    //Step 5.- Compute the associated Rotated Rect.

    cv::Mat I_for_contours, I_ROI_boxes;
    I.copyTo(I_ROI_boxes);
    I.copyTo(I_for_contours);
    std::vector<std::vector<cv::Point> > contours = DetectContoursInImage(I_for_contours, object_color, 4);


    RotatedRect rotated_item;
    for(int i=0;i<contours.size();i++)
    {
        rotated_item = cv::minAreaRect(cv::Mat(contours[i]));
        cv::Point2f corner_points[4];
        rotated_item.points(corner_points);

        if(IMSHOW_MODE)
        {
            for( int j = 0; j < 4; j++)
            {
                cv::line(I_ROI_boxes, corner_points[j], corner_points[(j+1)%4], cv::Scalar(0, 255, 0), 5, 8);
            }
        }
    }


    if(detection_mode == DetectionFromTopMode::ELLIPSE_DETECTION)
    {
        std::cout<<"Elipse Detection Activated!"<<std::endl;
        cv::RotatedRect ellipse_inside_rect = shapeImageProcessor.detectEllipseInObject(I, rotated_item);
        Point2f rect_points[4];
        ellipse_inside_rect.points(rect_points);
        for( int j = 0; j < 4; j++ )
        {
            cv::line(I_ROI_boxes, rect_points[j], rect_points[(j+1)%4], cv::Scalar(255, 255, 255), 2, 8);
        }
        rotated_item = ellipse_inside_rect;
    }

    if(IMSHOW_MODE)
    {
        cv::imshow("Rotated Rect From Top", I_ROI_boxes);
    }

    return rotated_item;
}

std::vector<cv::RotatedRect> VisualObjectDetector::DetectObjects(const cv::Mat &I, const std::string object_color)
{
    cv::Mat I_for_contours, I_ROI_boxes;
    I.copyTo(I_ROI_boxes);
    I.copyTo(I_for_contours);
    std::vector<std::vector<cv::Point> > contours = DetectContoursInImage(I_for_contours, object_color, AREA_AND_SHAPE_FILTER);
    cv::drawContours(I_ROI_boxes, contours, -1, Scalar(255, 0, 255), 2);
    if(IMSHOW_MODE)
    {
        cv::imshow("Contours Detected", I_ROI_boxes);
        cv::waitKey(1);
    }

    //    cv::RotatedRect top_cylinder;
    //    for(unsigned int i=0;i<contours.size();i++)
    //    {
    //        top_cylinder = cv::fitEllipse(cv::Mat(contours[i]));
    //        cv::Point2f corner_points[4];
    //        top_cylinder.points(corner_points);
    //        for( int j = 0; j < 4; j++)
    //        {
    //            cv::line(I, corner_points[j], corner_points[(j+1)%4], cv::Scalar(255, 255, 0), 2, 8);
    //        }

    //    }

    bool corners_inside_image = true;
    unsigned int boundary = 5;
    std::vector<cv::RotatedRect> rotated_objects_detected;
    for(unsigned int i=0;i<contours.size();i++)
    {
        cv::RotatedRect rotated_item = cv::minAreaRect(cv::Mat(contours[i]));
        cv::Point2f corner_points[4];
        rotated_item.points(corner_points);


        for(int j=0;j<4;j++)
        {
            if(((corner_points[j].x >= (I.cols - boundary)) || (corner_points[j].x < boundary)) ||
                    ((corner_points[j].y >= (I.rows - boundary)) || (corner_points[j].y < boundary)))
                corners_inside_image = false;
        }

        if(corners_inside_image)
            rotated_objects_detected.push_back(rotated_item);


        //        float rot_box_width_height_ratio = (float)rotated_item.size.width/(float)rotated_item.size.height;
        //        cout<<"Rot Box Width: "<< rotated_item.size.width<<endl;
        //        cout<<"Rot Box Height: "<< rotated_item.size.height<<endl;
        //        cout<<"Rot Box Ratio (Width/Height): "<< rot_box_width_height_ratio <<endl;

    }


    return rotated_objects_detected;
}

