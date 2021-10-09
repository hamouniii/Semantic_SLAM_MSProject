#include "PoseEstimator.h"

using namespace std;

PoseEstimator::PoseEstimator(void)
{
    cameraMatrix = cv::Mat::zeros(3,3,cv::DataType<double>::type);
    distCoeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type);
    Rvec = cv::Mat::zeros(3,1,cv::DataType<double>::type);
    Tvec = cv::Mat::zeros(3,1,cv::DataType<double>::type);
}


PoseEstimator::~PoseEstimator(void)
{
}


void PoseEstimator::init(std::string configFile)
{
    readConfigs(configFile);
}

bool PoseEstimator::readConfigs(std::string configFile)
{
    try
    {
        XMLFileReader my_xml_reader(configFile);

        cameraMatrix.at<double>(0,0) = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:CameraMatrix:at00");
        cameraMatrix.at<double>(0,1) = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:CameraMatrix:at01");
        cameraMatrix.at<double>(0,2) = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:CameraMatrix:at02");

        cameraMatrix.at<double>(1,0) = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:CameraMatrix:at10");
        cameraMatrix.at<double>(1,1) = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:CameraMatrix:at11");
        cameraMatrix.at<double>(1,2) = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:CameraMatrix:at12");

        cameraMatrix.at<double>(2,0) = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:CameraMatrix:at20");
        cameraMatrix.at<double>(2,1)= my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:CameraMatrix:at21");
        cameraMatrix.at<double>(2,2) = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:CameraMatrix:at22");

        distCoeffs.at<double>(0) = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:DistMatrix:at0");
        distCoeffs.at<double>(1) = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:DistMatrix:at1");
        distCoeffs.at<double>(2) = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:DistMatrix:at2");
        distCoeffs.at<double>(3) = my_xml_reader.readDoubleValue("ShapeColor_ObjectDetector_config:PoseEstimator:DistMatrix:at3");

        cout<<"Camera Matrix: "<<cameraMatrix<<endl;
        cout<<"distCoeffs: "<<distCoeffs<<endl;
    }
    catch ( cvg_XMLFileReader_exception &e)
    {
        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }
}


void PoseEstimator::EstimatePoseFromPNP(std::vector<cv::Point3f> points_3D, std::vector<cv::Point2f> points_2D)
{
    //ReadCameraParameters(1);
    std::vector<cv::Point2f> points_2D_sorted;
    if(points_2D.size() == 4)
    {
        points_2D_sorted = matchOBjectToImageRansac(points_2D);
        //cv::solvePnP(points_3D, points_2D_sorted, cameraMatrix, distCoeffs, Rvec, Tvec);
        cv::solvePnPRansac(points_3D, points_2D_sorted, cameraMatrix, distCoeffs, Rvec, Tvec);
        if(cv::countNonZero(Rvec) == 0 && cv::countNonZero(Tvec) == 0)
        {
            //std::cout<<"Using solvePnP without RANSAC"<<std::endl;
            cv::solvePnP(points_3D, points_2D_sorted, cameraMatrix, distCoeffs, Rvec, Tvec);
        }

        //std::cout<<"Translation Matrix From SolvePnP: "<<std::endl<<Tvec<<std::endl<<std::endl;
        //std::cout<<"Rotation Matrix From SolvePnP:: "<<std::endl<<Rvec<<std::endl<<std::endl;
    }
}


void PoseEstimator::ReadCameraParameters(int parameters_type)
{
    switch(parameters_type)
    {
        case 1: //Camera in GAZEBO
            cameraMatrix.at<double>(0,0) = 457.84999843514953;
            cameraMatrix.at<double>(0,1) = 0.0;
            cameraMatrix.at<double>(0,2) = 320.5;
            cameraMatrix.at<double>(1,0) = 0.0;
            cameraMatrix.at<double>(1,1) = 457.84999843514953;
            cameraMatrix.at<double>(1,2) = 240.5;
            cameraMatrix.at<double>(2,0) = 0.0;
            cameraMatrix.at<double>(2,1) = 0.0;
            cameraMatrix.at<double>(2,2) = 1.0;

            distCoeffs.at<double>(0) = 0;
            distCoeffs.at<double>(1) = 0;
            distCoeffs.at<double>(2) = 0;
            distCoeffs.at<double>(3) = 0;
            break;
        case 2: //Camera Logitech
            cameraMatrix.at<double>(0,0) = 632.754941;
            cameraMatrix.at<double>(0,1) = 0.0;
            cameraMatrix.at<double>(0,2) = 631.632937;
            cameraMatrix.at<double>(1,0) = 0.0;
            cameraMatrix.at<double>(1,1) = 324.168437;
            cameraMatrix.at<double>(1,2) = 229.244864;
            cameraMatrix.at<double>(2,0) = 0.0;
            cameraMatrix.at<double>(2,1) = 0.0;
            cameraMatrix.at<double>(2,2) = 1.0;

            distCoeffs.at<double>(0) = 0;
            distCoeffs.at<double>(1) = 0;
            distCoeffs.at<double>(2) = 0;
            distCoeffs.at<double>(3) = 0;
            break;

    }

    //cout<<"Camera Matrix:"<<endl;
    //cout<<cameraMatrix<<endl<<endl;
}



void PoseEstimator::Project3Dpoints(cv::Mat &I, std::string image_name, std::vector<cv::Point3f> &points_3D_for_project, cv::Mat &rotMat, cv::Mat &transMat)
{
    char door_corner_id[10];
    cv::Mat I_for_pose;
    I.copyTo(I_for_pose);


    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(points_3D_for_project, rotMat, transMat, cameraMatrix, distCoeffs, projectedPoints);


    cv::line(I_for_pose, projectedPoints[0], projectedPoints[1], cv::Scalar(0, 0, 255, 255), 3, CV_AA);
    cv::line(I_for_pose, projectedPoints[0], projectedPoints[2], cv::Scalar(0, 255, 0, 255), 3, CV_AA);
    cv::line(I_for_pose, projectedPoints[0], projectedPoints[3], cv::Scalar(255, 0, 0, 255), 3, CV_AA);

    cv::putText(I_for_pose, "x", projectedPoints[1], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255, 255), 2);
    cv::putText(I_for_pose, "y", projectedPoints[2], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0, 255), 2);
    cv::putText(I_for_pose, "z", projectedPoints[3], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0, 255), 2);

    std::sprintf(door_corner_id,"%f",Tvec.at<double>(0,0));
    cv::putText(I_for_pose, "x: ", cv::Point(10,20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    cv::putText(I_for_pose, door_corner_id, cv::Point(30,20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
    std::sprintf(door_corner_id,"%f",Tvec.at<double>(1,0));
    cv::putText(I_for_pose, "y: ", cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    cv::putText(I_for_pose, door_corner_id, cv::Point(30,50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
    std::sprintf(door_corner_id,"%f",Tvec.at<double>(2,0));
    cv::putText(I_for_pose, "z: ", cv::Point(10,80), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    cv::putText(I_for_pose, door_corner_id, cv::Point(30,80), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);

    imshow(image_name, I_for_pose);
}


std::vector<cv::Point2f> PoseEstimator::matchOBjectToImageRansac(std::vector<cv::Point2f> image_not_ordered)
{
    int MIN_ANGLE = 88;
    int MAX_ANGLE = 92;
    std::vector<cv::Point2f> image_ordered;
    std:vector<connections> conn_vector;
    connections conn;


    int min = 0;
    int max = image_not_ordered.size() - 1;
    float angle0, angle1;
    bool initialize = true;
    int o;
    int d;


    cv::Point2f p;


    // Initiliaze image_ordered

    for (int i=0; i<image_not_ordered.size(); i++){

        image_ordered.push_back(p);

    }


    // Randomly connect the whole graph

    int connected = 0;

    while(connected < image_not_ordered.size())
    {
        if(initialize)
        {
            // Initialize ordered
            conn.o.x = -1;
            conn.d0.x = -1;
            conn.d1.x = -1;
            conn.angle = -1;
            conn.connected = false;
            conn_vector.clear();

            for (int i=0; i<image_not_ordered.size(); i++)
                conn_vector.push_back(conn);

            initialize = false;
            connected = 0;

        }


        o = min + (rand() % (int)(max - min + 1));
        d = min + (rand() % (int)(max - min + 1));

        if (o == d)
            continue;


        if (conn_vector[o].o.x == -1) // If origin is not yet set
            conn_vector[o].o = image_not_ordered[o];


        if(conn_vector[o].d0.x == -1) // Not set destination 0
        {
            if((conn_vector[d].d0 != conn_vector[o].o) && (conn_vector[d].d1 != conn_vector[o].o) && (conn_vector[d].o.x != -1)){    // Not already a destination and origin is set

                if(conn_vector[d].d0.x == -1)
                {
                    // Cross linking
                    conn_vector[d].d0 = conn_vector[o].o;
                    conn_vector[o].d0 = conn_vector[d].o;

                }

                else if(conn_vector[d].d1.x == -1)
                {
                    // Cross linking
                    conn_vector[d].d1 = conn_vector[o].o;
                    conn_vector[o].d0 = conn_vector[d].o;

                }

            }

        }

        else if(conn_vector[o].d1.x == -1)// Not set destination 1
        {
            if((conn_vector[d].d0 != conn_vector[o].o) && (conn_vector[d].d1 != conn_vector[o].o) && (conn_vector[d].o.x != -1)){    // Not already a destination and origin is set

                if(conn_vector[d].d0.x == -1)
                {
                    // Cross linking
                    conn_vector[d].d0 = conn_vector[o].o;
                    conn_vector[o].d1 = conn_vector[d].o;

                }

                else if(conn_vector[d].d1.x == -1)
                {
                    // Cross linking
                    conn_vector[d].d1 = conn_vector[o].o;
                    conn_vector[o].d1 = conn_vector[d].o;

                }

            }

        }

        else{   // It is already connected

            conn_vector[o].connected = true;
            connected = 0;
            for(int i=0; i<conn_vector.size(); i++){

                if(conn_vector[i].connected){

                    connected++;

                }

            }

            //TODO: calculate angle
            angle0 = atan((conn_vector[o].d0.y - conn_vector[o].o.y)/(conn_vector[o].d0.x - conn_vector[o].o.x))*180/M_PI;
            angle1 = atan((conn_vector[o].d1.y - conn_vector[o].o.y)/(conn_vector[o].d1.x - conn_vector[o].o.x))*180/M_PI;
//            cout << "angle0: " << angle0 << endl;
//            cout << "angle1: " << angle1 << endl;


            if(((abs(angle0) + abs(angle1))) < MIN_ANGLE || (abs(angle0) + abs(angle1)) > MAX_ANGLE){
                initialize = true;
//                cout << "angle not: " << angle0 + angle1 << endl;
            }

            else{
//                cout << "angle: " << abs(angle0) + abs(angle1) << endl;
//                cout << "origin: " << o << endl;

                if(abs(angle0/angle1) >= 1){

                    conn_vector[o].angle = (angle0/abs(angle0))*(abs(angle0) + abs(angle1));

                }

                else{

                    conn_vector[o].angle = (angle1/abs(angle1))*(abs(angle0) + abs(angle1));

                }

            }

        }

    }


    // Look for the origin
    float dist_min = 14E12;
    float dist;
    int origin;
    for(int i=0; i<image_not_ordered.size(); i++)
    {
        dist = sqrt((conn_vector[i].o.x * conn_vector[i].o.x) + (conn_vector[i].o.y * conn_vector[i].o.y));
        if (dist < dist_min)
        {
            dist_min = dist;
            origin = i;
        }
    }


    // Order from origin

    image_ordered[0] = conn_vector[origin].o;
    if(conn_vector[origin].d0.x > conn_vector[origin].d1.x)
    {
        image_ordered[1] = conn_vector[origin].d0;
        image_ordered[3] = conn_vector[origin].d1;
    }

    else
    {
        image_ordered[1] = conn_vector[origin].d1;
        image_ordered[3] = conn_vector[origin].d0;
    }


    for(int i=0; i<image_not_ordered.size(); i++)
    {
        if((conn_vector[i].o != image_ordered[0]) && (conn_vector[i].o != image_ordered[1]) && (conn_vector[i].o != image_ordered[3]))
            image_ordered[2] = conn_vector[i].o;
    }


    //cout << image_ordered << endl;
    return image_ordered;
}


