
#include "opencv2/opencv.hpp"
//#include "opencv2/nonfree/features2d.hpp"
#include "CreadorMascaras.h"
#include "Hough.h"
#include "LineDetector.h"
#include "VisualObjectDetector.h"
#include "PoseEstimator.h"
#include "ColorImageProcessor.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>
using namespace std;


std::vector<cv::Point3f> Read3DObjectPoints(std::vector<cv::Point2f> &image_corners, int object_type);
std::vector<cv::Point3f> Read3DProjectedPoints(int object_type);



int main(int argc, char **argv)
{
    VisualObjectDetector visualObjectDetector;
	PoseEstimator poseEstimator;
	ColorImageProcessor colorImageProcessor;
	cv::VideoWriter writer;
	cv::VideoCapture cap;
    vector<vector<Point> > rectangles;

	/*********************************************************************************************************/
	// Código para CARGAR un VIDEO
	/*********************************************************************************************************/
    //cap.open("/media/carlos/Elements/IMAV_2016_Competition/Videos_Gazebo/video_calibrated.ogv");
    cap.open(0);
	//cap.open("I:/IMAV_2016/Videos_Gazebo/texture3.ogv");
    if(!cap.isOpened())  // check if we succeeded
        return -1;
	/*********************************************************************************************************/


	/*********************************************************************************************************/
	// Código para GUARDAR un VIDEO
	/*********************************************************************************************************/
	//std::string video_filename = "Corner_Detection_IMAV_2.avi";
	//writer.open(video_filename,-1,20,I_orig.size());
 //   if (!writer.isOpened())
 //   {
 //       cout  << "Could not open the input video: "<< endl;
 //       return -1;
 //   }
	/*********************************************************************************************************/


	while(1)
	{
		cv::Mat I, I_canny, I_hsv;
		cap>>I;
		if(I.empty())
		{
			printf("!!! cvCaptureFromAVI failed (file not found?)\n");
			return -1; 
		}

		cv::Mat I_for_hough, I_for_color, I_for_pose;
        cv::Mat I_squares,I_helipad;
        I.copyTo(I_for_hough);
        I.copyTo(I_for_color);
        I.copyTo(I_for_pose);
        I.copyTo(I_squares);
        I.copyTo(I_helipad);

        //colorImageProcessor.ProcessInLabColorSpace(I_for_color);
		//colorImageProcessor.ProcessInHLSColorSpace(I_for_color);
        //colorImageProcessor.ProcessInOpponentColorSpace(I_for_color);


        //std::vector<cv::Point2f> door_Item_2D;
        //door_Item_2D = visualObjectDetector.DetectObject(I_for_color, "Red", "Item");
        cv::RotatedRect rotated_item;
        rotated_item = visualObjectDetector.DetectObject(I_for_color, "Red", "Item");
        //rotated_item = visualObjectDetector.DetectObject(I_for_color, "Blue", "Item");


        bool detect_door_and_window = true;

//        std::vector<cv::Point2f> door_corners_2D;
//        door_corners_2D = visualObjectDetector.DetectDoorCorners(I_for_color);
//        std::vector<cv::Point3f> door_corners_3D = Read3DObjectPoints(door_corners_2D, 1);
////		poseEstimator.EstimatePoseFromPNP(door_corners_3D, door_corners_2D);
////		if(door_corners_2D.size() == 4)
////		{
////			std::vector<cv::Point3f> points_3D_for_project = Read3DProjectedPoints(1);
////			cv::Mat Rmat = poseEstimator.getOrientation();
////			cv::Mat Tmat = poseEstimator.getTranslation();
////			poseEstimator.Project3Dpoints(I, "Door Projected Axis", points_3D_for_project, Rmat, Tmat);
////		}

//		if(detect_door_and_window)
//		{

//            std::vector<cv::Point2f> window_corners_2D;
//            window_corners_2D = visualObjectDetector.DetectWindowCorners(I_for_color);
//            std::vector<cv::Point3f> window_corners_3D = Read3DObjectPoints(window_corners_2D, 3);
////			poseEstimator.EstimatePoseFromPNP(window_corners_3D, window_corners_2D);
////			if(window_corners_2D.size() == 4)
////			{
////				std::vector<cv::Point3f> points_3D_for_project = Read3DProjectedPoints(3);
////				cv::Mat Rmat = poseEstimator.getOrientation();
////				cv::Mat Tmat = poseEstimator.getTranslation();
////				poseEstimator.Project3Dpoints(I, "Window Projected Axis", points_3D_for_project, Rmat, Tmat);
////			}
//		}



        char tecla = cv::waitKey(10);
		if(tecla==27)
			break;
		else if(tecla=='p')
			tecla=cv::waitKey();
		else if(tecla=='g')
		{
			//writer.write(I_ROIs_corners);
		}
    }

	return 0;
}

std::vector<cv::Point3f> Read3DObjectPoints(std::vector<cv::Point2f> &image_corners, int object_type)
{
	
	std::vector<cv::Point3f> points_3D;
	cv::Point3f p;
 
	switch(object_type)
	{
		case 1: //3D DOOR POINTS (with respect upper-left corner of the door GAZEBO)
			cout<<"Door"<<endl;
			points_3D.push_back(cv::Point3f(0.0, 0.0, 0.0));
			points_3D.push_back(cv::Point3f(1.2, 0.0, 0.0));
			points_3D.push_back(cv::Point3f(1.2, 2.0, 0.0));
			points_3D.push_back(cv::Point3f(0.0, 2.0, 0.0));
			break;
		case 2: //3D WINDOW POINTS (with respect upper-left corner of the door GAZEBO)
			cout<<"Window"<<endl;
			points_3D.push_back(cv::Point3f(6.0, 0.0, 0.0));
			points_3D.push_back(cv::Point3f(7.0, 0.0, 0.0));
			points_3D.push_back(cv::Point3f(7.0, 1.0, 0.0));
			points_3D.push_back(cv::Point3f(6.0, 1.0, 0.0));
			break;
		case 3: //3D WINDOW POINTS (with respect upper-left corner of the window GAZEBO)
			cout<<"Window"<<endl;
			points_3D.push_back(cv::Point3f(0.0, 0.0, 0.0));
			points_3D.push_back(cv::Point3f(1.0, 0.0, 0.0));
			points_3D.push_back(cv::Point3f(1.0, 1.0, 0.0));
			points_3D.push_back(cv::Point3f(0.0, 1.0, 0.0));
			break;
		case 4: //3D DOOR POINTS (Lab framework)
			cout<<"Window"<<endl;
			points_3D.push_back(cv::Point3f(0.0, 0.0, 0.0));
			points_3D.push_back(cv::Point3f(0.135, 0.0, 0.0));
			points_3D.push_back(cv::Point3f(0.135, 0.205, 0.0));
			points_3D.push_back(cv::Point3f(0.0, 0.205, 0.0));
			break;

		case 5: //3D WINDOW POINTS (Lab framework)
			cout<<"Window"<<endl;
			points_3D.push_back(cv::Point3f(0.0, 0.0, 0.0));
			points_3D.push_back(cv::Point3f(0.11, 0.0, 0.0));
			points_3D.push_back(cv::Point3f(0.11, 0.11, 0.0));
			points_3D.push_back(cv::Point3f(0.0, 0.11, 0.0));
			break;
	}

	//cout<<"window_corners_3D:"<<endl;
	//for(int i=0;i<points_3D.size();i++)
	//	cout<<points_3D[i]<<endl;
	//cout<<endl<<endl;

	return points_3D;
}

std::vector<cv::Point3f> Read3DProjectedPoints(int object_type)
{
	std::vector<cv::Point3f> points_3D_for_project;
	cv::Point3f p;
 
	switch(object_type)
	{
		case 1: //3D DOOR POINTS (with respect upper-left corner of the door GAZEBO)
			cout<<"Door"<<endl;
			points_3D_for_project.push_back(cv::Point3f(0.6, 1.0, 0.0));
			points_3D_for_project.push_back(cv::Point3f(1.2, 1.0, 0.0));
			points_3D_for_project.push_back(cv::Point3f(0.6, 0.0, 0.0));
			points_3D_for_project.push_back(cv::Point3f(0.6, 1.0, 0.6));
			break;
		case 2: //3D WINDOW POINTS (with respect upper-left corner of the door GAZEBO)
			cout<<"Window"<<endl;
			points_3D_for_project.push_back(cv::Point3f(6.0, 0.0, 0.0));
			points_3D_for_project.push_back(cv::Point3f(7.0, 0.0, 0.0));
			points_3D_for_project.push_back(cv::Point3f(7.0, 1.0, 0.0));
			points_3D_for_project.push_back(cv::Point3f(6.0, 1.0, 0.0));
			break;
		case 3: //3D WINDOW POINTS (with respect upper-left corner of the window GAZEBO)
			cout<<"Window"<<endl;
			points_3D_for_project.push_back(cv::Point3f(0.5, 0.5, 0.0));
			points_3D_for_project.push_back(cv::Point3f(1.0, 0.5, 0.0));
			points_3D_for_project.push_back(cv::Point3f(0.5, 0.0, 0.0));
			points_3D_for_project.push_back(cv::Point3f(0.5, 0.5, 0.5));
			break;
		case 4: //3D DOOR POINTS (Lab framework)
			cout<<"Window"<<endl;
			points_3D_for_project.push_back(cv::Point3f(0.065, 0.105, 0.0));
			points_3D_for_project.push_back(cv::Point3f(0.135, 0.105, 0.0));
			points_3D_for_project.push_back(cv::Point3f(0.065, 0.0, 0.0));
			points_3D_for_project.push_back(cv::Point3f(0.065, 0.105, 0.065));
			break;
	}

	return points_3D_for_project;
}




