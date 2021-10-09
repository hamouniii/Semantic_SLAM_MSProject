#include "LineDetector.h"


LineDetector::LineDetector(void)
{
	range_theta[0] = -10;
	range_theta[1] = 10;
	minVotes = 100;
}

std::vector<cv::Point> LineDetector::DetectHoughLines(Mat &frame)
{
	cv::Mat I,I_gray,frame_for_hough,I_gray_median,I_hsv;
	Mat edges,thresholded_edges,I_vertical_edges;
	frame.copyTo(I);
	frame.copyTo(frame_for_hough);

	cv::cvtColor(I,I_gray,CV_BGR2GRAY);
	//cv::cvtColor(I,I_hsv,CV_BGR2HSV);

	//std::vector<cv::Mat> canales_rgb;
	//std::vector<cv::Mat> canales_hsv;
	//cv::split(I,canales_rgb);
	//cv::split(I,canales_hsv);
	//cv::Mat I_B = canales_rgb[0];
	//cv::Mat I_G = canales_rgb[1];
	//cv::Mat I_R = canales_rgb[2];
	//cv::Mat I_H = canales_hsv[0];
	//cv::Mat I_S = canales_hsv[1];
	//cv::Mat I_V = canales_hsv[2];
	//cv::Mat I_V_eq;
	//cv::equalizeHist(I_V,I_V_eq);
	//imshow("I_V Equalized",I_V_eq);


	//cv::equalizeHist(I_gray,I_gray_median);
	medianBlur(I_gray,I_gray_median,5);
	

	//Código para aumentar el contraste y detectar líneas
	//en imágenes con bajo contraste.
	cv::Mat I_sharp,I_sharp_eq;
	cv::Mat I_sharp2,I_sharp_eq2;
	cv::GaussianBlur(I_gray,I_sharp, cv::Size(15,15),0,15);
	cv::addWeighted(I_gray, 1.5,I_sharp,-1, 0,I_sharp);
	I_sharp.copyTo(I_gray_median);


	//cv::equalizeHist(I_sharp,I_sharp_eq);
	//I_sharp_eq.copyTo(I_gray_median);
	//medianBlur(I_sharp_eq,I_gray_median,5);
	
	

	//Convolution of the image (gray level+median filter)
	//with the vertical kernel KernelI(2), and threshold the result image
	Mat kernel_I = maskCreator.getKernelI(2);
	//cv::filter2D(I_sharp_eq2,edges,I.depth(),kernel_I);
	cv::filter2D(I_gray_median,edges,I.depth(),kernel_I);
	cv::threshold(edges,thresholded_edges,50,255,cv::THRESH_BINARY);
	thresholded_edges.copyTo(I_vertical_edges_thr);
	thresholded_edges.copyTo(I_vertical_edges);

	//imshow("I_gray",I_gray);
	//imshow("I_gray Equalized",I_gray_median);
	//imshow("I_sharp",I_sharp);
	//imshow("I_sharp Equalized",I_sharp_eq);
	//imshow("I_sharp Equalized",I_sharp_eq);
    imshow("Kernel Vertical Thresholded",thresholded_edges);
	

	
	Mat Hough_Space;
	std::vector<cv::Point> rho_theta;

    //Computing the Hough Space (convert to parametric coordinates), Non maxima
    //supression and computing the vertical lines detected in the Hough Space
	Hough_Space = hough.ComputeHoughSpace(I_vertical_edges,range_theta);
	rho_theta = hough.FindMaximum_InHoughSpace(Hough_Space,4,minVotes);
	//std::cout<<"Rho Theta:"<<std::endl;
	//std::cout<<rho_theta<<std::endl;
	cv::Mat I_linea_v = hough.ComputeHoughLines(frame_for_hough,Hough_Space,rho_theta);
    //imshow("I_linea_v",I_linea_v);

	I_linea_v.copyTo(I_vertical_lines);



	return rho_theta;
}


std::vector<std::vector<cv::Point> > LineDetector::DetectVerticalContours(cv::Mat &result_contours_columnas_torre)
{
	cv::Mat tower_columns;
	cv::Mat bordes_kernel_thr;
	cv::Mat I_v_lines;

	I_vertical_edges_thr.copyTo(bordes_kernel_thr);
	I_vertical_lines.copyTo(I_v_lines);

	cv::bitwise_and(bordes_kernel_thr,I_v_lines,tower_columns);
	//imshow("Resultado AND",tower_columns);

	
	//Extract contours and remove those who doesn't verify the criteria.
	cv::Mat element9(18,2,CV_8U,Scalar(1));
	cv::dilate(tower_columns,tower_columns,element9);
	//cv::erode(tower_columns,tower_columns,cv::Mat(9,2,CV_8U,Scalar(1)));
    //imshow("Tower Columns",tower_columns);




	std::vector<std::vector<cv::Point> > contours_columnas_torre;
	cv::findContours(tower_columns,contours_columnas_torre,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

    //cv::Mat result_contours_columnas_torre(tower_columns.size(),CV_8U,Scalar(0));
		
	std::vector<std::vector<cv::Point> >::iterator itc_columnas=contours_columnas_torre.begin();
	while(itc_columnas!=contours_columnas_torre.end())
	{
		if(itc_columnas->size()<200)
			itc_columnas=contours_columnas_torre.erase(itc_columnas);
		else
			++itc_columnas;
	}
	cv::drawContours(result_contours_columnas_torre,contours_columnas_torre,-1,Scalar(255),2);
    //imshow("Contours Columnas Torre Modulo 1",result_contours_columnas_torre);
	
	return contours_columnas_torre;
}




LineDetector::~LineDetector(void)
{
}
