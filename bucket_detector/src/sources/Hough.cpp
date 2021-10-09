#include "Hough.h"
#include <math.h>
using namespace std;

#define PI 3.14159265359

Hough::Hough(void)
{

}

//********************************************************************************
//FUNCTION ComputeHoughSpace():
//Inputs: 
//- I_bordes is binary image of edges resulting of a previous edge detection stage
//- rng_theta is a vector containing the range of theta
//Output:
//- cv::Mat containing the Hough Space
//*********************************************************************************
Mat Hough::ComputeHoughSpace(cv::Mat &I_bordes,double rng_theta[])
{
	int h=I_bordes.rows;
	int w=I_bordes.cols;

	theta_min=floor(rng_theta[0]+0.5);
	theta_max=floor(rng_theta[1]+0.5);


	//Here the maximum of the rho parameter is specified
	double norm2=w*w+h*h;
	double rhoLimit=sqrt(norm2);
	rho_min = floor(-rhoLimit+0.5);  
	rho_max = floor(rhoLimit+0.5);

	//Rho range between Rho_min to Rho_max
	int rango_rho=(rho_max-rho_min);

	//Theta range between -theta_min to theta_max
	int range_theta=(theta_max-theta_min);
	//Theta increment in degrees
	double delta_theta=1;

	Hough_Space=Mat::zeros(rango_rho,range_theta,CV_16U);
	//cout<<"Tamanio Espacio Hough"<<endl<<"filas:"<<Hough_Space.rows<<endl<<"Columnas:"<<Hough_Space.cols<<endl;
	

	//Here, we loop over the whole binary image containing the detected edges, 
	//and for each pixel we compute the polar coordinates (rho and theta), 
	//and acummulate its vote into cv:: Mat Hough_Space.
	for(int y=0;y<h;y++)
	{
		for(int x=0;x<w;x++)
		{
			if(I_bordes.at<uchar>(y,x)>0)
			{
				for(double z=theta_min;z<theta_max;z+=delta_theta)
				{
					double theta=z*PI/180;
					int rho=floor(((double)x*cos(theta)+(double)y*sin(theta))+0.5);
					int y_hough=(int)rho+(-rho_min);
					int x_hough=(int)z+(-theta_min);
					Hough_Space.at<ushort>(y_hough,x_hough)++;
				}
			}
		}
	}


	//***********************************************************************************
	//The code bellow is used only for showing the results
	//***********************************************************************************
	Mat Espacio_Hough_resized;
	Mat Espacio_Hough_resized_8U;
	Mat hough_rgb;
	//Mat Espacio_Hough_resized=hough.ComputeHoughSpace(bordes_kernel);
	cv::resize(Hough_Space,Espacio_Hough_resized,Size(Hough_Space.cols,Hough_Space.rows/2));
	Espacio_Hough_resized.convertTo(Espacio_Hough_resized_8U,CV_8U);
	cv::cvtColor(Espacio_Hough_resized_8U,hough_rgb,CV_GRAY2BGR);

	imshow("Espacio Hough",Espacio_Hough_resized_8U);
	//*************************************************************************************
	
	return Hough_Space;
}


//Note: This function has to be OPTIMIZED
//**************************************************************************************
//FUNCTION FindMaximum_InHoughSpace():
//Inputs: 
//- (E_hough) is the Hough space stored as a cv::Mat object computed with ComputeHoughSpace().
//- num_picos is and integer representing the number of peaks that you want to detect.
//- umbral is the threshold above which you want to retain the detected lines.
//Output:
//- vector<cv::Point> of (rho,theta) pairs.
//***************************************************************************************
std::vector<cv::Point> Hough::FindMaximum_InHoughSpace(Mat &E_Hough,int num_picos,int umbral)
{
	double minVal; 
	double maxVal; 
	Point minLoc;
	Point maxLoc;

	int picos_detectados=0;//This variable will store the number of peaks detected
	int vecino=5;//this variable means that a neighborhood environment of 3x3 is
	//considered during maxima supression. (1 pixel previous and posterior the
	//central pixel).
	Mat Hough_new;
	E_Hough.copyTo(Hough_new);

	std::vector<cv::Point> rho_theta;
	std::vector<cv::Point> peaks;

	bool peak_search_finished=false;

	while(!peak_search_finished)
	{
		minMaxLoc(Hough_new,&minVal,&maxVal,&minLoc,&maxLoc);
		//We don't consider peaks with values<threshold(umbral)
		if(maxVal>=umbral)
		{
			picos_detectados++;
			if(picos_detectados>num_picos)
				break;
			//cout<<endl<<"***** DETECCION DE MAXIMOS *****"<<endl;
			//cout<<"pico numero: "<<picos_detectados<<endl;
			//cout << "max val: " << maxVal << endl;
			//cout<< "max Location:"<<maxLoc<<endl;
			peaks.push_back(maxLoc);

			//Here we substract the offset rho_max, theta_max
			int rho_c=maxLoc.y-(-rho_min);
			int theta_c=maxLoc.x-(-theta_min);
			rho_theta.push_back(Point(theta_c,rho_c));
			//cout<< "max Location:"<<Point(theta_c,rho_c)<<endl;
		
			//Here the neighborhood environment of the detected peak
			//is defined in order to remove the local peaks around it.
			int p1=(maxLoc.y-vecino),p2=(maxLoc.y+vecino);
			int q1=(maxLoc.x-vecino),q2=(maxLoc.x+vecino);
			if(p1<0)
				p1=0;
			if(p2>=Hough_new.rows)
				p2=Hough_new.rows-1;
			if(q1<0)
				q1=0;
			if(q2>=Hough_new.cols)
				q2=Hough_new.cols-1;
   
			//This loop is used for doing the non-maxima supression
			for(int rho=p1;rho<=p2;rho++)
			{
				for(int theta=q1;theta<=q2;theta++)
				{
					Hough_new.at<ushort>(rho,theta)=0;
				}
			}
		}
		else
			peak_search_finished=true;

	}
	
	//***********************************************************************************
	//The code bellow is used only for showing the results
	//***********************************************************************************
	Mat Espacio_Hough_resized;
	Mat Espacio_Hough_resized_8U;
	Mat hough_rgb;

	cv::resize(E_Hough,Espacio_Hough_resized,Size(Hough_Space.cols,Hough_Space.rows/2));
	Espacio_Hough_resized.convertTo(Espacio_Hough_resized_8U,CV_8U);
	cv::cvtColor(Espacio_Hough_resized_8U,hough_rgb,CV_GRAY2BGR);
	for(int i=0;i<peaks.size();i++)
	{
		peaks[i].y=peaks[i].y/2;
		cv::circle(hough_rgb,peaks[i],5,Scalar(0,0,255),2);
	}

	Mat Hough_new_resized;
	Mat Hough_new_resized_8U;
	cv::resize(Hough_new,Hough_new_resized,Size(Hough_new.cols,Hough_new.rows/2));
	Hough_new_resized.convertTo(Hough_new_resized_8U,CV_8U);
	//imshow("Hough new resized",Hough_new_resized_8U);
	//imshow("Espacio Hough Maximum",hough_rgb);
	//*************************************************************************************
	


	return rho_theta;
}



//**********************************************************************************
//FUNCTION ComputeHoughLines():
//Inputs: 
//- I is the RGB original image for showing the detected Hough Lines.
//- E_Hough is the Hough space computed with ComputeHoughSpace().
//- rho_theta is a std::vector<cv::Point> containing the detected (rho,theta) pairs.
//Output:
//- cv::Mat binary image showing the final results of the detected Hough lines.
//***********************************************************************************
Mat Hough::ComputeHoughLines(Mat &I,Mat &E_Hough,std::vector<cv::Point> rho_theta)
{
	int h=I.rows;
	int w=I.cols;
	Mat Lineas_Hough;
	Mat I_rgb;
	I.copyTo(I_rgb);
	Mat I_gray;
	cv::cvtColor(I,I_gray,CV_BGR2GRAY);
	Mat I_linea=Mat(I_gray.size(),CV_8U,Scalar(0));
	

	//Inside this loop, each (rho,theta) pair is converted
	//to the corresponding points in the image.
	for(int i=0;i<rho_theta.size();i++)
	{
		double theta_def=rho_theta[i].x;
		double rho_def=rho_theta[i].y;

		if(theta_def==90)
			theta_def=89.99999;
		if(theta_def==0)
			theta_def=0.00001;

		theta_def=theta_def*PI/180;

		std::vector<int> x_imagen,y_imagen;
		//Eje y=0;x=rho-y*sen(theta)/cos(theta);
		double x1=rho_def/cos(theta_def);
		if(x1<=w && x1>=0)
		{
			x_imagen.push_back(floor(x1+0.5));
			y_imagen.push_back(0);
		}

		//Eje x=width;y=rho-x*cos(theta)/sen(theta);
		double y1=(rho_def-w*cos(theta_def))/sin(theta_def);
		if(y1<=h && y1>=0)
		{
			x_imagen.push_back(w);
			y_imagen.push_back(floor(y1+0.5));;
		}

		//Eje y=heigth;x=rho-y*sen(theta)/cos(theta);
		double x2=(rho_def-h*sin(theta_def))/cos(theta_def);
		if(x2<=w && x2>=0)
		{
			x_imagen.push_back(floor(x2+0.5));;
			y_imagen.push_back(h);
		}

		//Eje x=0;y=rho-x*cos(theta)/sen(theta);
		double y2=rho_def/sin(theta_def);
		if(y2<=h && y2>=0)
		{
			x_imagen.push_back(0);
			y_imagen.push_back(floor(y2+0.5));;
		}

		//cout<<"X imagen:"<<endl;
		//for(int i=0;i<x_imagen.size();i++)
		//	cout<<x_imagen[i]<<endl;
		//cout<<endl;
		//cout<<"Y imagen:"<<endl;
		//for(int i=0;i<y_imagen.size();i++)
		//	cout<<y_imagen[i]<<endl;
		//cout<<endl;


		//**************************************************************************************************************
		//This code is for drawing the detected Hough lines
		//**************************************************************************************************************
		if(x_imagen.size())
		{
			//cout<<"Punto 1:"<<endl<<Point(x_imagen[0],y_imagen[0])<<endl<<"Punto 2:"<<endl<<cv::Point(x_imagen[1],y_imagen[1])<<endl;
			cv::line(I_rgb,cv::Point(x_imagen[0],y_imagen[0]),cv::Point(x_imagen[1],y_imagen[1]),Scalar(255,0,0),4);
			line(I_linea,cv::Point(x_imagen[0],y_imagen[0]),cv::Point(x_imagen[1],y_imagen[1]),Scalar(255),4);
		}
		//***************************************************************************************************************
		
	}
	//imshow("Resultado HOUGH MIO",I_rgb);

	return I_linea;

}

Hough::~Hough(void)
{
}
