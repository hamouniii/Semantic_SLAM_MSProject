#include "CreadorMascaras.h"
using namespace std;

CreadorMascaras::CreadorMascaras(void)
{
	kernel_X=Mat(5,5,CV_32F,Scalar(-2));
	
	//kernel_T=Mat(5,5,CV_32F,Scalar(0));
	kernel_T=Mat(8,5,CV_32F,Scalar(0));
	//kernel_T=Mat(5,8,CV_32F,Scalar(0));
	
	kernel_cruz=Mat(5,5,CV_32F,Scalar(0));
	//kernel_diag_dcha=Mat(3,3,CV_32F,Scalar(0));
	kernel_diag_dcha=Mat(5,5,CV_32F,Scalar(0));

	kernel_unsharp=Mat(3,3,CV_32F,Scalar(0));
}
Mat CreadorMascaras::getKernelT(int tipo)
{

	switch(tipo)
	{
	case 1:
		kernel_T=Mat(8,5,CV_32F,Scalar(0));
		//-1 -1 2 -1 -1
		//-1 -1 2 -1 -1
		// 2  2 6  2  2
		//-1 -1 2 -1 -1
		//-1 -1 2 -1 -1
		//-1 -1 2 -1 -1
		//-1 -1 2 -1 -1
		//-1 -1 2 -1 -1
		for(int i=0;i<kernel_T.rows;i++)
		{
			for(int j=0;j<kernel_T.cols;j++)
			{
				if(i==2)
				{
					if(j==2)
						kernel_T.at<float>(i,j)=6;
					else
						kernel_T.at<float>(i,j)=2;
				}
				else
				{
					if(j==2)
						kernel_T.at<float>(i,j)=2;
					else
						kernel_T.at<float>(i,j)=-1;
				}
			}
		}
		break;

	case 2:
		kernel_T=Mat(5,8,CV_32F,Scalar(0));
		//-1 -1 2 -1 -1 -1 -1 -1
		//-1 -1 2 -1 -1 -1 -1 -1
		// 2  2 6  2  2  2  2  2
		//-1 -1 2 -1 -1 -1 -1 -1
		//-1 -1 2 -1 -1 -1 -1 -1

		for(int i=0;i<kernel_T.rows;i++)
		{
			for(int j=0;j<kernel_T.cols;j++)
			{
				if(i==2)
				{
					if(j==2)
						kernel_T.at<float>(i,j)=6;
					else
						kernel_T.at<float>(i,j)=2;
				}
				else
				{
					if(j==2)
						kernel_T.at<float>(i,j)=2;
					else
						kernel_T.at<float>(i,j)=-1;
				}
			}
		}
		break;
	case 3:
		kernel_T=Mat(11,11,CV_32F,Scalar(0));
		// 0  0  0 -1 0 2 0 -1  0  0  0
		//-1 -1 -1  5 0 2 0 -5 -1 -1 -1
		// 0  0  0  0 0 2 0  0  0  0  0
		// 2  2  2  2 2 4 2  2  2  2  2
		// 0  0  0  0 0 2 0  0  0  0  0
		//-1 -1 -1  5 0 2 0 -5 -1 -1 -1
		// 0  0  0 -1 0 2 0 -1  0  0  0
		// 0  0  0 -1 0 2 0 -1  0  0  0
		// 0  0  0 -1 0 2 0 -1  0  0  0
		// 0  0  0 -1 0 2 0 -1  0  0  0
		// 0  0  0 -1 0 2 0 -1  0  0  0
		for(int i=0;i<kernel_T.rows;i++)
		{
			for(int j=0;j<kernel_T.cols;j++)
			{
				
				if(i==3)
				{
					if(j==5)
						kernel_T.at<float>(i,j)=4;
					else
						kernel_T.at<float>(i,j)=2;
				}

				else if(i==1||i==5)
				{
					if(j==5)
						kernel_T.at<float>(i,j)=2;
					else if(j==3||j==7)
						kernel_T.at<float>(i,j)=-5;
					else if(j==4||j==6)
						kernel_T.at<float>(i,j)=0;
					else
						kernel_T.at<float>(i,j)=-1;
				}
				else if(i==2||i==4)
				{
					if(j==5)
						kernel_T.at<float>(i,j)=2;
				}
				else
				{
					if(j==3||j==7)
						kernel_T.at<float>(i,j)=-1;
					else if(j==5)
						kernel_T.at<float>(i,j)=2;
				}
			}
		}
		break;
	case 4:
		kernel_T=Mat(7,11,CV_32F,Scalar(0));
		// 0  0  0 -1 0 2 0 -1  0  0  0
		//-1 -1 -1  5 0 2 0 -5 -1 -1 -1
		// 0  0  0  0 0 2 0  0  0  0  0
		// 2  2  2  2 2 4 2  2  2  2  2
		// 0  0  0  0 0 2 0  0  0  0  0
		//-1 -1 -1  5 0 2 0 -5 -1 -1 -1
		// 0  0  0 -1 0 2 0 -1  0  0  0

		for(int i=0;i<kernel_T.rows;i++)
		{
			for(int j=0;j<kernel_T.cols;j++)
			{
				
				if(i==3)
				{
					if(j==5)
						kernel_T.at<float>(i,j)=4;
					else
						kernel_T.at<float>(i,j)=2;
				}

				else if(i==1||i==5)
				{
					if(j==5)
						kernel_T.at<float>(i,j)=2;
					else if(j==3||j==7)
						kernel_T.at<float>(i,j)=-5;
					else if(j==4||j==6)
						kernel_T.at<float>(i,j)=0;
					else
						kernel_T.at<float>(i,j)=-1;
				}
				else if(i==2||i==4)
				{
					if(j==5)
						kernel_T.at<float>(i,j)=2;
				}
				else
				{
					if(j==3||j==7)
						kernel_T.at<float>(i,j)=-1;
					else if(j==5)
						kernel_T.at<float>(i,j)=2;
				}
			}
		}
		break;
	case 5:
		kernel_T=Mat(12,7,CV_32F,Scalar(0));
		//-1 -1 0 2 0 -1 -1
		//-1 -1 0 2 0 -1 -1
		// 0  0 0 2 0  0  0
		// 2  2	2 2 2  2  2
		//-1 -1 0 2 0 -1 -1
		//-1 -1 0 2 0 -1 -1
		//-1 -1 0 2 0 -1 -1
		//-1 -1 0 2 0 -1 -1
		//-1 -1 0 2 0 -1 -1
		for(int i=0;i<kernel_T.rows;i++)
		{
			for(int j=0;j<kernel_T.cols;j++)
			{
				if(i==3)
					kernel_T.at<float>(i,j)=2;
				else if(i==2||i==4)
				{
					if(j==3)
						kernel_T.at<float>(i,j)=2;
				}
				else
				{
					if(j==3)
						kernel_T.at<float>(i,j)=2;
					else if(j==2||j==4)
						kernel_T.at<float>(i,j)=0;
					else
						kernel_T.at<float>(i,j)=-1;
				}
			}
		}
		break;
	}

	//cout<<"Kernel_T"<<endl<<kernel_T<<endl;
	return kernel_T;
}

Mat CreadorMascaras::getKernelL()
{
	kernel_L=Mat(7,7,CV_32F,Scalar(0));

	for(int i=0;i<kernel_L.rows;i++)
	{
		for(int j=0;j<kernel_L.cols;j++)
		{
			if(i==0)
				kernel_L.at<float>(i,j)=-1;
			else if(i==1)
			{
				if(j==6)
					kernel_L.at<float>(i,j)=-1;
			}
			else if(i==2)
			{
				if(j==4)
					kernel_L.at<float>(i,j)=6;
				else if(j==5)
					kernel_L.at<float>(i,j)=0;
				else if(j==6)
					kernel_L.at<float>(i,j)=-1;
				else
					kernel_L.at<float>(i,j)=2;
			}
			else if(i==3)
			{
				if(j==4)
					kernel_L.at<float>(i,j)=2;
				else if(j==6)
					kernel_L.at<float>(i,j)=-1;
			}
			else
			{
				if(j==4)
					kernel_L.at<float>(i,j)=2;
				else if(j==3||j==5)
					kernel_L.at<float>(i,j)=0;
				else
					kernel_L.at<float>(i,j)=-1;
			
			}
		}
	}
	//cout<<"Kernel_L"<<endl<<kernel_L<<endl;
	return kernel_L;

}

Mat CreadorMascaras::getKernelI(int tipo)
{
	switch(tipo)
	{
		case 1:
			kernel_I=Mat(9,3,CV_32F,Scalar(0));
			//-1 2 -1
			//-1 2 -1
			//-1 2 -1
			//-1 2 -1
			//-1 2 -1
			//-1 2 -1
			//-1 2 -1
			//-1 2 -1
			//-1 2 -1
			for(int i=0;i<kernel_I.rows;i++)
			{
				for(int j=0;j<kernel_I.cols;j++)
				{
					if(j==0||j==2)
						kernel_I.at<float>(i,j)=-1;
					else
						kernel_I.at<float>(i,j)=2;
				}
			}
			break;
		case 2:
			kernel_I=Mat(9,5,CV_32F,Scalar(0));
			//-1 0 2 0 -1
			//-1 0 2 0 -1
			//-1 0 2 0 -1
			//-1 0 2 0 -1
			//-1 0 2 0 -1
			//-1 0 2 0 -1
			//-1 0 2 0 -1
			//-1 0 2 0 -1
			//-1 0 2 0 -1
			for(int i=0;i<kernel_I.rows;i++)
			{
				for(int j=0;j<kernel_I.cols;j++)
				{
					if(j==0||j==4)
						kernel_I.at<float>(i,j)=-1;
					else if(j==2)
						kernel_I.at<float>(i,j)=2;
				}
			}
			break;
		case 3:
			kernel_I=Mat(9,5,CV_32F,Scalar(0));
			//1 0 -2 0 1
			//1 0 -2 0 1
			//1 0 -2 0 1
			//1 0 -2 0 1
			//1 0 -2 0 1
			//1 0 -2 0 1
			//1 0 -2 0 1
			//1 0 -2 0 1
			//1 0 -2 0 1
			for(int i=0;i<kernel_I.rows;i++)
			{
				for(int j=0;j<kernel_I.cols;j++)
				{
					if(j==0||j==4)
						kernel_I.at<float>(i,j)=1;
					else if(j==2)
						kernel_I.at<float>(i,j)=-2;
				}
			}
			break;
		case 4:
			kernel_I=Mat(3,5,CV_32F,Scalar(0));
			//-1 0 2 0 -1
			//-1 0 2 0 -1
			//-1 0 2 0 -1
			for(int i=0;i<kernel_I.rows;i++)
			{
				for(int j=0;j<kernel_I.cols;j++)
				{
					if(j==0||j==4)
						kernel_I.at<float>(i,j)=-1;
					else if(j==2)
						kernel_I.at<float>(i,j)=2;
				}
			}
			break;
		case 5:
			kernel_I=Mat(9,5,CV_32F,Scalar(0));
			//-1 0 1
			//-1 0 1
			//-1 0 1
			//-1 0 1
			//-1 0 1
			//-1 0 1
			//-1 0 1
			//-1 0 1
			//-1 0 1
			for(int i=0;i<kernel_I.rows;i++)
			{
				kernel_I.at<float>(i,0)=-1;
				kernel_I.at<float>(i,1)=0;
				kernel_I.at<float>(i,2)=1;
			}
			break;
		case 6:
			kernel_I=Mat(9,9,CV_32F,Scalar(0));
			//-1 0 0 0 2 0 0 0 -1
			//-1 0 0 0 2 0 0 0 -1
			//-1 0 0 0 2 0 0 0 -1
			//-1 0 0 0 2 0 0 0 -1
			//-1 0 0 0 2 0 0 0 -1
			//-1 0 0 0 2 0 0 0 -1
			//-1 0 0 0 2 0 0 0 -1
			//-1 0 0 0 2 0 0 0 -1
			//-1 0 0 0 2 0 0 0 -1
			for(int i=0;i<kernel_I.rows;i++)
			{
				for(int j=0;j<kernel_I.cols;j++)
				{
					if(j==0||j==8)
						kernel_I.at<float>(i,j)=-1;
					else if(j==4)
						kernel_I.at<float>(i,j)=2;
				}
			}
			break;
	}
	//cout<<"Kernel_I"<<endl<<kernel_I<<endl;
	return kernel_I;
}

Mat CreadorMascaras::getKernel_(int tipo)
{
	switch(tipo)
	{
	case 1:
		kernel_=Mat(3,9,CV_32F,Scalar(0));
		//-1 -1 -1 -1 -1 -1 -1 -1 -1
		// 2  2  2  2  2  2  2  2  2
		//-1 -1 -1 -1 -1 -1 -1 -1 -1 
		for(int i=0;i<kernel_.rows;i++)
		{
			for(int j=0;j<kernel_.cols;j++)
			{
				if(i==0||i==2)
					kernel_.at<float>(i,j)=-1;
				else
					kernel_.at<float>(i,j)=2;
			}
		}
		break;
	case 2:
		kernel_=Mat(5,9,CV_32F,Scalar(0));
		//-1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
		// 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
		// 2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2  2
		// 0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
		//-1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1 -1
		for(int i=0;i<kernel_.rows;i++)
		{
			for(int j=0;j<kernel_.cols;j++)
			{
				if(i==0||i==4)
					kernel_.at<float>(i,j)=-1;
				else if(i==2)
					kernel_.at<float>(i,j)=2;
			}
		}
		break;

	case 3:
		kernel_=Mat(5,3,CV_32F,Scalar(0));
		//-1 -1 -1 
		// 0  0  0
		// 2  2  2
		// 0  0  0
		//-1 -1 -1
		for(int i=0;i<kernel_.rows;i++)
		{
			for(int j=0;j<kernel_.cols;j++)
			{
				if(i==0||i==4)
					kernel_.at<float>(i,j)=-1;
				else if(i==2)
					kernel_.at<float>(i,j)=2;
			}
		}
		break;
	}
	//cout<<"Kernel_"<<endl<<kernel_<<endl;
	return kernel_;

}

Mat CreadorMascaras::getKernelCruz()
{
	//-1 -1 2 -1 -1
	//-1 -1 2 -1 -1
	// 2  2 0  2  2
	//-1 -1 2 -1 -1
	//-1 -1 2 -1 -1
	for(int i=0;i<kernel_cruz.rows;i++)
	{
		if(i==2)
		{
			for(int j=0;j<kernel_cruz.cols;j++)
			{
				if(j==2)
					kernel_cruz.at<float>(i,j)=0;
				else
					kernel_cruz.at<float>(i,j)=2;
			}
		}
		else
		{
			for(int j=0;j<kernel_cruz.cols;j++)
			{
				if(j==2)
					kernel_cruz.at<float>(i,j)=2;
				else
					kernel_cruz.at<float>(i,j)=-1;
			}
		}
	}
	//cout<<"Kernel_cruz"<<endl<<kernel_cruz<<endl;
	return kernel_cruz;
}

Mat CreadorMascaras::getKernelDiagDcha()
{
	//0   0 -7  0  3
	//0  -1  0  3  0
	//-7  0  3  0  0
	//0   3  0  0  0
	//3   0  0  0  0 
	for(int i=0;i<kernel_diag_dcha.rows;i++)
	{
		for(int j=0;j<kernel_diag_dcha.cols;j++)
		{
			if(i==0)
			{
				kernel_diag_dcha.at<float>(i,2)=-7;
				kernel_diag_dcha.at<float>(i,4)=3;
			}
			if(i==1)
			{
				kernel_diag_dcha.at<float>(i,1)=-1;
				kernel_diag_dcha.at<float>(i,3)=3;
			}
			if(i==2)
			{
				kernel_diag_dcha.at<float>(i,0)=-7;
				kernel_diag_dcha.at<float>(i,2)=3;
			}
			if(i==3)
				kernel_diag_dcha.at<float>(i,1)=3;
			if(i==4)
				kernel_diag_dcha.at<float>(i,0)=3;
		}
	}


	// 0  1  2
	//-1  0  1
	//-2 -1  0
	/*for(int i=0;i<kernel_diag_dcha.rows;i++)
	{
		for(int j=0;j<kernel_diag_dcha.cols;j++)
		{
			if(i==0)
			{
				kernel_diag_dcha.at<float>(i,1)=1;
				kernel_diag_dcha.at<float>(i,2)=2;
			}
			if(i==1)
			{
				kernel_diag_dcha.at<float>(i,0)=-1;
				kernel_diag_dcha.at<float>(i,2)=1;
			}
			if(i==2)
			{
				kernel_diag_dcha.at<float>(i,0)=-2;
				kernel_diag_dcha.at<float>(i,1)=-1;
			}
		}
	}*/
	//cout<<"Kernel_diag_dcha"<<endl<<kernel_diag_dcha<<endl;
	return kernel_diag_dcha;
}

Mat CreadorMascaras::getKernelX()
{
	//4 -2 -2 -2 4
	//-2 4 -2 4 -2
	//-2 -2 8 -2 -2
	//-2 4 -2 4 -2
	//4 -2 -2 -2 4
	for (int i=0; i<kernel_X.rows; i++) 
	{
		kernel_X.at<float>(i,i)= 4;
		kernel_X.at<float>(4-i,i)= 4;

	}
	kernel_X.at<float>(2,2)= 8;
	//cout<<"Kernel_X"<<endl<<kernel_X<<endl;
	return kernel_X;
}

Mat CreadorMascaras::getKernelUnsharp(float alpha)
{
	//          [-alpha alpha-1 -alpha]
	//1/alpha+1*[alpha-1 alpha+5 alpha-1]
	//          [-alpha alpha-1 -alpha]
	float coef=1/(alpha+1);
	for(int i=0;i<kernel_unsharp.rows;i++)
	{
		for(int j=0;j<kernel_unsharp.cols;j++)
		{
			if(i==0||i==2)
			{
				kernel_unsharp.at<float>(i,0)=-alpha;
				kernel_unsharp.at<float>(i,1)=alpha-1;
				kernel_unsharp.at<float>(i,2)=-alpha;
			}
			else
			{
				kernel_unsharp.at<float>(i,0)=alpha-1;
				kernel_unsharp.at<float>(i,1)=alpha+5;
				kernel_unsharp.at<float>(i,2)=alpha-1;
			}

		}
	}
	//cout<<"Kernel_Unsharp"<<endl<<kernel_unsharp<<endl;
	return coef*kernel_unsharp;

}


CreadorMascaras::~CreadorMascaras(void)
{
}
