#ifndef _CREADORMASCARAS_H_
#define _CREADORMASCARAS_H_

#include <opencv2/opencv.hpp>
using namespace cv;

class CreadorMascaras
{
private:
	Mat kernel_X;
	Mat kernel_T;
	Mat kernel_I;
	Mat kernel_L;
	Mat kernel_;
	Mat kernel_Z;
	Mat kernel_cruz;
	Mat kernel_diag_dcha;
	Mat kernel_unsharp;
public:
	Mat getKernelX();
	Mat getKernelT(int tipo);
	Mat getKernelI(int tipo);
	Mat getKernelL();
	Mat getKernel_(int tipo);
	Mat getKernelZ();
	Mat getKernelCruz();
	Mat getKernelDiagDcha();
	Mat getKernelUnsharp(float alpha);
	CreadorMascaras(void);
	~CreadorMascaras(void);
};

#endif

