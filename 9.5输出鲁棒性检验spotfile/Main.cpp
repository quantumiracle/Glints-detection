
#include "PupilDetection.h"

/* ���������в��� */
int PupilDetectionTest(void);
/* ���ڵ��� */
void DebugTest(void);

//int GlintProcessing(Mat &srcImg);

int main()
{
	/*
	Mat img = imread("1.bmp");
	cvtColor(img, img, CV_BGR2GRAY);
	threshold(img, img, 100, 255, CV_THRESH_BINARY_INV);
	imwrite("img.bmp", img);
	*/
	//���������в���
	PupilDetectionTest();

	system("pause");
	return 1;
}
