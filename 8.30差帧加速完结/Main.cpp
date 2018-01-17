
#include "PupilDetection.h"

/* 对样本进行测试 */
int PupilDetectionTest(void);
/* 用于调试 */
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
	//对样本进行测试
	PupilDetectionTest();

	system("pause");
	return 1;
}
