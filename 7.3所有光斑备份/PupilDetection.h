
/****************************************************************
*	��Ŀ��ͫ�׼�ⶨλ											*
*	���ߣ��޼��� ���Ӻ�											*
*		  ������ ������ 										*
*	���ʱ�䣺													*
*																*
****************************************************************/

#ifndef PUPILDETECTION_H
#define PUPILDETECTION_H

//#include "cv.h"
//#include "opencv2/cxcore.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <time.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <Windows.h>

#define PD_SUCCEED					0			//�ɹ�
#define PD_FAILED					1			//ʧ��
#define SHRINK_AREA_FAILED			2			//��Сͫ������ʧ��
#define PD_PI						3.1415926		
#define TWOPI						6.2831852

#define IMGHEIGHT					120			//ͼƬ�ߴ��һ���߶�
#define IMGWIDTH					160			//ͼƬ�ߴ��һ�����
#define SRCIMGHEIGHT				480			//ԭͼ�߶�
#define SRCIMGWIDTH					640			//ԭͼ���

#define SRCIMG_X_EXTENT				10			//ԭͼ��x�ĵ�����Χ
#define SRCIMG_Y_EXTENT				10			//ԭͼ��y�ĵ�����Χ
#define SRCIMG_A_EXTENT				12			//ԭͼ��a�ĵ�����Χ
#define SRCIMG_B_EXTENT				12			//ԭͼ��b�ĵ�����Χ
#define SRCIMG_ALPHA_EXTENT			1			//ԭͼ��alpha�ĵ�����Χ
#define SRCIMG_FRAME_THRESHOLD		30			//ԭͼ�߿��С

#define FRAME_WIDTH ((double)IMGWIDTH / SRCIMGWIDTH) * SRCIMG_FRAME_THRESHOLD			//��ȡͼƬ���߱߿�Ŀ��
#define A_B_THRESHOLD				3			//��Բ����b����С����
#define RATIO_THRESHOLD				0.2			//��Բ�����볤��֮�ȵ���Сֵ
#define THETA_INCREMENT				0.05		//������ԲԲ�ܻҶ�ֵ��ƽ��ֵʱthetaֵ������
#define ALPHA_INCREMENT				0.3			//������Բalpha�Ƕ�ֵ������

#define ADJUST_THRESHOLD			0			//�Ƿ������������ֵ	
//#define G_L_THRESHOLD				195			//ͼ��������ֵ
#define SHRINK_AREA_MINVAL_TH		0.3			//��Сͫ������minValֵ����ֵ

#define CONTOURS_MIN_SIZE			2			//���Բ�ܵ�����ٸ���
#define CONTOURS_MAX_SIZE			300			//���Բ�ܵ��������
#define GET_POINT_OFFSET			1			//��ߴ���ʱ����ȡ��ƫ����
#define G_L_OFFSET					4			//������G_L_OFFSETֵ��Ϊ��������
#define G_L_DIFF_TH					10			//������ʱ��4�������ʱ��ֵ����ֵ

#define IMG_MIN_POINT				20			//Сͼ��Բ������ٸ�����С�ڸ�ֵ�򲻿���
#define SRCIMG_MIN_POINT			50			//ԭͼ��Բ������ٸ�����С�ڸ�ֵ�򲻿���

//#define OFFSET					2			//Сͼoffsetֵ
#define OUT_OFFSET					2			//ԭͼ����Բoffsetֵ
#define IN_OFFSET					3			//ԭͼ����Բoffsetֵ

#define SEPARATION_OF_TONES_NUM		20			//ȷ��ͫ�״��·�Χʱ�ָ�Ҷ�ֵ�ĸ���
#define CENTER_THRESHOLD_EXTENT		30			//��Բ���ĻҶ�ֵ��ֵ��Χ

using namespace std;  
using namespace cv; 

//ͫ�״��¶�λʱ���̱߳������
typedef struct		
{
	double coarsePosValAccum;
	int coarsePosThreshold;

	int minX;
	int maxX;
	int minY;
	int maxY;

	double radius;
}PDCoarPosThVal;

//��Բ����
typedef struct		
{
	int x;			//���ĺ�����
	int y;			//����������
	int a;			//����
	int b;			//����
	double alpha;	//ƫת�Ƕ�
}PDEllipse;

//��Բ������ֵ
typedef struct		
{
	double maxVal;	//������ֵ
	int x;			//���ĺ�����
	int y;			//����������
	int a;			//����
	int b;			//����
	double alpha;	//ƫת�Ƕ�
}PDEllipse_DiffVal;

//�ݶ�����ĸ�������ʱֵ
typedef struct		
{
	int outPointVal;
	int inPointVal;

}PDPointVal;

//���߳�ʱ���߳��ڹ�������
typedef struct		
{

	int minX;
	int maxX;
	int minY;
	int maxY;

	int startA;
	int startB;
	int startX;
	int startY;
	int startAlphaK;

	int endA;
	int endB;
	int endX;
	int endY;
	int endAlphaK;

}PDThreadPublicVal;

class PupilDetection
{
public:
	//�ӿ�
	PupilDetection();
	~PupilDetection();
	/* ����ͼ�� */
	int setImg(Mat &_img);
	/* ����ͫ�׼�ⶨλ����⵽����Բ����ellipse�� */
	int DetectPupil(void);
	/* ����ellipse */
	void getEllipse(PDEllipse &_ellipse);
	/* ����ͼ�� */
	int getImg(Mat &_img);
	void getstr(string &_str);
	/* ԭͼ����������̺߳��� */
	static DWORD WINAPI m_threadFun(LPVOID lpParameter);
	/* ԭͼ����������̺߳��� */
	static DWORD WINAPI m_srcThreadFun(LPVOID lpParameter);
	/* ͫ�״ֶ�λ����������̺߳��� */
	static DWORD WINAPI PupilCoarsePositioning(LPVOID threadIndex);

	static int m_coarsePosThreshold;	//����Ӧ��ֵ����ֵ
	static int G_L_THRESHOLD;
private:
	/* �Դ���ͼ����д��� */
	int ImgProcessing(void);
	/* ����ͼ���ϸ���4��������ݶ�ֵ */
	void GradCalcu(void);
	/* �������a, b, alpha�µ���ԲԲ�ܸ��� */
	void PeripheryPointCalcu(const int a, const int b, const int alphaK);
	/* ���������Բ��ƫ�����µĲ�ּ��� */
	static inline double EllipDiffCalcu(const int _x, const int _y, const int count, int *points);
	/* ������Բ */
	void PDDrawEllipse(Mat &_img, int imgWidth, int imgHeight);
	/* ԭͼ������� */
	void SrcImgDetectPupil(void);
	/* ������Բ�߽����ص��ݶ�ƽ��ֵ */
	static inline double SrcImgEllipseDiffCalcu(const int _x, const int _y, const int count, PDPointVal *points);
	/* �������a, b, alpha�µ���ԲԲ�ܸ��� */
	void SrcImgPeripheryPointCalcu(const int a, const int b, const int alphaK);
	/* �����������ȷ����ֱ�� A * x + B * y + C = 0 */
	void LinearEquationCalcu(CvPoint &point1, CvPoint &point2, int &A, int &B, int &C);
	/* �Թ������������,�� A*x + B*y + C < 0�������filledVal1 */
	void SpotAreaFill(Mat &img, CvPoint leftTopPoint, CvPoint rightBottomPoint, int filledVal1, int filledVal2,
		int A, int B, int C);
	/* ����ͫ�״ֶ�λʱ������ɸѡ */
	static bool ContourScreening(vector<Point> &contours, double &valueAccum, Point &circleCenter, int &circleRadius);
	void PDDrawCircle(Mat &img, int r, int _x, int _y);

	//���ݱ���
	PDEllipse m_ellipse;		//��Բ
	Mat m_img;					//ͼ��(Сͼ)
	Mat m_srcImg;				//ԭͼ
	string m_str;
	
	static Mat m_staticImg;
	static Mat m_staticSrcImg;
	static PDThreadPublicVal m_threadPublicVal;			//����ԭͼ����ʱ�̺߳�����Ĺ�������
	static bool *m_threadEndLabel;
	static bool m_srcThreadEndLabel[2 * SRCIMG_A_EXTENT + 1];		//�߳̽�����־

	static int *(*m_peripheryPoints);		//Сͼ���Զ�λʱ���������м�ֵ
	static PDPointVal *(*m_srcPeripheryPoints);		//ԭͼ����ʱ���������м�ֵ
	static PDEllipse_DiffVal *m_threadEllipse_DiffVal;
	static PDEllipse_DiffVal m_srcThreadEllipse_DiffVal[2 * SRCIMG_A_EXTENT + 1];		//��������̵߳õ�����Բ������
	static int m_grads[IMGHEIGHT * IMGWIDTH];		//�ݶ�ֵ��Сͼ��

	static Mat m_coarsePosImg;
	static double m_coarsePosValAccum;
	
	static bool m_coarsePosThLabel[SEPARATION_OF_TONES_NUM + 1];
	static PDCoarPosThVal m_coarsePosThVals[SEPARATION_OF_TONES_NUM + 1];

	static int m_offset;	
	/*static int m_outOffset;
	static int m_inOffset;*/

	double *m_cosTheta;	//���cos theta��ֵ������
	double *m_sinTheta;	//���sin theta��ֵ������
	double *m_cosAlpha;	//���cos alpha��ֵ������
	double *m_sinAlpha;	//���sin alpha��ֵ������
};

#endif