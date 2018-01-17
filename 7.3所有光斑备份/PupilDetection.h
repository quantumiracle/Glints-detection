
/****************************************************************
*	项目：瞳孔检测定位											*
*	作者：罗家意 丁子涵											*
*		  田雨沛 章泽宇 										*
*	完成时间：													*
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

#define PD_SUCCEED					0			//成功
#define PD_FAILED					1			//失败
#define SHRINK_AREA_FAILED			2			//缩小瞳孔区域失败
#define PD_PI						3.1415926		
#define TWOPI						6.2831852

#define IMGHEIGHT					120			//图片尺寸归一化高度
#define IMGWIDTH					160			//图片尺寸归一化宽度
#define SRCIMGHEIGHT				480			//原图高度
#define SRCIMGWIDTH					640			//原图宽度

#define SRCIMG_X_EXTENT				10			//原图对x的调整范围
#define SRCIMG_Y_EXTENT				10			//原图对y的调整范围
#define SRCIMG_A_EXTENT				12			//原图对a的调整范围
#define SRCIMG_B_EXTENT				12			//原图对b的调整范围
#define SRCIMG_ALPHA_EXTENT			1			//原图对alpha的调整范围
#define SRCIMG_FRAME_THRESHOLD		30			//原图边框大小

#define FRAME_WIDTH ((double)IMGWIDTH / SRCIMGWIDTH) * SRCIMG_FRAME_THRESHOLD			//截取图片两边边框的宽度
#define A_B_THRESHOLD				3			//椭圆短轴b的最小长度
#define RATIO_THRESHOLD				0.2			//椭圆短轴与长轴之比的最小值
#define THETA_INCREMENT				0.05		//计算椭圆圆周灰度值的平均值时theta值的增量
#define ALPHA_INCREMENT				0.3			//遍历椭圆alpha角度值的增量

#define ADJUST_THRESHOLD			0			//是否调整处理光斑阈值	
//#define G_L_THRESHOLD				195			//图像处理光斑阈值
#define SHRINK_AREA_MINVAL_TH		0.3			//缩小瞳孔区域minVal值得阈值

#define CONTOURS_MIN_SIZE			2			//光斑圆周点的最少个数
#define CONTOURS_MAX_SIZE			300			//光斑圆周点的最多个数
#define GET_POINT_OFFSET			1			//光斑处理时外扩取点偏移量
#define G_L_OFFSET					4			//光斑外的G_L_OFFSET值认为是正常点
#define G_L_DIFF_TH					10			//处理光斑时对4个点分类时差值的阈值

#define IMG_MIN_POINT				20			//小图椭圆点的最少个数，小于该值则不考虑
#define SRCIMG_MIN_POINT			50			//原图椭圆点的最少个数，小于该值则不考虑

//#define OFFSET					2			//小图offset值
#define OUT_OFFSET					2			//原图外椭圆offset值
#define IN_OFFSET					3			//原图内椭圆offset值

#define SEPARATION_OF_TONES_NUM		20			//确定瞳孔大致范围时分割灰度值的个数
#define CENTER_THRESHOLD_EXTENT		30			//椭圆中心灰度值阈值范围

using namespace std;  
using namespace cv; 

//瞳孔大致定位时各线程保存变量
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

//椭圆参数
typedef struct		
{
	int x;			//中心横坐标
	int y;			//中心纵坐标
	int a;			//长轴
	int b;			//短轴
	double alpha;	//偏转角度
}PDEllipse;

//椭圆及其差分值
typedef struct		
{
	double maxVal;	//差分最大值
	int x;			//中心横坐标
	int y;			//中心纵坐标
	int a;			//长轴
	int b;			//短轴
	double alpha;	//偏转角度
}PDEllipse_DiffVal;

//梯度运算的各个点临时值
typedef struct		
{
	int outPointVal;
	int inPointVal;

}PDPointVal;

//多线程时各线程内公共变量
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
	//接口
	PupilDetection();
	~PupilDetection();
	/* 传入图像 */
	int setImg(Mat &_img);
	/* 进行瞳孔检测定位，检测到的椭圆存入ellipse中 */
	int DetectPupil(void);
	/* 返回ellipse */
	void getEllipse(PDEllipse &_ellipse);
	/* 传出图像 */
	int getImg(Mat &_img);
	void getstr(string &_str);
	/* 原图并发计算的线程函数 */
	static DWORD WINAPI m_threadFun(LPVOID lpParameter);
	/* 原图并发计算的线程函数 */
	static DWORD WINAPI m_srcThreadFun(LPVOID lpParameter);
	/* 瞳孔粗定位并发计算的线程函数 */
	static DWORD WINAPI PupilCoarsePositioning(LPVOID threadIndex);

	static int m_coarsePosThreshold;	//自适应二值化阈值
	static int G_L_THRESHOLD;
private:
	/* 对传入图像进行处理 */
	int ImgProcessing(void);
	/* 计算图像上各点4个方向的梯度值 */
	void GradCalcu(void);
	/* 计算给定a, b, alpha下的椭圆圆周各点 */
	void PeripheryPointCalcu(const int a, const int b, const int alphaK);
	/* 计算给定椭圆及偏移量下的差分计算 */
	static inline double EllipDiffCalcu(const int _x, const int _y, const int count, int *points);
	/* 绘制椭圆 */
	void PDDrawEllipse(Mat &_img, int imgWidth, int imgHeight);
	/* 原图调整检测 */
	void SrcImgDetectPupil(void);
	/* 计算椭圆边界像素点梯度平均值 */
	static inline double SrcImgEllipseDiffCalcu(const int _x, const int _y, const int count, PDPointVal *points);
	/* 计算给定a, b, alpha下的椭圆圆周各点 */
	void SrcImgPeripheryPointCalcu(const int a, const int b, const int alphaK);
	/* 计算给定两点确定的直线 A * x + B * y + C = 0 */
	void LinearEquationCalcu(CvPoint &point1, CvPoint &point2, int &A, int &B, int &C);
	/* 对光斑区域进行填充,对 A*x + B*y + C < 0区域填充filledVal1 */
	void SpotAreaFill(Mat &img, CvPoint leftTopPoint, CvPoint rightBottomPoint, int filledVal1, int filledVal2,
		int A, int B, int C);
	/* 进行瞳孔粗定位时轮廓的筛选 */
	static bool ContourScreening(vector<Point> &contours, double &valueAccum, Point &circleCenter, int &circleRadius);
	void PDDrawCircle(Mat &img, int r, int _x, int _y);

	//数据变量
	PDEllipse m_ellipse;		//椭圆
	Mat m_img;					//图像(小图)
	Mat m_srcImg;				//原图
	string m_str;
	
	static Mat m_staticImg;
	static Mat m_staticSrcImg;
	static PDThreadPublicVal m_threadPublicVal;			//保存原图调整时线程函数间的公共变量
	static bool *m_threadEndLabel;
	static bool m_srcThreadEndLabel[2 * SRCIMG_A_EXTENT + 1];		//线程结束标志

	static int *(*m_peripheryPoints);		//小图粗略定位时计算各点的中间值
	static PDPointVal *(*m_srcPeripheryPoints);		//原图调整时计算各点的中间值
	static PDEllipse_DiffVal *m_threadEllipse_DiffVal;
	static PDEllipse_DiffVal m_srcThreadEllipse_DiffVal[2 * SRCIMG_A_EXTENT + 1];		//保存各个线程得到的椭圆及其差分
	static int m_grads[IMGHEIGHT * IMGWIDTH];		//梯度值，小图用

	static Mat m_coarsePosImg;
	static double m_coarsePosValAccum;
	
	static bool m_coarsePosThLabel[SEPARATION_OF_TONES_NUM + 1];
	static PDCoarPosThVal m_coarsePosThVals[SEPARATION_OF_TONES_NUM + 1];

	static int m_offset;	
	/*static int m_outOffset;
	static int m_inOffset;*/

	double *m_cosTheta;	//存放cos theta的值的数组
	double *m_sinTheta;	//存放sin theta的值的数组
	double *m_cosAlpha;	//存放cos alpha的值的数组
	double *m_sinAlpha;	//存放sin alpha的值的数组
};

#endif