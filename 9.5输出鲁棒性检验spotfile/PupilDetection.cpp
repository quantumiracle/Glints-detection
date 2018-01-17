
#include "PupilDetection.h"

Mat PupilDetection::m_staticImg;
int ** PupilDetection::m_peripheryPoints = NULL;
bool *PupilDetection::m_threadEndLabel = NULL;
PDEllipse_DiffVal *PupilDetection::m_threadEllipse_DiffVal = NULL;
int PupilDetection::m_grads[IMGHEIGHT * IMGWIDTH];

int PupilDetection::m_coarsePosThreshold = 0;
int PupilDetection::G_L_THRESHOLD=0;
int PupilDetection::last_facula[4][2];
Mat PupilDetection::m_coarsePosImg;
double PupilDetection::m_coarsePosValAccum;
bool PupilDetection::m_coarsePosThLabel[SEPARATION_OF_TONES_NUM + 1];
PDCoarPosThVal PupilDetection::m_coarsePosThVals[SEPARATION_OF_TONES_NUM + 1];

Mat PupilDetection::m_staticSrcImg;
PDPointVal ** PupilDetection::m_srcPeripheryPoints = NULL;
PDThreadPublicVal PupilDetection::m_threadPublicVal;
PDEllipse_DiffVal PupilDetection::m_srcThreadEllipse_DiffVal[2 * SRCIMG_A_EXTENT + 1];
bool PupilDetection::m_srcThreadEndLabel[2 * SRCIMG_A_EXTENT + 1];
int PupilDetection::static_para_signal=0;
float PupilDetection::SPOT_RATIO_THRESHOLD = 0.001;
int  PupilDetection::SPOT_GREYVALUE_DIFF = 60;
int PupilDetection::m_offset;
//int PupilDetection::OUT_OFFSET;
//int PupilDetection::IN_OFFSET;

extern bool showImg;

void PDDrawRect(Mat &img, Point pt1, Point pt2);

PupilDetection::PupilDetection()
{
	/*
	//�õ�theta����
	int thetaLen = TWOPI / THETA_INCREMENT;
	int k = 0;
	m_cosTheta = new double[thetaLen];
	m_sinTheta = new double[thetaLen];
	for (k = 0; k < thetaLen; k ++)
	{
		m_cosTheta[k] = cos(THETA_INCREMENT * k);
		m_sinTheta[k] = sin(THETA_INCREMENT * k);
	}

	//�õ�alpha����
	int alphaLen = PD_PI / ALPHA_INCREMENT;
	m_cosAlpha = new double[alphaLen + 1];
	m_sinAlpha = new double[alphaLen + 1];
	for (k = 0; k < alphaLen + 1; k ++)
	{
		m_cosAlpha[k] = cos(ALPHA_INCREMENT * k);
		m_sinAlpha[k] = sin(ALPHA_INCREMENT * k);
	}

	m_ellipse.a = 0;
	m_ellipse.b = 0;
	m_ellipse.x = 0;
	m_ellipse.y = 0;
	m_ellipse.alpha = 0;

	//�������Բ�ܵ�����
	//Сͼ
	const int LEN = (IMGHEIGHT / 2 + 1) * (IMGHEIGHT / 2 + 1) * (int)(PD_PI / ALPHA_INCREMENT + 1);
	m_peripheryPoints = new int *[LEN];
	for (int i = LEN - 1; i >= 0; i --)
	{
		m_peripheryPoints[i] = NULL;
	}
	const int MAX_ALPHA_K = PD_PI / ALPHA_INCREMENT + 1;
	for (int a = 1; a < IMGHEIGHT / 2 + 1; a ++)
	{
		for (int b = a * RATIO_THRESHOLD; b <= a; b ++)
		{

			for (int alphaK = 0; alphaK < MAX_ALPHA_K; alphaK ++)
			{
				//�������a, b, alpha�µ���ԲԲ�ܸ���
				PeripheryPointCalcu(a, b, alphaK);
			}
		}
	}
	
	//ԭͼ
	const int SRC_LEN = (SRCIMGHEIGHT / 2 + 1) * (SRCIMGHEIGHT / 2 + 1) * (int)(PD_PI / ALPHA_INCREMENT + 1);
	m_srcPeripheryPoints = new PDPointVal *[SRC_LEN];
	for (int i = SRC_LEN - 1; i >= 0; i --)
	{
		m_srcPeripheryPoints[i] = NULL;
	}

	for (int a = 1; a < SRCIMGHEIGHT / 2 + 1; a ++)
	{
		for (int b = a * RATIO_THRESHOLD; b <= a; b ++)
		{

			for (int alphaK = 0; alphaK < MAX_ALPHA_K; alphaK ++)
			{
				//�������a, b, alpha�µ���ԲԲ�ܸ���
				SrcImgPeripheryPointCalcu(a, b, alphaK);
			}
		}
	}*/

}

PupilDetection::~PupilDetection()
{

	
	
	//�������Բ�ܵ�����
	//Сͼ
	/*const int LEN = (IMGHEIGHT / 2 + 1) * (IMGHEIGHT / 2 + 1) * (int)(PD_PI / ALPHA_INCREMENT + 1);
	for (int i = LEN - 1; i >= 0; i --)
	{
		if (NULL != m_peripheryPoints[i])
		{
			delete []m_peripheryPoints[i];
			m_peripheryPoints[i] = NULL;
		}		
	}
	
	//ԭͼ
	const int SRC_LEN = (SRCIMGHEIGHT / 2 + 1) * (SRCIMGHEIGHT / 2 + 1) * (int)(PD_PI / ALPHA_INCREMENT + 1);
	for (int i = SRC_LEN - 1; i >= 0; i --)
	{
		if (NULL != m_srcPeripheryPoints[i])
		{
			delete []m_srcPeripheryPoints[i];
			m_srcPeripheryPoints[i] = NULL;
		}
	}
	
	if (NULL != m_peripheryPoints)
	{
		delete []m_peripheryPoints;
		m_peripheryPoints = NULL;
	}
	
	if (NULL != m_srcPeripheryPoints)
	{
		delete []m_srcPeripheryPoints;
		m_srcPeripheryPoints = NULL;
	}
	*/
}

/* ����ͼ�� */
int PupilDetection::setImg(Mat &_img)
{
	_img.copyTo(m_img);
	return PD_SUCCEED;

}

void PupilDetection::getstr(string &_str)
{
	_str = m_str;

	return;
}

/* ����ellipse */
void PupilDetection::getEllipse(PDEllipse &_ellipse)
{
	_ellipse.x = m_ellipse.x;
	_ellipse.y = m_ellipse.y;
	_ellipse.a = m_ellipse.a;
	_ellipse.b = m_ellipse.b;
	_ellipse.alpha = m_ellipse.alpha;

	return ;
}


/* ����ͼ���ϸ�����ݶ�ֵ */
void PupilDetection::GradCalcu(void)      /*resize��Сͼ���е������ݶ�ֵ*/
{

	
} 

int PupilDetection::framediff(Mat img, Mat last_img){
	int i = 0,greyvalue_img=0,greyvalue_last_img=0;
	float avr_diff = 0,diff_avr=0;
	int pixels = img.size[0] * img.size[1];
	for (i = 0; i < pixels; i++){
		avr_diff = float(avr_diff*i + abs(img.data[i] - last_img.data[i])) / float(i + 1);
		greyvalue_img += img.data[i];
		greyvalue_last_img += last_img.data[i];
		//avr_diff += abs(img.data[i] - last_img.data[i]);
	}
	diff_avr = float(abs(greyvalue_img - greyvalue_last_img))/pixels;
	cout << diff_avr << endl;
	//cout << avr_diff << endl;
	if (diff_avr < 10 && avr_diff < 5)
		return 1;
	else{
		return 0;
		//cout << diff_avr << '  ' << avr_diff << endl;
		//waitKey(0);
	}
}



/* ����ͫ�׼�ⶨλ����⵽����Բ����ellipse�� */
int PupilDetection::DetectPupil(int signal)
{
	static_para_signal = signal;
	if (NULL == m_img.data)
	{
#if _DEBUG
		cout << "ͼ��Ϊ�գ�" << endl;
#endif
		return PD_FAILED;
	}
	
	//����ͼ��
	int temp = ImgProcessing();
	if (PD_FAILED == temp)
	{
#if _DEBUG
		cout << "ͼ����ʧ�ܣ�" << endl;
#endif
		return PD_FAILED;
	}
	else if (SHRINK_AREA_FAILED == temp)
	{
#if _DEBUG
		cout << "��Сͫ�׶�λ����ʧ�ܣ����ܵ�ԭ���Ǹ�ͼ��ͫ�������������ԡ� " << endl;
#endif
		//return SHRINK_AREA_FAILED;
	}
//#if SHOW_IMG
	if (showImg)
	{
		PDDrawRect(m_srcImg, Point(m_threadPublicVal.minX, m_threadPublicVal.minY), Point(m_threadPublicVal.maxX, m_threadPublicVal.maxY));
		imshow("m_srcImg", m_srcImg);
		waitKey(0);
	}
//#endif
	
	//����ͼ���ϸ���2��������ݶ�ֵ
	//GradCalcu();

	m_img.copyTo(m_staticImg);

	/*const double srcimg_img_ratio = (double)IMGWIDTH / SRCIMGWIDTH;
	const int MAX_A = min(m_threadPublicVal.maxX - m_threadPublicVal.minX, m_threadPublicVal.maxY - m_threadPublicVal.minY)
		* srcimg_img_ratio / 2 - m_offset;	
	if (MAX_A < m_offset)
	{
		return PD_FAILED;
	}
	m_threadEllipse_DiffVal = new PDEllipse_DiffVal[MAX_A - OUT_OFFSET];

	m_threadEndLabel = new bool[MAX_A - OUT_OFFSET];
	for (int i = 0; i < MAX_A - OUT_OFFSET; i ++)
	{
		m_threadEllipse_DiffVal[i].maxVal = 0;
		m_threadEndLabel[i] = false;
	}

	HANDLE *handle = new HANDLE[MAX_A - OUT_OFFSET];
	for (int a = OUT_OFFSET + 1, i = 0; a <= MAX_A; a ++, i ++)
	{
		handle[i] = CreateThread(NULL, 0, m_threadFun, (LPVOID)a, 0, NULL);
		CloseHandle(handle[i]);	
	}//a

	for (int i = 0; i < MAX_A - OUT_OFFSET; i ++)
	{
		//�ȴ��������߳̽���
		while (!m_threadEndLabel[i]);

	}

	double maxVal = -1;
	for (int i = 0; i < MAX_A - OUT_OFFSET; i ++)
	{
		if (maxVal < m_threadEllipse_DiffVal[i].maxVal)     //Сͼ��Բ
		{
			maxVal = m_threadEllipse_DiffVal[i].maxVal;
			m_ellipse.a = m_threadEllipse_DiffVal[i].a;
			m_ellipse.b = m_threadEllipse_DiffVal[i].b;
			m_ellipse.x = m_threadEllipse_DiffVal[i].x;
			m_ellipse.y = m_threadEllipse_DiffVal[i].y;
			m_ellipse.alpha = m_threadEllipse_DiffVal[i].alpha;
		}
	}

	delete []m_threadEllipse_DiffVal;
	m_threadEllipse_DiffVal = NULL;
	delete[]m_threadEndLabel;
	m_threadEndLabel = NULL;
	delete[]handle;
	handle = NULL;

	//�滭��Բ
//#if SHOW_IMG
	if (showImg)
	{
		PDDrawEllipse(m_img, IMGWIDTH, IMGHEIGHT);
		resize(m_img, m_img, Size(SRCIMGWIDTH, SRCIMGHEIGHT));
		imshow("m_img", m_img);
		waitKey(0);
	}
//#endif
	*///ԭͼ�������
	SrcImgDetectPupil();    //�Ŵ��ԭͼ
	
	return PD_SUCCEED;
}

/* ԭͼ����������̺߳��� */
DWORD WINAPI PupilDetection::m_threadFun(LPVOID lpParameter)
{
	
	return 1;

}

/* ������Բ�߽����ص��ݶ�ƽ��ֵ */
double PupilDetection::EllipDiffCalcu(const int _x, const int _y, const int count, int *points)
{
	
	return 1;
}

/* �������a, b, alpha�µ���ԲԲ�ܸ��� */
void PupilDetection::PeripheryPointCalcu(const int a, const int b, const int alphaK)
{
	int x = -1, y = -1;
	double d_x = -1, d_y = -1;
	int oldX = -1, oldY = -1;
	int tempPointVal;
	int count = 5;

	int minX = 0;
	int maxX = 0;
	int minY = 0;
	int maxY = 0;

	const double COS_OUT_B = m_cosAlpha[alphaK] * b;
	const double COS_OUT_A = m_cosAlpha[alphaK] * a;
	const double SIN_OUT_B = m_sinAlpha[alphaK] * b;
	const double SIN_OUT_A = m_sinAlpha[alphaK] * a;

	const int MAX_K = TWOPI / THETA_INCREMENT;
	int *tempPoints = new int[MAX_K + 5];

	for (int k = 0; k < MAX_K; k ++)
	{
		//����Բ�ϵ�����ļ���
		d_x = COS_OUT_B * m_cosTheta[k] - SIN_OUT_A * m_sinTheta[k];
		d_y = SIN_OUT_B * m_cosTheta[k] + COS_OUT_A * m_sinTheta[k];

		x = cvRound(d_x);
		y = cvRound(d_y);

		//ֻͳ���µ�����
		if (x != oldX || y != oldY)
		{
		
			//��
			if (x < 0)
			{
				if (x < minX)minX = x;						
			}
			//��
			else 
			{
				if (x > maxX)maxX = x;
			}

			//��
			if (y < 0)
			{
				if (y < minY)minY = y;						
			}
			//��
			else 
			{
				if (y > maxY)maxY = y;
			}											
			
			tempPoints[count] = y * IMGWIDTH + x;
			count ++;

			oldX = x;
			oldY = y;
		}
	
		
	}
	count -= 5;
	if (count < SRCIMG_MIN_POINT)
	{
		delete[]tempPoints;
		tempPoints = NULL;
		return;
	}
	else
	{
		tempPoints[0] = count;

		tempPoints[1] = minX;
		tempPoints[2] = maxX;

		tempPoints[3] = minY;
		tempPoints[4] = maxY;
	}

	m_peripheryPoints[a * (int)(IMGHEIGHT / 2 + 1) * (int)(PD_PI / ALPHA_INCREMENT + 1) + b * (int)(PD_PI / ALPHA_INCREMENT + 1) + alphaK] = tempPoints;

}

/* ����ͼ�� */
int PupilDetection::getImg(Mat &_img)
{
	m_srcImg.copyTo(_img);

	return PD_SUCCEED;
}

/* ������Բ */
void PupilDetection::PDDrawEllipse(Mat &_img, int imgWidth, int imgHeight)
{
	int oldX = 0;
	int oldY = 0;
	int count = 0;
	double valueAccum = 0;

	if (m_ellipse.alpha < 0)
	{
		m_ellipse.alpha += PD_PI;
	}

	int centerX = m_ellipse.x;
	int centerY = m_ellipse.y;
	int a = m_ellipse.a;
	int b = m_ellipse.b;
	int alphaK = m_ellipse.alpha / ALPHA_INCREMENT;

	int x = -1, y = -1;
	double d_x = -1, d_y = -1;
	const double COS_B = m_cosAlpha[alphaK] * b;
	const double COS_A = m_cosAlpha[alphaK] * a;
	const double SIN_B = m_sinAlpha[alphaK] * b;
	const double SIN_A = m_sinAlpha[alphaK] * a;
	const int MAX_K = TWOPI / THETA_INCREMENT;

	//����
	_img.data[centerY * imgWidth + centerX] = 255;
	_img.data[(centerY + 1) * imgWidth + centerX] = 255;
	_img.data[(centerY - 1) * imgWidth + centerX] = 255;
	_img.data[centerY * imgWidth + centerX + 1] = 255;
	_img.data[centerY * imgWidth + centerX - 1] = 255;
	
	for (int k = 0; k < MAX_K; k ++)
	{
		//��Բ�ϵ�����ļ���
		d_x = COS_B * m_cosTheta[k] - SIN_A * m_sinTheta[k] + centerX;
		d_y = SIN_B * m_cosTheta[k] + COS_A * m_sinTheta[k] + centerY;	

		x = cvRound(d_x);
		y = cvRound(d_y);

		if ((x >= 0) && (x < imgWidth) && (y >= 0) && (y < imgHeight))	
		{
				//ֻͳ���µ�����
				if (x != oldX || y != oldY)
				{
					_img.data[y * imgWidth + x] = 255;
					oldX = x;
					oldY = y;

				}		
		}

	}//for theta
	
	return;
}

/* ԭͼ������� */
void PupilDetection::SrcImgDetectPupil(void)
{
	/*double ratio = ((double)SRCIMGWIDTH / IMGWIDTH + (double)SRCIMGHEIGHT / IMGHEIGHT) / 2;
	
	m_threadPublicVal.startA = m_ellipse.a * ratio - SRCIMG_A_EXTENT;
	m_threadPublicVal.startB = max((double)m_ellipse.b * ratio - SRCIMG_B_EXTENT, m_threadPublicVal.startA * RATIO_THRESHOLD);
	m_threadPublicVal.startX = m_ellipse.x * ratio - SRCIMG_X_EXTENT;
	m_threadPublicVal.startY = m_ellipse.y * ratio - SRCIMG_Y_EXTENT;
	m_threadPublicVal.startAlphaK = (m_ellipse.alpha - SRCIMG_ALPHA_EXTENT) / ALPHA_INCREMENT;

	m_threadPublicVal.endA = m_ellipse.a * ratio + SRCIMG_A_EXTENT;
	m_threadPublicVal.endB = m_ellipse.b * ratio + SRCIMG_B_EXTENT;
	m_threadPublicVal.endX = m_ellipse.x * ratio + SRCIMG_X_EXTENT;
	m_threadPublicVal.endY = m_ellipse.y * ratio + SRCIMG_Y_EXTENT;
	m_threadPublicVal.endAlphaK = (m_ellipse.alpha + SRCIMG_ALPHA_EXTENT) / ALPHA_INCREMENT + 1;

	//a �����߽�
	if (m_threadPublicVal.startA < A_B_THRESHOLD)m_threadPublicVal.startA = A_B_THRESHOLD;
	if (m_threadPublicVal.endA >(min(m_threadPublicVal.maxX - m_threadPublicVal.minX, m_threadPublicVal.maxY - m_threadPublicVal.minY) / 2))
		m_threadPublicVal.endA = min(m_threadPublicVal.maxX - m_threadPublicVal.minX, m_threadPublicVal.maxY - m_threadPublicVal.minY) / 2;
	//x �����߽�
	if (m_threadPublicVal.startX < m_threadPublicVal.minX)m_threadPublicVal.startX = m_threadPublicVal.minX;
	if (m_threadPublicVal.endX > m_threadPublicVal.maxX)m_threadPublicVal.endX = m_threadPublicVal.maxX;
	//y �����߽�
	if (m_threadPublicVal.startY < m_threadPublicVal.minY)m_threadPublicVal.startY = m_threadPublicVal.minY;
	if (m_threadPublicVal.endY > m_threadPublicVal.maxY)m_threadPublicVal.endY = m_threadPublicVal.maxY;

	PDDrawRect(m_srcImg, Point(m_threadPublicVal.startX, m_threadPublicVal.startY), Point(m_threadPublicVal.endX, m_threadPublicVal.endY));
	imshow("m_srcImg", m_srcImg);
	waitKey(0);*/
	/* ������������m_srcImg.data  m_threadPublicVal  m_srcPeripheryPoints������GPU�� */

	/* ����GPU�ĺ������������̵߳Ľ�����������У�������CPU��m_srcThreadEllipse_DiffVal�� */
	/****************************************** �м�ʡ�� **********************************************/
	m_srcImg.copyTo(m_staticSrcImg);

	/*for (int i = 0; i < 2 * SRCIMG_A_EXTENT + 1; i ++)
	{
		m_srcThreadEllipse_DiffVal[i].maxVal = 0;
		m_srcThreadEndLabel[i] = false;
	}

	HANDLE handle[2 * SRCIMG_A_EXTENT + 1];    //��ͼ����
	for (int a = m_threadPublicVal.startA, i = 0; a <= m_threadPublicVal.endA; a ++, i ++)
	{
		
		handle[i] = CreateThread(NULL, 0, m_srcThreadFun, (LPVOID)a, 0, NULL);
		CloseHandle(handle[i]);
//		m_srcThreadFun(a);
	}
	for (int a = m_threadPublicVal.startA, i = 0; a <= m_threadPublicVal.endA; a ++, i ++)
	{
		//�ȴ��������߳̽���
		while (!m_srcThreadEndLabel[i]);

	}


	double maxVal = 0;
	for (int i = 0; i < 2 * SRCIMG_A_EXTENT + 1; i ++)
	{
		if (maxVal <= m_srcThreadEllipse_DiffVal[i].maxVal)
		{
			maxVal = m_srcThreadEllipse_DiffVal[i].maxVal;
			m_ellipse.a = m_srcThreadEllipse_DiffVal[i].a;
			m_ellipse.b = m_srcThreadEllipse_DiffVal[i].b;
			m_ellipse.x = m_srcThreadEllipse_DiffVal[i].x;
			m_ellipse.y = m_srcThreadEllipse_DiffVal[i].y;
			m_ellipse.alpha = m_srcThreadEllipse_DiffVal[i].alpha;
		}
	}
//	cout << "maxVal : " << maxVal << endl;

	/*m_ellipse.a = 25;
	m_ellipse.b = 24;
	m_ellipse.x = 356;
	m_ellipse.y = 170;
	m_ellipse.alpha = 0;*/

	//�滭��Բ
	//PDDrawEllipse(m_srcImg, SRCIMGWIDTH, SRCIMGHEIGHT);

}

/* ����������̺߳��� */
DWORD WINAPI PupilDetection::m_srcThreadFun(LPVOID lpParameter)
{
	
	return 1;
}

/* ������Բ�߽����ص��ݶ�ƽ��ֵ */
inline double PupilDetection::SrcImgEllipseDiffCalcu(const int _x, const int _y, const int count, PDPointVal *points)
{

	return 1;
}

/* �������a, b, alpha�µ���ԲԲ�ܸ��� */
void PupilDetection::SrcImgPeripheryPointCalcu(const int a, const int b, const int alphaK)
{

	
	return;
}

/* �Դ���ͼ����д��� */
int PupilDetection::ImgProcessing()
{
	if (NULL == m_img.data)
	{
#if _DEBUG
		cout << "ͼ��Ϊ�գ�" << endl;
#endif
		return PD_FAILED;
	}

	if (1 != m_img.channels())
	{
		//�Ҷ�ͼ
		cvtColor(m_img, m_img, CV_BGR2GRAY);
	}
	int i4 = 0, i5 = 0, total = m_img.cols * m_img.rows, count = 0, last_count = 0, inc_pixels = 0;
	float ra = 0, avr_inc_pixels = 0;
	int thresholds = 0, max_greyvalue = 0, did = 0;
	/**/
	//cout << static_para_signal << endl;
	if (!static_para_signal){
		for (i5 = 255; i5 > 150; i5 = i5 - 2){
			count = 0;

			for (i4 = 0; i4 < total; i4++){  // ���ڵ���ĳֵ���ռ��

				if (m_img.data[i4] >= i5){
					count++;
				}
			}
			if (count > 0 && did == 0)
			{
				max_greyvalue = i5;
				did = 1;
			}
			inc_pixels = count - last_count;
			if (i5 < 245){    //ǰ10��Ԥ�ȣ�ʹ��ֵ�ȶ�������տ�ʼʱƽ��ֵ���ȶ�����������������ĸ���ֵ
				if (inc_pixels > 10 * avr_inc_pixels)   //һ��increaseΪ֮ǰƽ����10�����ϣ���Ϊ���������־
					break;
			}
			if (i5 != 255){
				avr_inc_pixels = (float)count / (float)(255 - i5);  //ƽ����λ�Ҷ�ֵ�½���������ص���
			}
			last_count = count;
			//cout << i5 << " n:" << count <<" "<< avr_inc_pixels << endl;
		}

		//cout << "i5"<<i5 <<" "<<inc_count<< endl;

		/*������i5��ֵռ�ȣ����ں�����б����������Ա�������ֵ����������Ӧ��*/

		count = 0;
		for (i4 = 0; i4 < total; i4++){  // ����ĳֵ���ռ��

			if (m_img.data[i4] > i5){
				count++;
			}
		}

		ra = (float)count / (float)total;
		//cout << "ra:" << ra  << endl;
		SPOT_RATIO_THRESHOLD = ra;
		SPOT_GREYVALUE_DIFF = max_greyvalue - i5;
		//cout << SPOT_GREYVALUE_DIFF << endl;


		for (i5 = 0; i5 < 15; i5++){
			count = 0;
			thresholds = 100 + 10 * i5;
			for (i4 = 0; i4 < total; i4++){
				if (m_img.data[i4]>thresholds)
					count++;
			}
			if ((float)count / (float)total < SPOT_RATIO_THRESHOLD){   //ȡǰ�ٷ�֮һ������Ϊ��ֵ->ra
				G_L_THRESHOLD = thresholds;
				break;
			}

		}
	}
	//cout << "G_L_THRESHOLD  " << G_L_THRESHOLD << endl;
	/*
	for (i5 = 255; i5 > 150; i5--){
	count = 0;
	for (i4 = 0; i4 < total; i4++){  // ����ĳֵ���ռ��

	if (m_img.data[i4] > i5){
	count++;
	}
	}

	ra = (float)count / (float)total;
	//cout << "ra:" << ra  << endl;
	cout << i5 << " n:" << count << endl;
	}*/

	//��ֵ��
	Mat thresholdImg;
	threshold(m_img, thresholdImg, G_L_THRESHOLD, 255, THRESH_BINARY);   /*GLΪ�й����ֵ*/

#if ADJUST_THRESHOLD
	//	if (showImg)
	{
		imshow("thresholdImg", thresholdImg);
		waitKey(0);
	}
#endif

	//������
	vector<vector<Point>> contours, contours1;

	Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
	dilate(thresholdImg, thresholdImg, element);   //����ʹ���׼ȷ
	findContours(thresholdImg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	Scalar color(180, 120, 50);
	drawContours(thresholdImg, contours, -1, color);
	/*
	imshow("saveImg", thresholdImg);
	waitKey(0);*/

	const int offset1 = 1;
	const int offset2 = 3;
	int width = m_img.cols;
	int height = m_img.rows;
	int topElement, bottomElement, leftElement, rightElement;
	int avgElement;
	int elements[4];
	int highEle[4] = { -1 }, lowEle[4] = { -1 };
	int maxOffset1 = 0, maxOffset2 = 0;
	int tempOffset1 = 0, tempOffset2 = 0;
	int x, y, r, R;
	int A, B, C;
	int rOffset;
	CvPoint linePoint1, linePoint2;
	CvPoint leftTopPoint, rightBottomPoint;
	Point2f circleCenter;
	float cirCleRadius;
	int contoursSize = contours.size();

	//cout << contoursSize << endl;

	int i = 0, h = 0, l = 0, t = 0, j = 0, k = 0;

	double tempCosTheta, tempSinTheta;
	const int MAX_K = TWOPI / THETA_INCREMENT;
	int fillingX, fillingY;


	/*for (i = 0; i < contoursSize; i++)
	{

	if (contours[i].size() < CONTOURS_MIN_SIZE || contours[i].size() > CONTOURS_MAX_SIZE)
	{
	continue;
	}
	j+=1;
	}
	j = 0;
	//cout << "  "<<j << endl;  //ʣ��������
	*/
	float circle_diff = 0, min_circle_diff = 100000;
	int spo_x = 0, spo_y = 0, spo_r = 0;  //��׼���
	int max_avr_greyvalue_diff = 0, sta_y = 0, sta_x = 0;
	float sum_diff = -1000;


	/*******һ�ֱ����ұ�׼���*********/
	for (i = 0; i < contoursSize; i++)
	{



		if (contours[i].size() < CONTOURS_MIN_SIZE || contours[i].size() > CONTOURS_MAX_SIZE)
		{
			contours[i][0].x = -1;  //ȡһλ���ڶ�continue����ѭ���ı��
			continue;    /*��������forѭ��*/
		}
		// cout << contours[i].size() << endl;
		minEnclosingCircle(contours[i], circleCenter, cirCleRadius);   /*��С���Բ��opencv�Դ�*/
		x = circleCenter.x;
		y = circleCenter.y;
		r = cirCleRadius;


		/*���߽�*/
		r = r + GET_POINT_OFFSET;
		if ((x + 2 * r > width - 50 || x - 2 * r < 50) || (y + 2 * r > height - 50 || y - 2 * r < 50))
		{
			contours[i][0].x = -3;
			continue;
		}
		/*if (1){
		n_contours++;
		}*/
		r = r - GET_POINT_OFFSET;
		/*ȥ��������*/
		int sumx = 0, sumy = 0, r_in = r - 1, r_out = r + 2, avr_in = 0, avr_out = 0;
		for (j = -r_in; j <= r_in; j++){
			sumy += m_img.data[(y + j)*width + x];
		}
		for (j = -r_in; j <= r_in; j++){
			sumx += m_img.data[y *width + x + j];

		}
		j = 0;
		avr_in = (sumx + sumy) / (4 * r_in + 2);
		avr_out = m_img.data[(y + r_out)*width + x] + m_img.data[(y - r_out)*width + x] + m_img.data[(y)*width + x + r_out] + m_img.data[(y)*width + x - r_out];
		avr_out = avr_out / 4;


		if (avr_in - avr_out > max_avr_greyvalue_diff){
			sta_y = y;
			sta_x = x;
			max_avr_greyvalue_diff = avr_in - avr_out;
		}
		/****************/
		//contours[i][1].x = avr_in-avr_out;//�ø�λ�洢�ò�ֵ,���ɣ���Ϊ�����ô�λ
		/***************/

		if (avr_in - avr_out < SPOT_GREYVALUE_DIFF / 3){   /*�������Χƽ����ֵ��20����*/ /***********���60��ô��******/
			contours[i][0].x = -2;
			//��¼�˲�ֵ�����ڶ���һ�α�������ж�
			continue;
		}
		if (avr_in - avr_out < SPOT_GREYVALUE_DIFF){    //��׼��ߵĲ�ֵҪ��60
			continue;
		}
		/*��diff�����ұ�׼��߲���Ϊ֮�����ο�*/
		for (int j = 0; j < contours[i].size(); j++)

		{
			circle_diff += abs((contours[i][j].y - y)*(contours[i][j].y - y) + (contours[i][j].x - x)*(contours[i][j].x - x) - (r*r));
		}
		j = 0;
		circle_diff = circle_diff / (float)(contours[i].size()*r*r*r);    //r^3�������ҽϴ��ߣ������������size�����ų�
		//cout << diff;
		if (circle_diff < min_circle_diff){
			min_circle_diff = circle_diff;
		}

		if (avr_in - avr_out - 100 * circle_diff>sum_diff){  //Խ��Խ��
			sum_diff = avr_in - avr_out - 100 * circle_diff;  //max_avrԽ��mindiffԽСԽ�ã���*100ʹ����ͬ����
			spo_x = x;
			spo_y = y;
			spo_r = r;
			spo_r = min(10, max(spo_r, 5));    //5Ϊ��׼��߰뾶����,10Ϊ����
		}





		/***�ûҶ�ֵʮ�ֶ�λ�������***/
		/*sumx = 0, sumy = 0;
		int xx = x, yy = y;
		int len = 10, i1 = 0, i2 = 0, i3 = 0, maxunit = 0, X = 0, Y = 0;
		for (i1 = -len / 2; i1 <= len / 2; i1++){
		for (i2 = -len / 2; i2 <= len / 2; i2++){
		sumx = 0; sumy = 0;
		X = x + i1;
		Y = y + i2;
		//maxunit = m_img.data[(Y)*width + X];
		for (i3 = -r; i3 <= r; i3++){
		sumy += m_img.data[(Y + i3)*width + X];
		}
		for (i3 = -r; i3 <= r; i3++){
		sumx += m_img.data[Y *width + X + i3];

		}
		sumx = sumx + sumy;

		if (sumx >maxunit){
		maxunit = sumx;
		xx = X;
		yy = Y;
		}
		}
		}
		*/

		//m_img.data[yy*width + xx] = 0;
		/*m_img.data[(yy + 1)*width + xx] = 0;
		m_img.data[(yy - 1)*width + xx] = 0;
		m_img.data[yy *width + xx + 1] = 0;
		m_img.data[yy *width + xx - 1] = 0;*/
		continue;






	}
	//cout << "min_diff: " << mindiff << endl;
	//cout << "max_avr: " << max_avr << endl;
	/**///ֻ�лҶ�ƽ���ݶȺ�������ù��
	m_img.data[(sta_y + 1)*width + sta_x] = 0;
	m_img.data[(sta_y - 1)*width + sta_x] = 0;
	m_img.data[sta_y *width + sta_x + 1] = 0;
	m_img.data[sta_y *width + sta_x - 1] = 0;
	j = 0;
	int sig = 0, n_con = 0;  //sig��ǳߴ����n_con��¼���кϸ�contour����
	int configure_contours[50][3];  //�ϸ�contour�����¼����
	//cout << "r:" << spo_r << endl;





	/*******���ֵ�һ�α�����0.1��ֵ�ҹ��*********/
	for (i = 0; i < contoursSize; i++)
	{

		if (contours[i][0].x < 0)
		{
			continue;
		}

		minEnclosingCircle(contours[i], circleCenter, cirCleRadius);   /*��С���Բ��opencv�Դ�*/
		x = circleCenter.x;
		y = circleCenter.y;
		r = cirCleRadius;

		contours[i][0].y = (x - spo_x)*(x - spo_x) + (y - spo_y)*(y - spo_y);  //ȡһλ��¼���׼��߾���

		if (contours[i][0].y > 2200 * spo_r){   //���׼splot������Ϊ12000������������Ϊ��Χ�ų�  /**********��12000��ô�����۹�ƫʱ�ᳬ����ֵ��̫С�ֻ�Ѷ�������*****/
			continue;
		}
		if (y < spo_y - 6 * spo_r){   //ע��ͼ��y����������
			continue;
		}
		if (r > 2 * spo_r){
			sig = 1;
			break;
		}
		//cout <<"1:  "<< contours[i][0].y<<"  ";
		m_img.data[y*width + x] = 0;
		/*��ȫ��*/m_img.data[(y + 1)*width + x] = 0;
		m_img.data[(y - 1)*width + x] = 0;
		m_img.data[y *width + x + 1] = 0;
		m_img.data[y *width + x - 1] = 0;
		configure_contours[n_con][0] = x;
		configure_contours[n_con][1] = y;
		configure_contours[n_con][2] = r;
		n_con++;
	}




	float thre = 0;
	int first_n_con = 0;


	for (k = 1; k < 5; k++){
		thre = 0.1*k + SPOT_RATIO_THRESHOLD;
		first_n_con = n_con;
		/*******���ֵڶ��α�����0.4��ֵ�ҹ�ߣ������һ�Σ�*********/
		if (n_con < 4){
			for (i5 = 0; i5 < 15; i5++){
				count = 0;
				thresholds = 100 + 10 * i5;
				for (i4 = 0; i4 < total; i4++){
					if (m_img.data[i4]>thresholds)
						count++;
				}
				if ((float)count / (float)total < thre){   //ȡǰ�ٷ�֮�ĸ�����Ϊ��ֵ
					G_L_THRESHOLD = thresholds;
					break;
				}

			}

			threshold(m_img, thresholdImg, G_L_THRESHOLD, 255, THRESH_BINARY);
			dilate(thresholdImg, thresholdImg, element);   //����ʹ���׼ȷ
			findContours(thresholdImg, contours1, RETR_EXTERNAL, CHAIN_APPROX_NONE);
			contoursSize = contours1.size();

			for (i = 0; i < contoursSize; i++)
			{

				if (contours1[i].size() < CONTOURS_MIN_SIZE || contours1[i].size() > CONTOURS_MAX_SIZE)
				{
					contours1[i][0].x = -1;  //ȡһλ���ڶ�continue����ѭ���ı��
					continue;
				}
				// cout << contours[i].size() << endl;
				minEnclosingCircle(contours1[i], circleCenter, cirCleRadius);
				x = circleCenter.x;
				y = circleCenter.y;
				r = cirCleRadius;


				int  sumx = 0, sumy = 0, r_in = r - 1, r_out = r + 3, avr_in = 0, avr_out = 0;
				for (j = -r_in; j <= r_in; j++){
					sumy += m_img.data[(y + j)*width + x];
				}
				for (j = -r_in; j <= r_in; j++){
					sumx += m_img.data[y *width + x + j];
				}
				j = 0;
				avr_in = (sumx + sumy) / (4 * r_in + 2);
				avr_out = m_img.data[(y + r_out)*width + x] + m_img.data[(y - r_out)*width + x] + m_img.data[(y)*width + x + r_out] + m_img.data[(y)*width + x - r_out];
				avr_out = avr_out / 4;

				if (avr_in - avr_out < SPOT_GREYVALUE_DIFF / 3){
					contours1[i][0].x = -2;
					continue;
				}
				if (contours1[i][0].x < 0)
				{
					continue;
				}
				minEnclosingCircle(contours1[i], circleCenter, cirCleRadius);
				x = circleCenter.x;
				y = circleCenter.y;
				r = cirCleRadius;
				/*if (r > 2 * spo_r){
					sig = 1;              //���������ж�
					continue;
					}*/
				contours1[i][0].y = (x - spo_x)*(x - spo_x) + (y - spo_y)*(y - spo_y);  //ȡһλ��¼���׼��߾���

				//cout << "r:" << spo_r << endl;

				if (contours1[i][0].y > 2200 * spo_r){   //���׼splot������Ϊ12000������������Ϊ��Χ�ų�  
					continue;
				}
				if (y < spo_y - 6 * spo_r){   //ע��ͼ��y����������
					continue;
				}
				//cout << "2:  " << contours1[i][0].y << "  ";

				int sig1 = 0;  //sig1Ϊ1����ԭ����ظ���Ϊͬһ��
				/*�¹����ԭ��߸���+-15pixels���϶�Ϊ��ԭ��Ϊͬһ�����*/
				for (j = 0; j < first_n_con; j++){
					if (x > configure_contours[j][0] - 3 * spo_r && x<configure_contours[j][0] + 3 * spo_r && y>configure_contours[j][1] - 3 * spo_r && y < configure_contours[j][1] + 3 * spo_r)
					{
						sig1 = 1;
						break;
					}
					else;
				}
				if (sig1 == 0){

					configure_contours[n_con][0] = x;
					configure_contours[n_con][1] = y;
					configure_contours[n_con][2] = r;
					m_img.data[y*width + x] = 0;
					/*��ȫ��*/m_img.data[(y + 1)*width + x] = 0;
					m_img.data[(y - 1)*width + x] = 0;
					m_img.data[y *width + x + 1] = 0;
					m_img.data[y *width + x - 1] = 0;
					n_con++;

				}



			}
		}

	}



	m_img.data[spo_y*width + spo_x] = 0;  //��Ǳ�׼���
	/**/m_img.data[(spo_y + 1)*width + spo_x] = 0;
	m_img.data[(spo_y - 1)*width + spo_x] = 0;
	m_img.data[spo_y *width + spo_x + 1] = 0;
	m_img.data[spo_y *width + spo_x - 1] = 0;
	/*	imshow("saveImg", m_img);
	waitKey(0);*/








	/*********������************/
	stringstream sss;

	int sig2 = 0, dist = 0, max_dist = 0, N_con = 0, sig_i = -1,  qualify = 0;
	int configure_ordered_contours[4][2];
	k = 0;
	int avr_x = 0, avr_y = 0;
	sss << "5 ";
	if (n_con > 10 || n_con <= 1)
	{
		//cout << "ERROR" << endl;
		sss << "0";
	}
	else{
		if (n_con < 4){
			sss << "0 ";
			//imshow("saveImg", m_img);
			//waitKey(0);
			//sss << "num_error��less!   " << "num:" << n_con << endl;
			//cout << "num_error��less!   "<<"num:"<<n_con<<endl;
		}

		else if (n_con > 4){    /********�Դ���4����ߣ�ѡȡ�����໥���������4��*****/
			N_con = n_con;
			while (n_con > 4)
			{
				//sss << "error4! ";
				//imshow("saveImg", m_img);
				//waitKey(0);
				for (i = 0; i < N_con; i++){
					if (configure_contours[i][0] == -1) //���ڱ�����ж����Ĺ��
						continue;
					for (j = 0; j < N_con; j++){
						if (configure_contours[j][0] == -1)
							continue;
						dist += (configure_contours[i][0] - configure_contours[j][0]) * (configure_contours[i][0] - configure_contours[j][0]) + (configure_contours[i][1] - configure_contours[j][1])*(configure_contours[i][1] - configure_contours[j][1]);
					}
					if (dist > max_dist)
					{
						sig_i = i;
						max_dist = dist;

					}

				}

				n_con--;
				m_img.data[configure_contours[sig_i][1] * width + configure_contours[sig_i][0]] = 255;   //���ù�߱��ȥ��
				configure_contours[sig_i][0] = -1;
				sig_i = -1;
				max_dist = 0;
				dist = 0;

			}


			//sss.str("   size_error!");
			//sss << x<<"  "<<y<<"size_error";
			//sss << "error! " << endl;
			//sss << "num_error: more!   " << "num:" << n_con << endl;
			//cout << "num_error: more!   " << "num:" << n_con << endl;
		}
		if (n_con == 4 && sig == 0){
			/***У��4�㱣֤������**/
			for (i = 0; i < N_con; i++){
				if (configure_contours[i][0] == -1) //�����������ų��Ĺ��
					continue;
				for (j = 0; j < N_con; j++){
					if (configure_contours[j][0] == -1 || i == j)
						continue;
					dist = (configure_contours[i][0] - configure_contours[j][0]) * (configure_contours[i][0] - configure_contours[j][0]) + (configure_contours[i][1] - configure_contours[j][1])*(configure_contours[i][1] - configure_contours[j][1]);
					if (dist > spo_r * 3000 || dist < 300 * spo_r)   //�ĵ��෶Χ
					{
						//cout << dist << endl;
						sss << "0 ";
						//imshow("saveImg", m_img);
						//waitKey(0);
						sig2 = 1;
						i = 4;   //ֱ������2��ѭ��
						break;
					}
				}
			}

			if (sig2 == 0)
			{
				avr_x = 0;
				avr_y = 0;
				for (j = 0; j < 4; j++){
					avr_x += configure_contours[j][0];
					avr_y += configure_contours[j][1];
				}
				avr_x = avr_x / 4;
				avr_y = avr_y / 4;
				/*m_img.data[avr_y*width + avr_x] = 255;
				imshow("saveImg", m_img);
				waitKey(0);*/
				//cout << configure_contours[0][0] << avr_x << configure_contours[0][1] << avr_y;
				sss << "1 ";
				for (j = 0; j < 4; j++)
				{
					if (configure_contours[j][0] < avr_x&&configure_contours[j][1] < avr_y)
					{

						configure_ordered_contours[0][0] = configure_contours[j][0];
						configure_ordered_contours[0][1] = configure_contours[j][1];

					}

					else if (configure_contours[j][0] < avr_x&&configure_contours[j][1] > avr_y)
					{
						configure_ordered_contours[1][0] = configure_contours[j][0];
						configure_ordered_contours[1][1] = configure_contours[j][1];
					}
					else if (configure_contours[j][0] > avr_x&&configure_contours[j][1] > avr_y){
						configure_ordered_contours[2][0] = configure_contours[j][0];
						configure_ordered_contours[2][1] = configure_contours[j][1];
					}
					else if (configure_contours[j][0] > avr_x&&configure_contours[j][1] < avr_y)
					{
						configure_ordered_contours[3][0] = configure_contours[j][0];
						configure_ordered_contours[3][1] = configure_contours[j][1];

					}
					else {
						qualify = 0;
						break;
					}
					//cout << qualify;
				}
				for (j = 0; j < 4; j++)
				{
					if (last_facula[0][0])
					{
						
						//cout << last_facula[0][0] << configure_ordered_contours[0][0];
						if (abs(last_facula[j][0] - configure_ordered_contours[j][0]) < 20 && abs(last_facula[j][1] - configure_ordered_contours[j][1]) < 20)
						{
							qualify = 1;

						}
						else {
							qualify = 0;
							break;
						}
					}
					else qualify = 1;
				}
			


				if (qualify)
				{
					for (j = 0; j < 4; j++){
						   sss << configure_ordered_contours[j][0] << " " << configure_ordered_contours[j][1] << " ";
						/* //sss << "(" << configure_contours[j][0] << "," << configure_contours[j][1] << ")" << "  ";

						if (configure_contours[j][0] < avr_x&&configure_contours[j][1] < avr_y)
						{
						sss << configure_contours[j][0] << " " << configure_contours[j][1] << " ";
						//last_facula[1][0] = configure_ordered_contours[j][0];
						}
						}
						for (j = 0; j < 4; j++){
						if (configure_contours[j][0] < avr_x&&configure_contours[j][1] > avr_y)
						{
						sss <<  configure_contours[j][0] << " " << configure_contours[j][1] << " ";
						}
						}
						for (j = 0; j < 4; j++){
						if (configure_contours[j][0] > avr_x&&configure_contours[j][1] > avr_y)
						{
						sss << configure_contours[j][0] << " " << configure_contours[j][1] << " ";
						}
						}
						for (j = 0; j < 4; j++){
						if (configure_contours[j][0] > avr_x&&configure_contours[j][1] < avr_y)
						{
						sss <<  configure_contours[j][0] << " " << configure_contours[j][1] << " ";
						}
						}*/
						last_facula[j][0] = configure_ordered_contours[j][0];
						last_facula[j][1] = configure_ordered_contours[j][1];
						//sss << endl;
						//cout << "success!   " << endl;
						
					}
				}
				
				else{
					sss << "0 ";
				}
			}
			else if (sig == 1){           //sig=1����
				sss << "0 ";
				//imshow("saveImg", m_img);
				//waitKey(0);
				//sss<<"size_error! ";
				//cout << "size_error!   " ;
			}
		}
	}
	

		m_str = sss.str();
		//cout << mindiff << "   " << spo_r << "   " <<endl;












		/*

		//���Ҷ�ֵ�ֲ�����ֵ������Ѱ�����Բ
		m_img.copyTo(m_coarsePosImg);
		const uchar SEPARATION_OF_TONES_STEP = G_L_THRESHOLD / (SEPARATION_OF_TONES_NUM - 1);
		uchar separationOfTones[SEPARATION_OF_TONES_NUM + 1];
		for (int i = 0; i < SEPARATION_OF_TONES_NUM; i ++)
		{
		separationOfTones[i] = i * SEPARATION_OF_TONES_STEP;
		}

		m_threadPublicVal.minX = 0;
		m_threadPublicVal.maxX = SRCIMGWIDTH;
		m_threadPublicVal.minY = SRCIMG_FRAME_THRESHOLD;
		m_threadPublicVal.maxY = SRCIMGHEIGHT - SRCIMG_FRAME_THRESHOLD;

		//��Сͫ������
		//���ǵ�һ�Ŵ����ͼ
		int beginI = 1;
		int endI = SEPARATION_OF_TONES_NUM;
		//����ǰһ֡ͼ�������Ӧ����ֵ����������Χ
		if (abs(m_coarsePosThreshold) > 0.0001)
		{
		int thresholdIndex = m_coarsePosThreshold / SEPARATION_OF_TONES_STEP;
		beginI = max(1, thresholdIndex - 3);
		endI = min(SEPARATION_OF_TONES_NUM, thresholdIndex + 3);
		}
		for (int i = beginI; i < endI; i ++)
		{
		m_coarsePosThLabel[i] = false;
		}

		m_coarsePosValAccum = 1;
		HANDLE handle[SEPARATION_OF_TONES_NUM + 1];
		for (int i = beginI; i < endI; i ++)
		{
		handle[i] = CreateThread(NULL, 0, PupilCoarsePositioning, (LPVOID)i, 0, NULL);
		CloseHandle(handle[i]);
		}
		//�ȴ��߳̽���
		for (int i = beginI; i < endI; i ++)
		{
		while (!m_coarsePosThLabel[i]);
		}

		//����Сֵ
		double minVal = 1, radius = 0;
		for (int i = beginI; i < endI; i ++)
		{
		if (minVal > m_coarsePosThVals[i].coarsePosValAccum)
		{
		minVal = m_coarsePosThVals[i].coarsePosValAccum;
		m_coarsePosThreshold = m_coarsePosThVals[i].coarsePosThreshold;
		m_threadPublicVal.minX = m_coarsePosThVals[i].minX;
		m_threadPublicVal.maxX = m_coarsePosThVals[i].maxX;  //����ȷ������
		m_threadPublicVal.minY = m_coarsePosThVals[i].minY;
		m_threadPublicVal.maxY = m_coarsePosThVals[i].maxY;
		radius = m_coarsePosThVals[i].radius;
		}
		}

		//��minVal����ĳ����ֵ������Ϊ��ͫ�������������ԣ�ͼ�����п���û��ͫ��
		if (minVal > SHRINK_AREA_MINVAL_TH)
		{
		return SHRINK_AREA_FAILED;
		}

		//����offsetֵ
		m_offset = (radius / 30) + 1;


		//#if SHOW_IMG
		if (showImg)
		{
		//	PDDrawRect(m_coarsePosImg, Point(m_threadPublicVal.minX, m_threadPublicVal.minY), Point(m_threadPublicVal.maxX, m_threadPublicVal.maxY));
		threshold(m_coarsePosImg, thresholdImg, m_coarsePosThreshold, 255, CV_THRESH_BINARY_INV);
		imshow("m_coarsePosImg", thresholdImg);
		waitKey(0);
		}
		//#endif*/
		//����ԭͼ
		m_img.copyTo(m_srcImg);
		//��С�ֱ���
		resize(m_img, m_img, Size(IMGWIDTH, IMGHEIGHT));

		return PD_SUCCEED;
	}


/* �����������ȷ����ֱ�� A * x + B * y + C = 0 */
void PupilDetection::LinearEquationCalcu(CvPoint &point1, CvPoint &point2, int &A, int &B, int &C)
{
	
}

/* �Թ������������,�� A*x + B*y + C < 0�������filledVal1 */
void PupilDetection::SpotAreaFill(Mat &img, CvPoint leftTopPoint, CvPoint rightBottomPoint, int filledVal1, int filledVal2,
	int A, int B, int C)
{
	

}

/* ͫ�״ֶ�λ����������̺߳��� */
DWORD WINAPI PupilDetection::PupilCoarsePositioning(LPVOID threadIndex)
{
	
	return 1;
}

/* ����ͫ�״ֶ�λʱ������ɸѡ */
bool PupilDetection::ContourScreening(vector<Point> &contours, double &valueAccum, Point &circleCenter, int &circleRadius)
{



	return true;
}

void PDDrawRect(Mat &img, Point pt1, Point pt2)
{
	
	return;
}

void PupilDetection::PDDrawCircle(Mat &img, int r, int _x, int _y)
{
	
	return;
}