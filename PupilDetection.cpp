
#include "PupilDetection.h"

Mat PupilDetection::m_staticImg;
int ** PupilDetection::m_peripheryPoints = NULL;
bool *PupilDetection::m_threadEndLabel = NULL;
PDEllipse_DiffVal *PupilDetection::m_threadEllipse_DiffVal = NULL;
int PupilDetection::m_grads[IMGHEIGHT * IMGWIDTH];

int PupilDetection::m_coarsePosThreshold = 0;
Mat PupilDetection::m_coarsePosImg;
double PupilDetection::m_coarsePosValAccum;
bool PupilDetection::m_coarsePosThLabel[SEPARATION_OF_TONES_NUM + 1];
PDCoarPosThVal PupilDetection::m_coarsePosThVals[SEPARATION_OF_TONES_NUM + 1];

Mat PupilDetection::m_staticSrcImg;
PDPointVal ** PupilDetection::m_srcPeripheryPoints = NULL;
PDThreadPublicVal PupilDetection::m_threadPublicVal;
PDEllipse_DiffVal PupilDetection::m_srcThreadEllipse_DiffVal[2 * SRCIMG_A_EXTENT + 1];
bool PupilDetection::m_srcThreadEndLabel[2 * SRCIMG_A_EXTENT + 1];

int PupilDetection::m_offset;
//int PupilDetection::OUT_OFFSET;
//int PupilDetection::IN_OFFSET;

extern bool showImg;

void PDDrawRect(Mat &img, Point pt1, Point pt2);

PupilDetection::PupilDetection()
{

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
	}

}

PupilDetection::~PupilDetection()
{

	if (NULL != m_cosTheta)
	{
		delete m_cosTheta;
		m_cosTheta = NULL;
	}
	if (NULL != m_sinTheta)
	{
		delete m_sinTheta;
		m_sinTheta = NULL;
	}
	if (NULL != m_cosAlpha)
	{
		delete m_cosAlpha;
		m_cosAlpha = NULL;
	}
	if (NULL != m_sinAlpha)
	{
		delete m_sinAlpha;
		m_sinAlpha = NULL;
	}
	
	//�������Բ�ܵ�����
	//Сͼ
	const int LEN = (IMGHEIGHT / 2 + 1) * (IMGHEIGHT / 2 + 1) * (int)(PD_PI / ALPHA_INCREMENT + 1);
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
	
}

/* ����ͼ�� */
int PupilDetection::setImg(Mat &_img)
{
	_img.copyTo(m_img);
	return PD_SUCCEED;

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

	int imgElement = 0;
	int imgLeftElement = 0;
	int imgRightElement = 0;
	int imgTopElement = 0;
	int imgBottomElement = 0;
	int leftVal, topVal = 0;
	const double srcimg_img_ratio = (double)IMGWIDTH / SRCIMGWIDTH;
	const int MIN_X = max(int(m_threadPublicVal.minX * srcimg_img_ratio + m_offset), FRAME_WIDTH);
	const int MIN_Y = (m_threadPublicVal.minY * srcimg_img_ratio + m_offset);
	const int MAX_X = min(int(m_threadPublicVal.maxX * srcimg_img_ratio - m_offset), IMGWIDTH - FRAME_WIDTH);
	const int MAX_Y = (m_threadPublicVal.maxY * srcimg_img_ratio - m_offset);

	for (int x = MIN_X; x < MAX_X; x ++)
	{
		for (int y = MIN_Y; y < MAX_Y; y ++)
		{
			
			imgElement = m_img.data[y * IMGWIDTH + x];
			imgLeftElement = m_img.data[y * IMGWIDTH + x - m_offset];       /*ֻ������*/
			imgTopElement = m_img.data[(y - m_offset) * IMGWIDTH + x];

			leftVal = abs(imgLeftElement - imgElement);
			topVal = abs(imgTopElement - imgElement);
			
			m_grads[y * IMGWIDTH + x] = leftVal + topVal;     

		}
	}
}

/* ����ͫ�׼�ⶨλ����⵽����Բ����ellipse�� */
int PupilDetection::DetectPupil(void)
{

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
		return SHRINK_AREA_FAILED;
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
	GradCalcu();

	m_img.copyTo(m_staticImg);

	const double srcimg_img_ratio = (double)IMGWIDTH / SRCIMGWIDTH;
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
	//ԭͼ�������
	SrcImgDetectPupil();    //�Ŵ��ԭͼ

	return PD_SUCCEED;
}

/* ԭͼ����������̺߳��� */
DWORD WINAPI PupilDetection::m_threadFun(LPVOID lpParameter)
{
	int a = (int)lpParameter;
	int aIndex = a - (OUT_OFFSET + 1);

	double maxVal = 0;
	double tempVal = 0;
	int minX, minY, maxX, maxY = 0;
	int tempPointCount, maxPointCount = 0;
	const double srcimg_img_ratio = (double)IMGWIDTH / SRCIMGWIDTH;

	const int MIN_X = max(int(m_threadPublicVal.minX * srcimg_img_ratio), FRAME_WIDTH);
	const int MIN_Y = (m_threadPublicVal.minY * srcimg_img_ratio);
	const int MAX_X = min(int(m_threadPublicVal.maxX * srcimg_img_ratio), IMGWIDTH - FRAME_WIDTH);
	const int MAX_Y = (m_threadPublicVal.maxY * srcimg_img_ratio);
	const int MAX_ALPHA_K = PD_PI / ALPHA_INCREMENT + 1;
	int *tempPoints = NULL;
	const int A_NUM = (int)(IMGHEIGHT / 2 + 1) * (int)(PD_PI / ALPHA_INCREMENT + 1);
	const int B_NUM = (int)(PD_PI / ALPHA_INCREMENT + 1);

	int tempX = 0, tempY = 0, tempB = 0, tempAlphaK = 0;
	int x = 0, y = 0, b = 0, alphaK = 0;

	for (int tempB = max(A_B_THRESHOLD, int(a * RATIO_THRESHOLD)); tempB <= a; tempB ++)
	{

		for (int tempAlphaK = 0; tempAlphaK < MAX_ALPHA_K; tempAlphaK ++)
		{
			//�������a, b, alpha�µ���ԲԲ�ܸ���
			tempPoints = m_peripheryPoints[a * A_NUM + tempB * B_NUM + tempAlphaK];
			if (!tempPoints)
			{
				continue;
			}
			tempPointCount = tempPoints[0];
			if (tempPointCount < 0)
			{
				continue;
			}

			minX = tempPoints[1];
			maxX = tempPoints[2];
			minY = tempPoints[3];
			maxY = tempPoints[4];

			minX = MIN_X - minX + m_offset;
			maxX = MAX_X - maxX - m_offset;
			minY = MIN_Y - minY + m_offset;
			maxY = MAX_Y - maxY - m_offset;
			for (int tempX = minX; tempX < maxX; tempX ++)
			{
				for (int tempY = minY; tempY < maxY; tempY ++)
				{

					tempVal = EllipDiffCalcu(tempX, tempY, tempPointCount, tempPoints + 5);

					if (maxVal < tempVal)
					{
						maxVal = tempVal;
						x = tempX;
						y = tempY;
						b = tempB;
						alphaK = ALPHA_INCREMENT * tempAlphaK;
					}//if

				}//y

			}//x
			if (a == b)		//a == b��������һ��Բ��alphaΪ0�͹���
			{
				break;
			}//if
		}//alphaK
	}//b

	m_threadEllipse_DiffVal[aIndex].maxVal = maxVal;
	m_threadEllipse_DiffVal[aIndex].a = a;
	m_threadEllipse_DiffVal[aIndex].b = b;
	m_threadEllipse_DiffVal[aIndex].x = x;
	m_threadEllipse_DiffVal[aIndex].y = y;
	m_threadEllipse_DiffVal[aIndex].alpha = alphaK * ALPHA_INCREMENT;

	m_threadEndLabel[aIndex] = true;

	return 1;

}

/* ������Բ�߽����ص��ݶ�ƽ��ֵ */
double PupilDetection::EllipDiffCalcu(const int _x, const int _y, const int count, int *points)
{
	int valueAccum = 0;

	//������ĻҶ�ֵ����һ��ֵ�����ų��������
	if ((m_staticImg.data[_y * IMGWIDTH + _x]) > (m_coarsePosThreshold + CENTER_THRESHOLD_EXTENT))
	{
		return -1;
	}

	int x = 0, y = 0;
	const int incrementVal = _y * IMGWIDTH + _x;
	int tempPointVal0, tempPointVal1, tempPointVal2;
	short tempGrad0, tempGrad1, tempGrad2;
	int limit = (count / 3) * 3;
	int k = 0;

	for (; k < limit; k += 3)
	{
		//��Բ�ϵ�����ļ���
		tempPointVal0 = points[k] + incrementVal;
		tempPointVal1 = points[k + 1] + incrementVal;
		tempPointVal2 = points[k + 2] + incrementVal;

		//�ݶ�����
		tempGrad0 = m_grads[tempPointVal0];
		tempGrad1 = m_grads[tempPointVal1];
		tempGrad2 = m_grads[tempPointVal2];

		valueAccum += tempGrad0;
		valueAccum += tempGrad1;
		valueAccum += tempGrad2;

	}//for theta
	for (; k < count; k ++)
	{
		//��Բ�ϵ�����ļ���
		tempPointVal0 = points[k] + incrementVal;

		//�ݶ�����
		valueAccum += m_grads[tempPointVal0];

	}//for theta

	return (double)valueAccum / count;
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
	double ratio = ((double)SRCIMGWIDTH / IMGWIDTH + (double)SRCIMGHEIGHT / IMGHEIGHT) / 2;
	
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

	/*PDDrawRect(m_srcImg, Point(m_threadPublicVal.startX, m_threadPublicVal.startY), Point(m_threadPublicVal.endX, m_threadPublicVal.endY));
	imshow("m_srcImg", m_srcImg);
	waitKey(0);*/
	/* ������������m_srcImg.data  m_threadPublicVal  m_srcPeripheryPoints������GPU�� */

	/* ����GPU�ĺ������������̵߳Ľ�����������У�������CPU��m_srcThreadEllipse_DiffVal�� */
	/****************************************** �м�ʡ�� **********************************************/
	m_srcImg.copyTo(m_staticSrcImg);

	for (int i = 0; i < 2 * SRCIMG_A_EXTENT + 1; i ++)
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
	/************************************ ������� ****************************************************/

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
	PDDrawEllipse(m_srcImg, SRCIMGWIDTH, SRCIMGHEIGHT);

}

/* ����������̺߳��� */
DWORD WINAPI PupilDetection::m_srcThreadFun(LPVOID lpParameter)
{
	/* ��ʶ�߳� */
	int a = (int)lpParameter;
	int aIndex = a - m_threadPublicVal.startA;

	int alphaKLen = PD_PI / ALPHA_INCREMENT + 1;
	int minX, minY, maxX, maxY = 0;
	double maxVal = 0;
	double tempVal = 0;
	int tempPointCount, maxPointCount = 0;
	PDPointVal *tempPoints = NULL;
	const int A_NUM = (SRCIMGHEIGHT / 2 + 1) * (int)(PD_PI / ALPHA_INCREMENT + 1);
	const int B_NUM = (int)(PD_PI / ALPHA_INCREMENT + 1);

	int startB = m_threadPublicVal.startB;
	int endB = m_threadPublicVal.endB;
	int startAlphaK = m_threadPublicVal.startAlphaK;
	int endAlphaK = m_threadPublicVal.endAlphaK;
	int startX = m_threadPublicVal.startX;
	int endX = m_threadPublicVal.endX;
	int startY = m_threadPublicVal.startY;
	int endY = m_threadPublicVal.endY;

	int tempX = 0, tempY = 0, tempB = 0, tempAlphaK = 0;
	int x = 0, y = 0, b = 0, alphaK = 0;

	
	for (int tempB = startB; tempB < min(endB, a); tempB ++)
	{
		for (int _alphaK = startAlphaK; _alphaK < endAlphaK; _alphaK ++)
		{
			tempAlphaK = _alphaK;
			tempAlphaK = (tempAlphaK + alphaKLen) % alphaKLen;
			//�������a, b, alpha�µ���ԲԲ�ܸ���
			tempPoints = m_srcPeripheryPoints[a * A_NUM + tempB * B_NUM + tempAlphaK];
			if (!tempPoints)
			{
				continue;
			}
			tempPointCount = tempPoints[0].outPointVal;
			
			minX = tempPoints[1].inPointVal;
			maxX = tempPoints[1].outPointVal;
			minY = tempPoints[2].inPointVal;
			maxY = tempPoints[2].outPointVal;

			minX = max(m_threadPublicVal.minX - minX + OUT_OFFSET, startX);
			maxX = min(m_threadPublicVal.maxX - maxX - OUT_OFFSET, endX);
			minY = max(m_threadPublicVal.minY - minY + OUT_OFFSET, startY);
			maxY = min(m_threadPublicVal.maxY - maxY - OUT_OFFSET, endY);

			for (int tempX = minX; tempX < maxX; tempX ++)
			{
				for (int tempY = minY; tempY < maxY; tempY ++)
				{
					tempVal = SrcImgEllipseDiffCalcu(tempX, tempY, tempPointCount, tempPoints + 3);
					if (maxVal < tempVal)
					{
						maxVal = tempVal;
						x = tempX;
						y = tempY;
						b = tempB;
						alphaK = tempAlphaK;
					}//if
				}//y
			}//x
		}//alpha
		if (a == tempB)continue;
	}//b

	m_srcThreadEllipse_DiffVal[aIndex].maxVal = maxVal;
	m_srcThreadEllipse_DiffVal[aIndex].a = a;
	m_srcThreadEllipse_DiffVal[aIndex].b = b;
	m_srcThreadEllipse_DiffVal[aIndex].x = x;
	m_srcThreadEllipse_DiffVal[aIndex].y = y;
	m_srcThreadEllipse_DiffVal[aIndex].alpha = alphaK * ALPHA_INCREMENT;

	m_srcThreadEndLabel[aIndex] = true;
	
	return 1;
}

/* ������Բ�߽����ص��ݶ�ƽ��ֵ */
inline double PupilDetection::SrcImgEllipseDiffCalcu(const int _x, const int _y, const int count, PDPointVal *points)
{

	//������ĻҶ�ֵ����һ��ֵ�����ų��������
	if ((m_staticSrcImg.data[_y * SRCIMGWIDTH + _x]) > (m_coarsePosThreshold + CENTER_THRESHOLD_EXTENT))
	{
		return -1;
	}
	double valueAccum;
	int incrementVal = _y * SRCIMGWIDTH + _x;
	int imgOutElement = 0;
	int imgInElement = 0;
	int tempOutElement0, tempOutElement1, tempOutElement2, tempOutElement3, tempOutElement4 = 0;
	int tempInElement0, tempInElement1, tempInElement2, tempInElement3, tempInElement4 = 0;
	int outPos0, inPos0, outPos1, inPos1, outPos2, inPos2, outPos3, inPos3, outPos4, inPos4;
	int limit = (count / 5) * 5;

	PDPointVal tempP0, tempP1, tempP2, tempP3, tempP4;
	int k = 0;
	for (; k < limit; k += 5)
	{
		//��Բ�ϵ�����ļ���
		tempP0 = points[k];
		tempP1 = points[k + 1];
		tempP2 = points[k + 2];
		tempP3 = points[k + 3];
		tempP4 = points[k + 4];

		outPos0 = tempP0.outPointVal + incrementVal;
		inPos0 = tempP0.inPointVal + incrementVal;
		outPos1 = tempP1.outPointVal + incrementVal;
		inPos1 = tempP1.inPointVal + incrementVal;
		outPos2 = tempP2.outPointVal + incrementVal;
		inPos2 = tempP2.inPointVal + incrementVal;
		outPos3 = tempP3.outPointVal + incrementVal;
		inPos3 = tempP3.inPointVal + incrementVal;
		outPos4 = tempP4.outPointVal + incrementVal;
		inPos4 = tempP4.inPointVal + incrementVal;

		tempOutElement0 = m_staticSrcImg.data[outPos0];
		tempInElement0 = m_staticSrcImg.data[inPos0];
		tempOutElement1 = m_staticSrcImg.data[outPos1];
		tempInElement1 = m_staticSrcImg.data[inPos1];
		tempOutElement2 = m_staticSrcImg.data[outPos2];
		tempInElement2 = m_staticSrcImg.data[inPos2];
		tempOutElement3 = m_staticSrcImg.data[outPos3];
		tempInElement3 = m_staticSrcImg.data[inPos3];
		tempOutElement4 = m_staticSrcImg.data[outPos4];
		tempInElement4 = m_staticSrcImg.data[inPos4];

		imgOutElement += tempOutElement0 + tempOutElement1 + tempOutElement2 + tempOutElement3 + tempOutElement4;
		imgInElement += tempInElement0 + tempInElement1 + tempInElement2 + tempInElement3 + tempInElement4;

	}//for k
	for (; k < count; k ++)
	{
		//��Բ�ϵ�����ļ���
		tempP0 = points[k];
		outPos0 = tempP0.outPointVal + incrementVal;
		inPos0 = tempP0.inPointVal + incrementVal;

		tempOutElement0 = m_staticSrcImg.data[outPos0];
		tempInElement0 = m_staticSrcImg.data[inPos0];

		imgOutElement += tempOutElement0;
		imgInElement += tempInElement0;

	}//for k
	valueAccum = imgOutElement - imgInElement;

	return valueAccum / count;
}

/* �������a, b, alpha�µ���ԲԲ�ܸ��� */
void PupilDetection::SrcImgPeripheryPointCalcu(const int a, const int b, const int alphaK)
{

	double dOutX = -1, dOutY = -1;
	double dInX = -1, dInY = -1;
	int oldOutX = -1, oldOutY = -1;
	int oldInX = -1, oldInY = -1;
	int outX, outY, inX, inY;
	PDPointVal tempPoint;
	double tempCosTheta, tempSinTheta;

	int count = 3;
	int minX = 0;
	int maxX = 0;
	int minY = 0;
	int maxY = 0;

	const double COS_OUT_A = m_cosAlpha[alphaK] * (a + OUT_OFFSET);
	const double COS_OUT_B = COS_OUT_A * b / a;
	const double SIN_OUT_A = m_sinAlpha[alphaK] * (a + OUT_OFFSET);
	const double SIN_OUT_B = SIN_OUT_A * b / a;

	if ((b - IN_OFFSET) <= 0)return;
//	if ((a - OUT_OFFSET) * b / a <= 0)return;

	const double COS_IN_A = m_cosAlpha[alphaK] * (a - IN_OFFSET);
	const double COS_IN_B = COS_IN_A * b / a;
	const double SIN_IN_A = m_sinAlpha[alphaK] * (a - IN_OFFSET);
	const double SIN_IN_B = SIN_IN_A * b / a;

	const int MAX_K = TWOPI / THETA_INCREMENT;
	PDPointVal *tempPoints = new PDPointVal[MAX_K + 3];

	for (int k = 0; k < MAX_K; k ++)
	{
		//����Բ�ϵ�����ļ���
		tempCosTheta = m_cosTheta[k];
		tempSinTheta = m_sinTheta[k];

		dOutX = COS_OUT_B * tempCosTheta - SIN_OUT_A * tempSinTheta;
		dOutY = SIN_OUT_B * tempCosTheta + COS_OUT_A * tempSinTheta;   /******����Բ���*******/

		dInX = COS_IN_B * tempCosTheta - SIN_IN_A * tempSinTheta;
		dInY = SIN_IN_B * tempCosTheta + COS_IN_A * tempSinTheta;

		outX = cvRound(dOutX);
		outY = cvRound(dOutY);

		inX = cvRound(dInX);
		inY = cvRound(dInY);

		//ֻͳ���µ�����
		if ((inX != oldInX || inY != oldInY) && (outX != oldOutX || outY != oldOutY))     /*���ص��ظ�����*/
		{

			if (outX < minX)minX = outX;
			else if (outX > maxX)maxX = outX;
			if (outY < minY)minY = outY;
			else if (outY > maxY)maxY = outY;

			tempPoint.outPointVal = outY * SRCIMGWIDTH + outX;
			tempPoint.inPointVal = inY * SRCIMGWIDTH + inX;          /*inx,iny...Ϊ�������*/
			tempPoints[count] = tempPoint;
			count ++;

			oldOutX = outX;
			oldOutY = outY;
			oldInX = inX;
			oldInY = inY;
		}

	}

	count -= 3;   /*��ǰ������һЩ����������*/
	if (count < SRCIMG_MIN_POINT)//tempPoints[0].outPointVal = - 1;
	{
		delete[]tempPoints;   /*delete������÷�*/
		tempPoints = NULL;
	}
	else
	{
		tempPoints[0].inPointVal = count;
		tempPoints[0].outPointVal = count;

		tempPoints[1].inPointVal = minX;
		tempPoints[1].outPointVal = maxX;

		tempPoints[2].inPointVal = minY;
		tempPoints[2].outPointVal = maxY;
	}

	m_srcPeripheryPoints[a * (SRCIMGHEIGHT / 2 + 1) * (int)(PD_PI / ALPHA_INCREMENT + 1) + b * (int)(PD_PI / ALPHA_INCREMENT + 1) + alphaK] = tempPoints;      /*tempoints����Բ�㼯����ָ��*/

	return;
}

/* �Դ���ͼ����д��� */
int PupilDetection::ImgProcessing(void)
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
	vector<vector<Point>> contours;
	findContours(thresholdImg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	const int offset1 = 1;
	const int offset2 = 3;
	int width = m_img.cols;
	int height = m_img.rows;
	int topElement, bottomElement, leftElement, rightElement;
	int avgElement;
	int elements[4];
	int highEle[4] = {-1}, lowEle[4] = {-1};
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
	int i = 0, h = 0, l = 0, t = 0, j = 0, k = 0;

	double tempCosTheta, tempSinTheta;
	const int MAX_K = TWOPI / THETA_INCREMENT;
	int fillingX, fillingY;
	for (i = 0; i < contoursSize; i ++)
	{
		if (contours[i].size() < CONTOURS_MIN_SIZE || contours[i].size() > CONTOURS_MAX_SIZE)
		{
			continue;    /*��������forѭ��*/
		}

		minEnclosingCircle(contours[i], circleCenter, cirCleRadius);   /*��С���Բ��opencv�Դ�*/
		x = circleCenter.x;
		y = circleCenter.y;
		r = cvRound(cirCleRadius) + G_L_OFFSET;
		r = r + GET_POINT_OFFSET;

		if ((x + 2 * r > width - SRCIMG_FRAME_THRESHOLD || x - 2 * r < SRCIMG_FRAME_THRESHOLD) || (y + 2 * r > height || y - 2 * r < 0))
		{
			continue;
		}

		//�������
		topElement = (m_img.data[(y - r) * width + x] + m_img.data[(y - r) * width + x + 1] + m_img.data[(y - r + 1) * width + x]
			+ m_img.data[(y - r) * width + x - 1] + m_img.data[(y - r - 1) * width + x]) / 5;
		bottomElement = (m_img.data[(y + r) * width + x] + m_img.data[(y + r) * width + x + 1] + m_img.data[(y + r + 1) * width + x]
			+ m_img.data[(y + r) * width + x - 1] + m_img.data[(y + r - 1) * width + x]) / 5;;
		leftElement = (m_img.data[y * width + (x - r)] + m_img.data[y * width + (x - r + 1)] + m_img.data[(y + 1) * width + (x - r)]
			+ m_img.data[y * width + (x - r - 1)] + m_img.data[(y - 1) * width + (x - r)]) / 5;
		rightElement = (m_img.data[y * width + (x + r)] + m_img.data[y * width + (x + r + 1)] + m_img.data[(y + 1) * width + (x + r)]
			+ m_img.data[y * width + (x + r - 1)] + m_img.data[(y - 1) * width + (x + r)]) / 5;

		r = r - GET_POINT_OFFSET;

		elements[0] = topElement;
		elements[1] = rightElement;
		elements[2] = bottomElement;
		elements[3] = leftElement;

		leftTopPoint.x = x - r;
		leftTopPoint.y = y - r;
		rightBottomPoint.x = x + r;
		rightBottomPoint.y = y + r;

		int maxElement = 0;
		int minElement = 0;
		//�����ֵ
		for (t = 1; t < 4; t ++)
		{
			if (elements[maxElement] < elements[t])maxElement = t;
			if (elements[minElement] > elements[t])minElement = t;
		}
		//����
		h = 0, l = 0;
		if (abs(elements[maxElement] - elements[minElement]) < G_L_DIFF_TH)		//�ĸ�ֵ����
		{
			for (t = 0; t < 4; t ++)
			{
				highEle[t] = elements[t];
				lowEle[t] = -1;			
			}
			h = 4; 
			l = 0;
		}
		//������
		else
		{
			for (t = 0; t < 4; t ++)
			{
				highEle[t] = -1;
				lowEle[t] = -1;
				//elements[t]�Ͻӽ� elements[maxElement]
				if ((elements[maxElement] - elements[t]) < (elements[t] - elements[minElement]))
				{
					highEle[h] = t;
					h ++;
				}
				else
				{
					lowEle[l] = t;
					l ++;
				}
			}
			//(0, 3)ʱΪ����(0, 1)�ĸ��ţ���Ϊ(3, 0)
			if ((0 == lowEle[0]) && (3 == lowEle[1]))
			{
				int tempEle = lowEle[0];
				lowEle[0] = lowEle[1];
				lowEle[1] = tempEle;
			}
		}
		rOffset = 2 * r;
		switch (h)
		{
		case 4:		//(4, 0)(�ߣ� ��)
			avgElement = (topElement + leftElement + bottomElement + rightElement) / 4;
			for (j = x - r; j < x + r; j ++)
			{
				for (k = y - r; k < y + r; k ++)
				{
					m_img.data[(k)* width + j] = avgElement;
				}
			}
			break;

		case 3:		//(3, 1)
			
			switch (lowEle[0])
			{
				//���¶���δ���������ճ��ʱ����ǲ�����Ҫ�Ķ������Ϊ /* @_n */
			case 0:		//top
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (leftElement + bottomElement + rightElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.x = x - r;
				linePoint2.x = x + r;
				for (k = y - rOffset; k < y; k ++)
				{
					tempOffset1 = m_img.data[(k + offset1) * width + linePoint1.x] - m_img.data[(k - offset2) * width + linePoint1.x];
					tempOffset2 = m_img.data[(k + offset1) * width + linePoint2.x] - m_img.data[(k - offset2) * width + linePoint2.x];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.y = k;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.y = k;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				if (A * B >= 0)
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, elements[0], avgElement, A, B, C);
				}
				else
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, avgElement, elements[0], A, B, C);
				}
				break;

			case 1:		//right
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (leftElement + bottomElement + topElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.y = y - r;
				linePoint2.y = y + r;
				for (j = x; j < x + rOffset; j ++)
				{
					tempOffset1 = m_img.data[linePoint1.y * width + j - offset1] - m_img.data[linePoint1.y * width + j + offset2];
					tempOffset2 = m_img.data[linePoint2.y * width + j - offset1] - m_img.data[linePoint2.y * width + j + offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = j;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = j;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, avgElement, elements[1], A, B, C);
				break;

			case 2:		//bottom
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (leftElement + topElement + rightElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.x = x - r;
				linePoint2.x = x + r;
				for (k = y; k < y + rOffset; k ++)
				{
					tempOffset1 = m_img.data[(k - offset1) * width + linePoint1.x] - m_img.data[(k + offset2) * width + linePoint1.x];
					tempOffset2 = m_img.data[(k - offset1) * width + linePoint2.x] - m_img.data[(k + offset2) * width + linePoint2.x];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.y = k;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.y = k;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				if (A * B >= 0)
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, avgElement, elements[0], A, B, C);
				}
				else
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, elements[2], avgElement, A, B, C);
				}
				break;

			case 3:		//right
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (rightElement + bottomElement + topElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.y = y - r;
				linePoint2.y = y + r;
				for (j = x; j < x + rOffset; j ++)
				{
					tempOffset1 = m_img.data[linePoint1.y * width + j + offset1] - m_img.data[linePoint1.y * width + j - offset2];
					tempOffset2 = m_img.data[linePoint2.y * width + j + offset1] - m_img.data[linePoint2.y * width + j - offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = j;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = j;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, elements[3], avgElement, A, B, C);
				break;

			default:
				break;
			}
			break;

		case 2:		//(2, 2)
			int filledVal1, filledVal2;
			filledVal1 = (elements[highEle[0]] + elements[highEle[1]]) / 2;
			filledVal2 = (elements[lowEle[0]] + elements[lowEle[1]]) / 2;
			int rOffset1, rOffset2;
			rOffset1 = 2 * r;
			rOffset2 = r;
			CvPoint tempP1, tempP2;
			switch (lowEle[0])
			{
				//���¶���δ���������ճ��ʱ����ǲ�����Ҫ�Ķ������Ϊ /* @_n */
			case 0:		//top right			
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				tempP1.x = x - rOffset1;
				tempP1.y = y - rOffset2;
				tempP2.x = x + rOffset2;
				tempP2.y = y + rOffset1;
				for (k = 0; k < rOffset1; k ++)
				{
					tempOffset1 = m_img.data[(tempP1.y + offset1) * width + tempP1.x - offset1]
						- m_img.data[(tempP1.y - offset2) * width + tempP1.x + offset2];
					tempOffset2 = m_img.data[(tempP2.y + offset1) * width + tempP2.x - offset1]
						- m_img.data[(tempP2.y - offset2) * width + tempP2.x + offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = tempP1.x;
						linePoint1.y = tempP1.y;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = tempP2.x;
						linePoint2.y = tempP2.y;
					}
					tempP1.x ++;
					tempP1.y --;
					tempP2.x ++;
					tempP2.y --;
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, filledVal1, filledVal2, A, B, C);
				break;

			case 1:		//right bottom
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				tempP1.x = x - rOffset1;
				tempP1.y = y + rOffset2;
				tempP2.x = x + rOffset2;
				tempP2.y = y - rOffset1;
				for (k = 0; k < rOffset1; k ++)
				{
					tempOffset1 = m_img.data[(tempP1.y - offset1) * width + tempP1.x - offset1]
						- m_img.data[(tempP1.y + offset2) * width + tempP1.x + offset2];
					tempOffset2 = m_img.data[(tempP2.y - offset1) * width + tempP2.x - offset1]
						- m_img.data[(tempP2.y + offset2) * width + tempP2.x + offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = tempP1.x;
						linePoint1.y = tempP1.y;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = tempP2.x;
						linePoint2.y = tempP2.y;
					}
					tempP1.x ++;
					tempP1.y ++;
					tempP2.x ++;
					tempP2.y ++;
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, filledVal1, filledVal2, A, B, C);
				break;

			case 2:		//bottom left
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				tempP1.x = x + rOffset1;
				tempP1.y = y + rOffset2;
				tempP2.x = x + rOffset2;
				tempP2.y = y - rOffset1;
				for (k = 0; k < rOffset1; k ++)
				{
					tempOffset1 = m_img.data[(tempP1.y - offset1) * width + tempP1.x + offset1]
						- m_img.data[(tempP1.y + offset2) * width + tempP1.x - offset2];
					tempOffset2 = m_img.data[(tempP2.y - offset1) * width + tempP2.x + offset1]
						- m_img.data[(tempP2.y + offset2) * width + tempP2.x - offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = tempP1.x;
						linePoint1.y = tempP1.y;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = tempP2.x;
						linePoint2.y = tempP2.y;
					}
					tempP1.x --;
					tempP1.y ++;
					tempP2.x --;
					tempP2.y ++;
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, filledVal2, filledVal1, A, B, C);
				break;

			case 3:		//left top
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				tempP1.x = x + rOffset1;
				tempP1.y = y - rOffset2;
				tempP2.x = x - rOffset2;
				tempP2.y = y + rOffset1;
				for (k = 0; k < rOffset1; k ++)
				{
					tempOffset1 = m_img.data[(tempP1.y + offset1) * width + tempP1.x + offset1]
						- m_img.data[(tempP1.y - offset2) * width + tempP1.x - offset2];
					tempOffset2 = m_img.data[(tempP2.y + offset1) * width + tempP2.x + offset1]
						- m_img.data[(tempP2.y - offset2) * width + tempP2.x - offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = tempP1.x;
						linePoint1.y = tempP1.y;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = tempP2.x;
						linePoint2.y = tempP2.y;
					}
					tempP1.x --;
					tempP1.y --;
					tempP2.x --;
					tempP2.y --;
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, filledVal2, filledVal1, A, B, C);
				break;

			default:
				break;
			}
			break;

		case 1:		//(1, 3)
			switch (highEle[0])
			{
				//���¶���δ���������ճ��ʱ����ǲ�����Ҫ�Ķ������Ϊ /* @_n */
			case 0:		//top
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (leftElement + bottomElement + rightElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.x = x - r;
				linePoint2.x = x + r;
				for (k = y - rOffset; k < y; k ++)
				{
					tempOffset1 = m_img.data[(k - offset1) * width + linePoint1.x] - m_img.data[(k + offset2) * width + linePoint1.x];
					tempOffset2 = m_img.data[(k - offset1) * width + linePoint2.x] - m_img.data[(k + offset2) * width + linePoint2.x];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.y = k;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.y = k;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				if (A * B >= 0)
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, elements[0], avgElement, A, B, C);
				}
				else
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, avgElement, elements[0], A, B, C);
				}
				break;

			case 1:		//right
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (leftElement + bottomElement + topElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.y = y - r;
				linePoint2.y = y + r;
				for (j = x; j < x + rOffset; j ++)
				{
					tempOffset1 = m_img.data[linePoint1.y * width + j + offset1] - m_img.data[linePoint1.y * width + j - offset2];
					tempOffset2 = m_img.data[linePoint2.y * width + j + offset1] - m_img.data[linePoint2.y * width + j - offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = j;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = j;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, avgElement, elements[1], A, B, C);
				break;

			case 2:		//bottom
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (leftElement + topElement + rightElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.x = x - r;
				linePoint2.x = x + r;
				for (k = y; k < y + rOffset; k ++)
				{
					tempOffset1 = m_img.data[(k + offset1) * width + linePoint1.x] - m_img.data[(k - offset2) * width + linePoint1.x];
					tempOffset2 = m_img.data[(k + offset1) * width + linePoint2.x] - m_img.data[(k - offset2) * width + linePoint2.x];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.y = k;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.y = k;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				if (A * B >= 0)
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, avgElement, elements[0], A, B, C);
				}
				else
				{
					SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, elements[2], avgElement, A, B, C);
				}
				break;

			case 3:		//right
				/* @_0 leftElement,bottomElement,rightElement��ѡ�� */
				avgElement = (rightElement + bottomElement + topElement) / 3;
				maxOffset1 = 0;
				maxOffset2 = 0;
				tempOffset1 = 0;
				tempOffset2 = 0;
				/* @_2 ��k�ı����Լ�(k + offset) * width + linePoint1.x�ȵļ��㣬���ж�linePoint1, linePoint2��x, yֵ */
				linePoint1.y = y - r;
				linePoint2.y = y + r;
				for (j = x; j < x + rOffset; j ++)
				{
					tempOffset1 = m_img.data[linePoint1.y * width + j - offset1] - m_img.data[linePoint1.y * width + j + offset2];
					tempOffset2 = m_img.data[linePoint2.y * width + j - offset1] - m_img.data[linePoint2.y * width + j + offset2];
					if (tempOffset1 > maxOffset1)
					{
						maxOffset1 = tempOffset1;
						linePoint1.x = j;
					}
					if (tempOffset2 > maxOffset2)
					{
						maxOffset2 = tempOffset2;
						linePoint2.x = j;
					}
				}
				//��������ȷ����ֱ��
				LinearEquationCalcu(linePoint1, linePoint2, A, B, C);
				SpotAreaFill(m_img, leftTopPoint, rightBottomPoint, elements[3], avgElement, A, B, C);
				break;

			default:
				break;
			}
			break;

		default:
			break;
		}

	}


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
	//����ǰһ֡ͼ�������Ӧ����ֵ����������Χ            /*���ٱ�������ֵ����������һ֡�ӽ�*/
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
		handle[i] = CreateThread(NULL, 0, PupilCoarsePositioning, (LPVOID)i, 0, NULL);     /*һ������i���ɿ����̱߳�����Χ*/
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
		if (minVal > m_coarsePosThVals[i].coarsePosValAccum)   /*��С��ֵ����Ѵֶ�λ*/
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
	if (minVal > SHRINK_AREA_MINVAL_TH)     /*��֤ͫ�׽ӽ�Բ*/
	{
		return SHRINK_AREA_FAILED;
	}

	//����offsetֵ
	m_offset = (radius / 30) + 1;   /*����ͫ�״�Сȷ�����������Բƫ����*/
	/*OUT_OFFSET = (radius / 60) + 2;
	IN_OFFSET = 2 * OUT_OFFSET;*/

//#if SHOW_IMG
	if (showImg)
	{
	//	PDDrawRect(m_coarsePosImg, Point(m_threadPublicVal.minX, m_threadPublicVal.minY), Point(m_threadPublicVal.maxX, m_threadPublicVal.maxY));
		threshold(m_coarsePosImg, thresholdImg, m_coarsePosThreshold, 255, CV_THRESH_BINARY_INV);
		imshow("m_coarsePosImg", thresholdImg);
		waitKey(0);
	}
//#endif
	//����ԭͼ
	m_img.copyTo(m_srcImg);
	//��С�ֱ���
	resize(m_img, m_img, Size(IMGWIDTH, IMGHEIGHT));

	return PD_SUCCEED;
}

/* �����������ȷ����ֱ�� A * x + B * y + C = 0 */
void PupilDetection::LinearEquationCalcu(CvPoint &point1, CvPoint &point2, int &A, int &B, int &C)
{
	A = point2.y - point1.y;
	B = point1.x - point2.x;
	C = point2.x * point1.y - point1.x * point2.y;
	if (A > 0)
	{
		return;
	}
	else if (A < 0)
	{
		A = - A;
		B = - B;
		C = - C;
		return;
	}
	else if (B < 0)
	{
		B = - B;
		C = - C;
		return;
	}
}

/* �Թ������������,�� A*x + B*y + C < 0�������filledVal1 */
void PupilDetection::SpotAreaFill(Mat &img, CvPoint leftTopPoint, CvPoint rightBottomPoint, int filledVal1, int filledVal2,
	int A, int B, int C)
{
	int width = img.cols;
	int a = 0, b = 0, c = 0;

	for (int x = leftTopPoint.x; x < rightBottomPoint.x; x ++)
	{
		for (int y = leftTopPoint.y; y < rightBottomPoint.y; y ++)
		{
			if (A * x + B * y + C < 0)
			{
				img.data[y * width + x] = filledVal1;
				a ++;
			}
			else if (A * x + B * y + C == 0)
			{
				img.data[y * width + x] = (filledVal1 + filledVal2) / 2;
				b ++;
			}
			else
			{
				img.data[y * width + x] = filledVal2;
				c ++;
			}
		}
	}

}

/* ͫ�״ֶ�λ����������̺߳��� */
DWORD WINAPI PupilDetection::PupilCoarsePositioning(LPVOID threadIndex)
{
	const uchar SEPARATION_OF_TONES_STEP = G_L_THRESHOLD / (SEPARATION_OF_TONES_NUM - 1);
	int threadId = (int)threadIndex;
	int coarsePosThreshold = threadId * SEPARATION_OF_TONES_STEP;
	//��ֵ��
	Mat thresholdImg;
	threshold(m_coarsePosImg, thresholdImg, coarsePosThreshold, 255, CV_THRESH_BINARY_INV);
	
	/*stringstream ss;
	ss << (int)coarsePosThreshold;
	string s = ss.str();
	Mat tempImg;
	thresholdImg.copyTo(tempImg);
	imshow("thresholdImg "+s, thresholdImg);
	waitKey(0);*/
	
	//��ͼ��ƽ������
//	GaussianBlur(thresholdImg, thresholdImg, Size(5, 5), 0);

	//������
	vector<vector<Point>> contours;
	findContours(thresholdImg, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	const double ratioR = 2;

	Point circleCenter(0, 0), tempCircleCenter;
	int circleRadius = 0, tempCircleRadius;
	int contoursSize = contours.size();
	int i = 0, h = 0, l = 0, t = 0, j = 0, k = 0;
	int minX = SRCIMG_FRAME_THRESHOLD;
	int maxX = SRCIMGWIDTH - SRCIMG_FRAME_THRESHOLD;
	int minY = 0;
	int maxY = SRCIMGHEIGHT;
	double valueAccum = -1, tempValueAccum;
	bool changedLabel = false;

	for (i = 0; i < contoursSize; i ++)
	{
		if (ContourScreening(contours[i], tempValueAccum, tempCircleCenter, tempCircleRadius))
		{
			//��һ��
			if (!changedLabel)
			{
				valueAccum = tempValueAccum;
				circleCenter.x = tempCircleCenter.x;
				circleCenter.y = tempCircleCenter.y;
				circleRadius = tempCircleRadius;
				changedLabel = true;
				continue;
			}
			if (valueAccum > tempValueAccum)                       /*ȡ�б����ڷֲ��ֵ��һ����ֵ����Сֵ��Ӧ�Ĵֶ�λ*/
			{
				valueAccum = tempValueAccum;
				circleCenter.x = tempCircleCenter.x;
				circleCenter.y = tempCircleCenter.y;
				circleRadius = tempCircleRadius;
			}
			
		}
			
	}

	//�ֶ�λ��Ч
	if (changedLabel && (valueAccum < m_coarsePosValAccum))
	{
		m_coarsePosThVals[threadId].coarsePosValAccum = valueAccum;
		m_coarsePosThVals[threadId].coarsePosThreshold = (int)coarsePosThreshold;
		m_coarsePosThVals[threadId].minX = max(int(circleCenter.x - ratioR * circleRadius), minX);
		m_coarsePosThVals[threadId].maxX = min(int(circleCenter.x + ratioR * circleRadius), maxX);
		m_coarsePosThVals[threadId].minY = max(int(circleCenter.y - ratioR * circleRadius), minY);
		m_coarsePosThVals[threadId].maxY = min(int(circleCenter.y + ratioR * circleRadius), maxY);
		m_coarsePosThVals[threadId].radius = circleRadius;
	}
	else m_coarsePosThVals[threadId].coarsePosValAccum = 1;

	m_coarsePosThLabel[threadId] = true;
/*
	PDDrawRect(tempImg, Point(m_minX, m_minY), Point(m_maxX, m_maxY));
	imshow("thresholdImg", tempImg);
	waitKey(0);
*/
	return 1;
}

/* ����ͫ�״ֶ�λʱ������ɸѡ */
bool PupilDetection::ContourScreening(vector<Point> &contours, double &valueAccum, Point &circleCenter, int &circleRadius)
{

	Point2f circleC;
	float circleR;
	int contoursSize = contours.size();
	int x, y, r, R;
	int i, j;
	double ratio;
	int tempMinX, tempMinY, tempMaxX, tempMaxY;

	if (!contoursSize)
	{
		return false;
	}

	//��������Ӧ��һ����Χ��
	if (contoursSize < 50 || contoursSize > 800)
	{
		return false;
	}

	minEnclosingCircle(contours, circleC, circleR);
	x = circleC.x;
	y = circleC.y;
	r = cvRound(circleR);

	tempMinX = contours[0].x;
	tempMaxX = contours[0].x;
	tempMinY = contours[0].y;
	tempMaxY = contours[0].y;
	for (j = 1; j < contoursSize; j ++)
	{
		if (tempMinX > contours[j].x)tempMinX = contours[j].x;
		if (tempMaxX < contours[j].x)tempMaxX = contours[j].x;
		if (tempMinY > contours[j].y)tempMinY = contours[j].y;
		if (tempMaxY < contours[j].y)tempMaxY = contours[j].y;
	}
	ratio = float(tempMaxY - tempMinY) / (tempMaxX - tempMinX);
	ratio = ratio < 1 ? ratio : 1 / ratio;
	//���ڱ�ƽ����ȥ
	if (ratio < RATIO_THRESHOLD)return false;

	//���������r�Ĳ�ֵ�ľ���ֵƽ��ֵ                       
	float diffX, diffY;
	valueAccum = 0;
	for (i = 0; i < contoursSize; i ++)
	{
		diffX = float(contours[i].x - x) / r;
		diffY = float(contours[i].y - y) / r;

		valueAccum += abs(diffX * diffX + diffY * diffY - 1);
	}
	valueAccum = valueAccum / contoursSize;

	circleCenter.x = x;
	circleCenter.y = y;       /*��ͼ�ֶ�λ��ͫ�׼���С*/
	circleRadius = r;

	return true;
}

void PDDrawRect(Mat &img, Point pt1, Point pt2)
{
	int i = 0, j = 0;
	int width = img.cols, height = img.rows;

	if (pt1.x < 0 || pt1.x > width || pt1.y < 0 || pt1.y > height)
	{
		return;
	}
	if (pt2.x < 0 || pt2.x > width || pt2.y < 0 || pt2.y > height)
	{
		return;
	}

	for (i = pt1.x; i < pt2.x; i ++)
	{
		img.data[pt1.y * width + i] = 255;
		img.data[pt2.y * width + i] = 255;
	}
	for (j = pt1.y; j < pt2.y; j ++)
	{
		img.data[j * width + pt1.x] = 255;
		img.data[j * width + pt2.x] = 255;
	}
	return;
}

void PupilDetection::PDDrawCircle(Mat &img, int r, int _x, int _y)
{
	int oldX = 0;
	int oldY = 0;
	int imgWidth = img.cols;

	int centerX = _x;
	int centerY = _y;

	int x = -1, y = -1;
	float d_x = -1, d_y = -1;
	const int MAX_K = TWOPI / THETA_INCREMENT;


	for (int k = 0; k < MAX_K; k ++)
	{
		//��Բ�ϵ�����ļ���
		d_x = r * m_cosTheta[k] + centerX;
		d_y = r * m_sinTheta[k] + centerY;

		x = cvRound(d_x);
		y = cvRound(d_y);

		
		//ֻͳ���µ�����
		if (x != oldX || y != oldY)
		{
			img.data[y * imgWidth + x] = 255;
			oldX = x;
			oldY = y;

		}

	}//for theta

	return;
}