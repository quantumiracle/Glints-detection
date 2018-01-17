
#include "PupilDetection.h"

/* ��PDEllipse������ת����string */
void ConvertTostring(PDEllipse &srcEllipse, vector<string> &vecStr);
/* ��Ƶʵʱ���� */
void PupilDetecVideoTest(string testVideoPath, string preName, string resultFilePreName);

#define SAVE_VIDEO	1		//�Ƿ񱣴���Ƶ (1������  0��������)
//#define SHOW_IMG	1		//�Ƿ���ʾͼ�񣨵����ã�
bool showImg = false;

/* ���������в��� */

int PupilDetectionTest(void)
{
	/* ���� */
	/************************************************************************/
	// 0Ϊ�ڴ˴���VS2013������
	// 1Ϊ�ڷ�װ�ļ��в���
	// 2Ϊ������
	// 3Ϊ��Ƶʵʱ����

	int choice = 0;				

	int imgNum = 200;			//���ļ���������ͼ�������

	/************************************************************************/

	//��ȡͼ��·��
	string testImgPath;
	string testVideoPath;
	//����ͼƬ·����"eye_" Ϊ�ļ���ǰ׺
	string preName;
	string extend = "*.bmp";		//ͼ������
	//������Խ������·��
	string resultFilePreName = "./";		// txt�ļ�����ļ���·��
	string resultFileName = "resultFile5.txt";
	switch (choice)
	{
		//�ڴ˴���VS2013������
	case 0:
		testImgPath = "./images/src/";
		preName = "./images/des/eye_";
		break;
		//�ڷ�װ�ļ��в���
	case 1:
		testImgPath = "./srcImg/1/";
		preName = "./dstImg/1/eye_";
		break;
		//������
	case 2:
		testImgPath = "./images/1/";
		preName = "./images/debug/eye_";
		break;
		//��Ƶʵʱ����
	case 3:
		testVideoPath = "E:/OpenCVFile/PupilDetection/src/avi2/";		// ��ƵԴ·��
		preName = "./eyeVideos/2/";
		PupilDetecVideoTest(testVideoPath, preName, resultFilePreName);
		return PD_SUCCEED;
	default:
		break;
	}

	clock_t start, end, tempStart, tempEnd;
	clock_t initStart, initnEnd;
	Mat img;
	vector<Mat> vecReadMats;
	vector<Mat> vecWriteMats;
	Directory dir; 
	//��ȡͼ��
	bool imgFind = false;
	vector<string> imgPath = dir.GetListFiles(testImgPath, extend, imgFind);

	vector<PDEllipse> vecEllipse;
	vector<int> vecThreshold;
	PDEllipse resultEllipse;

	initStart = clock();
//	PupilDetection pulDetec;
	PupilDetection *pulDetec = new PupilDetection();     /******ִ�й���*******/
	initnEnd = clock();
	cout << "��ʼ��ʱ��Ϊ��" << initnEnd - initStart << " ms" << endl << endl;

	tempStart = clock();

	if (imgNum > imgPath.size())
	{
		imgNum = imgPath.size();
	}
	//��ͼ�����vecReadMats��
	for (int k = 0; k < imgNum; k ++)
	{
		string imgName = imgPath[k];
		string imgFullName = testImgPath + imgName;
		img = imread(imgFullName);
		vecReadMats.push_back(img);
	}

	tempEnd = clock();
	cout << "��ȡ����ʱ��Ϊ��" << tempEnd - tempStart << " ms" << endl << "��ʼ���ԣ�" << endl << endl;
	start = clock();
	for (int i = 0; i < imgNum; i ++)
	{
//		cout << "�� " << i << " ��ͼ : " << imgPath[i] << endl;
		/*if (i == 390)
		{
			int s = 0;
		}*/

		img = vecReadMats[i];
		
		pulDetec->setImg(img);
		if (PD_SUCCEED != pulDetec->DetectPupil())
		{
			pulDetec->getImg(img);
			//ʧ������Բ����ȫ����0
			resultEllipse.a = resultEllipse.b = resultEllipse.x = resultEllipse.y = resultEllipse.alpha = 0;

			vecWriteMats.push_back(img);
			vecEllipse.push_back(resultEllipse);
			continue;
		}
		pulDetec->getImg(img);
		pulDetec->getEllipse(resultEllipse);

		vecWriteMats.push_back(img);
		vecEllipse.push_back(resultEllipse);
		vecThreshold.push_back(PupilDetection::m_coarsePosThreshold);
	}

	end = clock();

	cout << "����ͼƬΪ��" << imgNum << "	������ʱ��Ϊ��" << end - start << " ms" << endl;
	cout << "ƽ��ÿ��ͼƬ����ʱ��Ϊ��" << (end - start) / imgNum  << " ms" << endl;

	//����ͼƬ������
	string fullName;
	stringstream ss;
	vector<string> vecStr;
	resultFilePreName += resultFileName;
	resultFileName = resultFilePreName;
	ofstream resultFile(resultFileName);
	if (!resultFile)
	{
#if _DEBUG
		cout << "�����ļ� " << resultFileName << " ʧ��!" << endl;
#endif
		exit(0);
	}

	for (int j = 0; j < vecWriteMats.size(); j ++)
	{
//		resize(vecWriteMats[j], vecWriteMats[j], Size(360, 270));
		ss.str("");
		ss << j;
		fullName = preName + ss.str() + ".bmp";
		imwrite(fullName, vecWriteMats[j]);
		vecStr.clear();
		ConvertTostring(vecEllipse[j], vecStr);
		resultFile << "x: " << vecStr[0] << "  y: " << vecStr[1] << "  a: " << vecStr[2]
			<< "  b: " << vecStr[3] << "  alpha: " << vecStr[4] << "  threshold: " << vecThreshold[j] << "  " << fullName << '\t' << imgPath[j] << endl;

	}
	
	resultFile.close();

	delete pulDetec;    /***********��new��Ӧ���˴�������************8*/
	pulDetec = NULL;

	cout << endl << "�������ԣ�" << endl;
	return PD_SUCCEED;

}


/* ��PDEllipse������ת����string */
void ConvertTostring(PDEllipse &srcEllipse, vector<string> &vecStr)
{
	stringstream ss;
	string str;
	
	ss.str("");
	ss << srcEllipse.x;
	str = ss.str();
	vecStr.push_back(str);

	ss.str("");
	ss << srcEllipse.y;
	str = ss.str();
	vecStr.push_back(str);

	ss.str("");
	ss << srcEllipse.a;
	str = ss.str();
	vecStr.push_back(str);

	ss.str("");
	ss << srcEllipse.b;
	str = ss.str();
	vecStr.push_back(str);

	ss.str("");
	ss << srcEllipse.alpha;
	str = ss.str();
	vecStr.push_back(str);
	return;

}

/* ��Ƶʵʱ���� */
void PupilDetecVideoTest(string testVideoPath, string preName, string resultFilePreName)
{
	clock_t start, end, tempStart, tempEnd;
	clock_t initStart, initnEnd;
	Mat img;
	VideoCapture videoCap;
	string imgFullName;
	Directory dir;
	//��ȡ��Ƶ
	string extend = "*.avi";		//��Ƶ����
	bool imgFind = false;
	vector<string> videoPath = dir.GetListFiles(testVideoPath, extend, imgFind);

	initStart = clock();
	PupilDetection pulDetec;
	initnEnd = clock();
	cout << "��ʼ��ʱ��Ϊ��" << initnEnd - initStart << " ms" << endl << endl;

	int videoNum = videoPath.size();
	Size frameSize(SRCIMGWIDTH, SRCIMGHEIGHT);
	stringstream ss;
	string writeFullName;
	int frameCount = 0;
	vector<PDEllipse> vecEllipse;
	PDEllipse resultEllipse;

	cout << "��ʼ���ԣ�" << endl << endl;
	start = clock();
	for (int i = 0; i < videoNum; i ++)
	{
		imgFullName.clear();
		imgFullName = testVideoPath + videoPath[i];

		videoCap.open(imgFullName);
		if (!videoCap.isOpened())
		{
			cout << "���ļ� " << videoPath[i] << " ʧ�� !" << endl;
			return;
		}
#if SAVE_VIDEO
		writeFullName = preName + videoPath[i];
		VideoWriter videoWriter(writeFullName, videoCap.get(CV_CAP_PROP_FOURCC), videoCap.get(CV_CAP_PROP_FPS), frameSize, false);

		// CV_FOURCC('M', 'J', 'P', 'G')
		if (!videoWriter.isOpened())
		{
			cout << "�����ļ� " << writeFullName << " ʧ�� !" << endl;
			return;
		}
#endif
		for (int j = 0;; j ++)
		{
			/*cout << j << '\t';
			if (j % 10 == 9)cout << endl;*/

			/*if (206 == j)
			{
				showImg = true;
			}*/

			videoCap >> img;
			if (img.empty())
			{
				break;
			}
			
			pulDetec.setImg(img);
			if (PD_SUCCEED != pulDetec.DetectPupil())
			{
#if SAVE_VIDEO
				pulDetec.getImg(img);
				videoWriter << img;
#endif
				//ʧ������Բ����ȫ����0
				resultEllipse.a = resultEllipse.b = resultEllipse.x = resultEllipse.y = resultEllipse.alpha = 0;
				vecEllipse.push_back(resultEllipse);
				frameCount ++;
				continue;
			}
#if SAVE_VIDEO
			pulDetec.getImg(img);
			videoWriter << img;
#endif
//#if SHOW_IMG
			if (showImg)
			{
				imshow("saveImg", img);
				waitKey(0);
			}
//#endif
			pulDetec.getEllipse(resultEllipse);
			vecEllipse.push_back(resultEllipse);
			frameCount ++;
		}

		//����ͼƬ������
		string fullName;
		stringstream ss;
		vector<string> vecStr;
		string resultFileName = resultFilePreName + videoPath[i] + ".txt";
		ofstream resultFile(resultFileName);
		if (!resultFile)
		{
#if _DEBUG
			cout << "�����ļ� " << resultFileName << " ʧ��!" << endl;
#endif
			exit(0);
		}

		for (int j = 0; j < vecEllipse.size(); j ++)
		{
			vecStr.clear();
			ConvertTostring(vecEllipse[j], vecStr);
			resultFile << vecStr[0] << " " << vecStr[1] << " " << vecStr[2]
				<< " " << vecStr[3] << " " << vecStr[4] << endl;
		}

		resultFile.close();

		cout << "����ɲ�����Ƶ����" << i + 1 << " / " << videoNum << endl;
#if SAVE_VIDEO
		videoWriter.release();
#endif
	}

	end = clock();

	cout << "����ͼƬΪ��" << frameCount << "	������ʱ��Ϊ��" << end - start << " ms" << endl;
	cout << "ƽ��ÿ��ͼƬ����ʱ��Ϊ��" << (end - start) / frameCount << " ms" << endl;

	cout << endl << "�������ԣ�" << endl;
	return;
}