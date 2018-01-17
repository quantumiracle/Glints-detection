
#include "PupilDetection.h"

/* 将PDEllipse各个数转换成string */
void ConvertTostring(PDEllipse &srcEllipse, vector<string> &vecStr);
/* 视频实时测试 */
void PupilDetecVideoTest(string testVideoPath, string preName, string resultFilePreName);

#define SAVE_VIDEO	1		//是否保存视频 (1：保存  0：不保存)
//#define SHOW_IMG	1		//是否显示图像（调试用）
bool showImg = false;

/* 对样本进行测试 */

int PupilDetectionTest(void)
{
	/* 参数 */
	/************************************************************************/
	// 0为在此处（VS2013）测试
	// 1为在封装文件中测试
	// 2为调试用
	// 3为视频实时测试

	int choice = 0;				

	int imgNum = 200;			//从文件中最多测试图像的数量

	/************************************************************************/

	//读取图像路径
	string testImgPath;
	string testVideoPath;
	//保存图片路径，"eye_" 为文件名前缀
	string preName;
	string extend = "*.bmp";		//图像类型
	//保存测试结果数据路径
	string resultFilePreName = "./";		// txt文件存放文件夹路径
	string resultFileName = "resultFile5.txt";
	switch (choice)
	{
		//在此处（VS2013）测试
	case 0:
		testImgPath = "./images/src/";
		preName = "./images/des/eye_";
		break;
		//在封装文件中测试
	case 1:
		testImgPath = "./srcImg/1/";
		preName = "./dstImg/1/eye_";
		break;
		//调试用
	case 2:
		testImgPath = "./images/1/";
		preName = "./images/debug/eye_";
		break;
		//视频实时测试
	case 3:
		testVideoPath = "E:/OpenCVFile/PupilDetection/src/avi2/";		// 视频源路径
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
	//读取图像
	bool imgFind = false;
	vector<string> imgPath = dir.GetListFiles(testImgPath, extend, imgFind);

	vector<PDEllipse> vecEllipse;
	vector<int> vecThreshold;
	PDEllipse resultEllipse;

	initStart = clock();
//	PupilDetection pulDetec;
	PupilDetection *pulDetec = new PupilDetection();     /******执行构造*******/
	initnEnd = clock();
	cout << "初始化时间为：" << initnEnd - initStart << " ms" << endl << endl;

	tempStart = clock();

	if (imgNum > imgPath.size())
	{
		imgNum = imgPath.size();
	}
	//将图像放入vecReadMats中
	for (int k = 0; k < imgNum; k ++)
	{
		string imgName = imgPath[k];
		string imgFullName = testImgPath + imgName;
		img = imread(imgFullName);
		vecReadMats.push_back(img);
	}

	tempEnd = clock();
	cout << "读取样本时间为：" << tempEnd - tempStart << " ms" << endl << "开始测试！" << endl << endl;
	start = clock();
	for (int i = 0; i < imgNum; i ++)
	{
//		cout << "第 " << i << " 张图 : " << imgPath[i] << endl;
		/*if (i == 390)
		{
			int s = 0;
		}*/

		img = vecReadMats[i];
		
		pulDetec->setImg(img);
		if (PD_SUCCEED != pulDetec->DetectPupil())
		{
			pulDetec->getImg(img);
			//失败则椭圆参数全部置0
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

	cout << "测试图片为：" << imgNum << "	总运行时间为：" << end - start << " ms" << endl;
	cout << "平均每张图片运行时间为：" << (end - start) / imgNum  << " ms" << endl;

	//保存图片及数据
	string fullName;
	stringstream ss;
	vector<string> vecStr;
	resultFilePreName += resultFileName;
	resultFileName = resultFilePreName;
	ofstream resultFile(resultFileName);
	if (!resultFile)
	{
#if _DEBUG
		cout << "创建文件 " << resultFileName << " 失败!" << endl;
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

	delete pulDetec;    /***********与new对应，此处跑析构************8*/
	pulDetec = NULL;

	cout << endl << "结束测试！" << endl;
	return PD_SUCCEED;

}


/* 将PDEllipse各个数转换成string */
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

/* 视频实时测试 */
void PupilDetecVideoTest(string testVideoPath, string preName, string resultFilePreName)
{
	clock_t start, end, tempStart, tempEnd;
	clock_t initStart, initnEnd;
	Mat img;
	VideoCapture videoCap;
	string imgFullName;
	Directory dir;
	//读取视频
	string extend = "*.avi";		//视频类型
	bool imgFind = false;
	vector<string> videoPath = dir.GetListFiles(testVideoPath, extend, imgFind);

	initStart = clock();
	PupilDetection pulDetec;
	initnEnd = clock();
	cout << "初始化时间为：" << initnEnd - initStart << " ms" << endl << endl;

	int videoNum = videoPath.size();
	Size frameSize(SRCIMGWIDTH, SRCIMGHEIGHT);
	stringstream ss;
	string writeFullName;
	int frameCount = 0;
	vector<PDEllipse> vecEllipse;
	PDEllipse resultEllipse;

	cout << "开始测试！" << endl << endl;
	start = clock();
	for (int i = 0; i < videoNum; i ++)
	{
		imgFullName.clear();
		imgFullName = testVideoPath + videoPath[i];

		videoCap.open(imgFullName);
		if (!videoCap.isOpened())
		{
			cout << "打开文件 " << videoPath[i] << " 失败 !" << endl;
			return;
		}
#if SAVE_VIDEO
		writeFullName = preName + videoPath[i];
		VideoWriter videoWriter(writeFullName, videoCap.get(CV_CAP_PROP_FOURCC), videoCap.get(CV_CAP_PROP_FPS), frameSize, false);

		// CV_FOURCC('M', 'J', 'P', 'G')
		if (!videoWriter.isOpened())
		{
			cout << "创建文件 " << writeFullName << " 失败 !" << endl;
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
				//失败则椭圆参数全部置0
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

		//保存图片及数据
		string fullName;
		stringstream ss;
		vector<string> vecStr;
		string resultFileName = resultFilePreName + videoPath[i] + ".txt";
		ofstream resultFile(resultFileName);
		if (!resultFile)
		{
#if _DEBUG
			cout << "创建文件 " << resultFileName << " 失败!" << endl;
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

		cout << "已完成测试视频数：" << i + 1 << " / " << videoNum << endl;
#if SAVE_VIDEO
		videoWriter.release();
#endif
	}

	end = clock();

	cout << "测试图片为：" << frameCount << "	总运行时间为：" << end - start << " ms" << endl;
	cout << "平均每张图片运行时间为：" << (end - start) / frameCount << " ms" << endl;

	cout << endl << "结束测试！" << endl;
	return;
}