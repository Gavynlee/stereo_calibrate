#include "monocular.h"

monocular::monocular(int boardWidth, int boardHeight, int squareSize, string filespath): 
	mBoardWidth(boardWidth), mBoardHeight(boardHeight), mSquareSize(squareSize), mFilespath(filespath)
{
    mBoardCorner = boardWidth * boardHeight;
    mBoardSize = Size(boardWidth, boardHeight);
}

monocular::~monocular()
{
}

void monocular::calRealPoint(vector<vector<Point3f> >& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
{
	vector<Point3f> imgpoint;
	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
	{
		for (int colIndex = 0; colIndex < boardwidth; colIndex++)
		{
			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}

/*设置相机的初始参数 也可以不估计*/
void monocular::guessCameraParam()
{
	/*分配内存*/
	mIntrinsic.create(3, 3, CV_64FC1);           //相机内参数
	mDistortion_coeff.create(5, 1, CV_64FC1);    //畸变参数

	/*
	fx 0 cx
	0 fy cy
	0 0  1     内参数
	*/
	mIntrinsic.at<double>(0, 0) = 256.8093262;   //fx         
	mIntrinsic.at<double>(0, 2) = 160.2826538;   //cx  
	mIntrinsic.at<double>(1, 1) = 254.7511139;   //fy  
	mIntrinsic.at<double>(1, 2) = 127.6264572;   //cy  

	mIntrinsic.at<double>(0, 1) = 0;
	mIntrinsic.at<double>(1, 0) = 0;
	mIntrinsic.at<double>(2, 0) = 0;
	mIntrinsic.at<double>(2, 1) = 0;
	mIntrinsic.at<double>(2, 2) = 1;

	/*
	k1 k2 p1 p2 p3    畸变参数
	*/
	mDistortion_coeff.at<double>(0, 0) = -0.193740;  //k1  
	mDistortion_coeff.at<double>(1, 0) = -0.378588;  //k2  
	mDistortion_coeff.at<double>(2, 0) = 0.028980;   //p1  
	mDistortion_coeff.at<double>(3, 0) = 0.008136;   //p2  
	mDistortion_coeff.at<double>(4, 0) = 0;          //p3  
}

void monocular::saveCameraParam()
{
    FileStorage fs("./result/monocular.yaml", FileStorage::WRITE); 
	if(!fs.isOpened())
	{
		cout << "open file error!" << endl;
		return;
	}

    fs << "cameraMatrix" << mIntrinsic;
    fs << "distCoeffs" << mDistortion_coeff;

    fs << "the overall RMS re-projection error" << mdRMS_error;
    fs << "the Mean pixel re-projection error" << mdave_error;

    fs.release();  
}

void monocular::ReadCameraParam()
{
    FileStorage fs("./result/monocular.yaml", FileStorage::READ);   
	if(!fs.isOpened())
	{
		cout << "open file error!" << endl;
		return;
	}

    fs["cameraMatrix"] >> mIntrinsic;  
    fs["distCoeffs"] >> mDistortion_coeff;  

    cout << "cameraMatrix is: " << mIntrinsic << endl;  
    cout << "distCoeffs is:" << mDistortion_coeff << endl;

    fs.release();	
}

void monocular::reproject_error()
{
	mdtotal_err = 0.0;       
	mdave_error = 0.0;             
 
	vector<Point2f> image_points2; // 保存重新计算得到的投影点
    image_points2.clear();

	for(size_t i = 0; i < mvRvecs.size(); i++)
	{
		vector<Point3f> tempPointSet = mvObjRealPoint[i];
		
		//通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点
		projectPoints(tempPointSet, mvRvecs[i], mvTvecs[i], mIntrinsic, mDistortion_coeff, image_points2);
 
		// 计算新的投影点和旧的投影点之间的误差
		vector<Point2f> tempImagePoint = mvCorners[i];
 
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2); //cornerSubPix
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);   //projectPoints
 
		//对标定结果进行评价
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
            //分别给两个角点坐标赋值他x，y坐标                                                               
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);   
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}

		//norm计算数组src1和src2的相对差范数
		mdtotal_err = mdtotal_err + norm(image_points2Mat, tempImagePointMat, NORM_L2);
		std::cout << "the " << i + 1 << " image Mean error：" << mdtotal_err << " pixel" << endl;
	}
	
    mdave_error = mdtotal_err/mvRvecs.size();
	std::cout << "The all Mean error: " << mdave_error << " pixel" << endl;	
}

void monocular::distort_correct()
{
	Mat mapx = Mat(mImages[0].size(), CV_32FC1);
	Mat mapy = Mat(mImages[0].size(), CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);      

    //计算出对应的映射表
    //第三个参数R，可选的输入，是第一和第二相机坐标之间的旋转矩阵；
	//第四个参数newCameraMatrix，输入的校正后的3X3摄像机矩阵；
	//第五个参数size，摄像机采集的无失真的图像尺寸；
	//第六个参数m1type，定义map1的数据类型，可以是CV_32FC1或者CV_16SC2；
	//第七个参数map1和第八个参数map2，输出的X / Y坐标重映射参数；
	initUndistortRectifyMap(mIntrinsic, mDistortion_coeff, R, mIntrinsic, mImages[0].size(), CV_32FC1, mapx, mapy);
 
	for (int i = 0; i < mImages.size(); i++)
	{
		Mat imageSource = mImages[i];
		Mat newimage = mImages[i].clone();
		Mat newimage1 = mImages[i].clone();
 
		//方法一：使用initUndistortRectifyMap和remap两个函数配合实现。
		//initUndistortRectifyMap用来计算畸变映射，remap把求得的映射应用到图像上。
		remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);
 
		//方法二：使用undistort函数实现
		undistort(imageSource, newimage1, mIntrinsic, mDistortion_coeff);		
		
		//输出图像
		string str = "./result/remap/" + to_string(i + 1) + ".jpg";
		string str1 = "./result/undistort/" + to_string(i + 1) + ".jpg";

		imwrite(str, newimage);
		imwrite(str1, newimage1);
	}
}

void monocular::calibrate()
{
    glob(mFilespath, mFiles, false);

    for(size_t i = 0; i < mFiles.size(); i++) 
        cout << "calibrate image path is: " << mFiles[i] << endl;

    for (size_t i = 0; i < mFiles.size(); i++) 
        mImages.push_back(imread(mFiles[i])); 
	
	if(mImages.size() < 5)
	{
		cout << "the image of calibration is not enough!" << endl;
		return;
	}

    mWidth = mImages[0].cols;
    mHeight = mImages[0].rows;

    for(size_t i = 0; i < mImages.size(); i++)
    {
        cv::Mat SrcImage = mImages[i];
        if(SrcImage.channels() != 1)
            cvtColor(SrcImage, SrcImage, CV_BGR2GRAY);

        bool isFind = findChessboardCorners(SrcImage, mBoardSize, mvCorner, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

        if (isFind == true)
		{
			//精确角点位置，亚像素角点检测
			cornerSubPix(SrcImage, mvCorner, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			//绘制角点
			drawChessboardCorners(SrcImage, mBoardSize, mvCorner, isFind);
			mvCorners.push_back(mvCorner);

            imshow("chessboard_corner", SrcImage);
			waitKey(50);

			cout << "The image " << i << " is good" << endl;
		}
		else
			cout << "The image " << i << " is bad, try again" << endl;
    }

	/*设置实际初始参数 根据calibrateCamera来 如果flag = 0 也可以不进行设置*/
	guessCameraParam();
	cout << "guess successful" << endl;

	/*计算实际的校正点的三维坐标*/
	calRealPoint(mvObjRealPoint, mBoardWidth, mBoardHeight, mImages.size(), mSquareSize);
	cout << "calculate real successful" << endl;

	/*标定摄像头*/

	//第一个参数：objectPoints，为世界坐标系中的三维点：vector<vector<Point3f>> object_points，
	//需要依据棋盘上单个黑白矩阵的大小，计算出（初始化）每一个内角点的世界坐标。
	//长100*宽75
	//第二个参数：imagePoints，为每一个内角点对应的图像坐标点：vector<vector<Point2f>> image_points
	//第三个参数：imageSize，为图像的像素尺寸大小
	//第四个参数：cameraMatrix为相机的内参矩阵：Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0));
	//第五个参数：distCoeffs为畸变矩阵Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0));
   
    //内参数矩阵 M=[fx γ u0,0 fy v0,0 0 1]
    //外参矩阵  5个畸变系数k1,k2,p1,p2,k3
 
	//第六个参数：rvecs旋转向量R，vector<Mat> tvecs;
	//第七个参数：tvecs位移向量T，和rvecs一样，应该为vector<Mat> tvecs;
	//第八个参数：flags为标定时所采用的算法  第九个参数：criteria是最优迭代终止条件设定。
	//return：重投影的总的均方根误差。
 
	//总结：得到相机内参矩阵K、相机的5个畸变系数、每张图片属于自己的平移向量T、旋转向量R
	mdRMS_error = calibrateCamera(mvObjRealPoint, mvCorners, Size(mWidth, mHeight), mIntrinsic, mDistortion_coeff, mvRvecs, mvTvecs, 0);
	cout << "the overall RMS re-projection error is: " << mdRMS_error << endl;
	cout << "calibration successful" << endl;

	/*保存并输出参数*/
	saveCameraParam();
	cout << "save camera param successful" << endl;

	/*畸变校正*/
    distort_correct();
    cout << "distort_correct successful" << endl;

	/*读取相机内部参数，输出到界面*/
	ReadCameraParam();
	cout << "read camera param finished!" << endl;
}