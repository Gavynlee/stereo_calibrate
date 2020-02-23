#include "stereo.h"

stereo::stereo(int boardWidth, int boardHeight, int squareSize, string Lfilespath, string Rfilespath):
    mBoardWidth(boardWidth), mBoardHeight(boardHeight), mSquareSize(squareSize), 
    mLfilespath(Lfilespath), mRfilespath(Rfilespath)
{
    mBoardCorner = boardWidth * boardHeight;
    mBoardSize = cv::Size(boardWidth, boardHeight);
}

stereo::~stereo()
{
}

void stereo::outputCameraParam()
{
	FileStorage fs("./result/stereo_calibrate.yaml", FileStorage::WRITE);

	if(!fs.isOpened())
	{
		cout << "open file error!" << endl;
		return;
	}

	fs << "cameraMatrixL" << mCameraMatrixL << "cameraDistcoeffL" << mDistCoeffL;
    fs << "cameraMatrixR" << mCameraMatrixR << "cameraDistcoeffR" << mDistCoeffR;
    
	fs << "R" << R << "T" << T;
    fs << "Rl" << Rl << "Rr" << Rr << "Pl" << Pl << "Pr" << Pr;
    fs << "Q" << Q;

    cout << "R=" << R << endl << "T=" << T << endl << "Rl=" << Rl << endl << "Rr=" << Rr << endl << "Pl=" << Pl << endl << "Pr=" << Pr << endl << "Q=" << Q << endl;
	cout << "cameraMatrixL=:" << mCameraMatrixL << endl << "cameraDistcoeffL=:" << mDistCoeffL << endl << "cameraMatrixR=:" << mCameraMatrixR << endl << "cameraDistcoeffR=:" << mDistCoeffR << endl;

	fs.release();
}

void stereo::drawRectifyImage()
{
	/*
	把校正结果显示出来
	把左右两幅图像显示到同一个画面上
	这里只显示了最后一副图像的校正结果。并没有把所有的图像都显示出来
	*/
	double sf;
	int w, h;

	sf = 600. / MAX(mWidth, mHeight);
	w = cvRound(mWidth * sf);
	h = cvRound(mHeight * sf);

    cv::Mat canvas;
	canvas.create(h, w * 2, CV_8UC3);

	/*左图像画到画布上*/
	Mat canvasPart = canvas(Rect(0, 0, w, h));                             //得到画布的一部分
	resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);//把图像缩放到跟canvasPart一样大小
	Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),           
		cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));        //获得被截取的区域
	rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                 //画上一个矩形

	cout << "Painted ImageL" << endl;

	/*右图像画到画布上*/
	canvasPart = canvas(Rect(w, 0, w, h));
	resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
		cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
	rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

	cout << "Painted ImageR" << endl;

	/*画上对应的线条*/
	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);

	cv::imshow("rectified", canvas);
    cv::imwrite("./result/rectified.jpg", canvas);
}

void stereo::stereo_calibrate()
{
    monocular Left_Cali(mBoardWidth, mBoardHeight, mSquareSize, mLfilespath);
    monocular Right_Cali(mBoardWidth, mBoardHeight, mSquareSize, mRfilespath);

    Left_Cali.calibrate();
    Right_Cali.calibrate();

    mCameraMatrixL = Left_Cali.mIntrinsic;
    mCameraMatrixR = Right_Cali.mIntrinsic;

    mDistCoeffL = Left_Cali.mDistortion_coeff;
    mDistCoeffR = Right_Cali.mDistortion_coeff;

    mWidth = Left_Cali.mWidth;
    mHeight = Right_Cali.mHeight;

    size_t ImageNumber = Left_Cali.mImages.size();
    for(size_t i = 0; i < ImageNumber; i++)
    {
        cv::Mat LeftImage = Left_Cali.mImages[i];
        cv::Mat RightImage = Right_Cali.mImages[i];

        if(LeftImage.channels() != 1)
            cvtColor(LeftImage, LeftImage, CV_BGR2GRAY);
        if(RightImage.channels() != 1)
            cvtColor(RightImage, RightImage, CV_BGR2GRAY);

		bool isFindL, isFindR;
		isFindL = findChessboardCorners(LeftImage, mBoardSize, mvCornerL);
		isFindR = findChessboardCorners(RightImage, mBoardSize, mvCornerR);

		if (isFindL == true && isFindR == true)
		{
            //对角点进行亚像素提取，提高角点精度
			cornerSubPix(LeftImage, mvCornerL, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			drawChessboardCorners(LeftImage, mBoardSize, mvCornerL, isFindL);
			imshow("chessboardL", LeftImage);
			mvImagePointL.push_back(mvCornerL);

			cornerSubPix(RightImage, mvCornerR, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			drawChessboardCorners(RightImage, mBoardSize, mvCornerR, isFindR);
			imshow("chessboardR", RightImage);
			mvImagePointR.push_back(mvCornerR);

			cout << "The image " << i << " is good" << endl;
		}
		else
			cout << "The image is bad please try again" << endl;
    }

	/*
	标定摄像头
	由于左右摄像机分别都经过了单目标定
	所以在此处选择flag = CALIB_USE_INTRINSIC_GUESS
	*/
    mdStereo_RMS = stereoCalibrate(Left_Cali.mvObjRealPoint, mvImagePointL, mvImagePointR, mCameraMatrixL, mDistCoeffL,
		mCameraMatrixR, mDistCoeffR, Size(mWidth, mHeight), R, T, E, F, 
        CALIB_USE_INTRINSIC_GUESS, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

    std::cout << "Stereo Calibration done with RMS error = " << mdStereo_RMS << endl;
	std::cout << "stereo calibration successful" << std::endl;

	/*
	立体校正的时候需要两幅图像共面并且行对准 以使得立体匹配更加的可靠
	使得两幅图像共面的方法就是把两个摄像头的图像投影到一个公共成像面上，这样每幅图像从本图像平面投影到公共图像平面都需要一个旋转矩阵R
	stereoRectify 这个函数计算的就是从图像平面投影到公共成像平面的旋转矩阵Rl,Rr。 Rl,Rr即为左右相机平面行对准的校正旋转矩阵。
	左相机经过Rl旋转，右相机经过Rr旋转之后，两幅图像就已经共面并且行对准了。
	其中Pl,Pr为两个相机的投影矩阵，其作用是将3D点的坐标转换到图像的2D点的坐标:P*[X Y Z 1]' =[x y w]
	Q矩阵为重投影矩阵，即矩阵Q可以把2维平面(图像平面)上的点投影到3维空间的点:Q*[x y d 1] = [X Y Z W]。其中d为左右两幅图像的时差
	*/
	//对标定过的图像进行校正
	cv::stereoRectify(mCameraMatrixL, mDistCoeffL, mCameraMatrixR, mDistCoeffR, Size(mWidth, mHeight), 
        R, T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY, -1, Size(mWidth, mHeight), &validROIL, &validROIR);
	
	std::cout << "stereo rectify successful" << std::endl;

	/*
	根据stereoRectify 计算出来的R 和 P 来计算图像的映射表 mapx,mapy
	mapx,mapy这两个映射表接下来可以给remap()函数调用，来校正图像，使得两幅图像共面并且行对准
	ininUndistortRectifyMap()的参数newCameraMatrix就是校正后的摄像机矩阵。在openCV里面，校正后的计算机矩阵Mrect是跟投影矩阵P一起返回的。
	所以我们在这里传入投影矩阵P，此函数可以从投影矩阵P中读出校正后的摄像机矩阵
	*/
	//摄像机校正映射
	cv::initUndistortRectifyMap(mCameraMatrixL, mDistCoeffL, Rl, Pl, Size(mWidth, mHeight), CV_32FC1, mapLx, mapLy);
	cv::initUndistortRectifyMap(mCameraMatrixR, mDistCoeffR, Rr, Pr, Size(mWidth, mHeight), CV_32FC1, mapRx, mapRy);

    cv::Mat LSrcImage = Left_Cali.mImages[0];
    cv::Mat RSrcImage = Right_Cali.mImages[0];
    
	cv::remap(LSrcImage, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
	cv::remap(RSrcImage, rectifyImageR, mapRx, mapRy, INTER_LINEAR);

	cv::imwrite("./result/rectifyImageL.jpg", rectifyImageL);
	cv::imwrite("./result/rectifyImageR.jpg", rectifyImageR);

    //输出标定的参数，并保存
    outputCameraParam();

    //显示校正后的图像
    drawRectifyImage();
}