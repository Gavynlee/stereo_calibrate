#ifndef STEREO_H
#define STEREO_H

#include <iostream>
#include <string>

#include "monocular.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class stereo
{
public:
    stereo(int boardWidth, int boardHeight, int squareSize, string Lfilespath, string Rfilespath);
    ~stereo();

    //立体标定
    void stereo_calibrate();

private:
    //输出标定参数
    void outputCameraParam();

    //绘制校正图像
    void drawRectifyImage();

private:
    int mBoardWidth;            //横向的角点数目
    int mBoardHeight;           //纵向的角点数据
    int mSquareSize;            //标定板黑白格子的大小 单位mm
    int mBoardCorner;           //总的角点数据

    int mWidth, mHeight;         //输入图像的大小
    cv::Size mBoardSize;        //标定板的尺寸

    cv::Mat R, T, E, F;                 //R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵
    cv::Mat Rl, Rr, Pl, Pr, Q;          //校正旋转矩阵R，投影矩阵P 重投影矩阵Q
    cv::Mat mapLx, mapLy, mapRx, mapRy; //映射表

    cv::Mat rectifyImageL;              //校正后的图像
    cv::Mat rectifyImageR;

    cv::Mat mCameraMatrixL, mCameraMatrixR;  //内部参数
    cv::Mat mDistCoeffL, mDistCoeffR;        //畸变

    cv::Rect validROIL, validROIR;           //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域

    vector<Point2f> mvCornerL;               //左边摄像机某一照片角点坐标集合
    vector<Point2f> mvCornerR;               //右边摄像机某一照片角点坐标集合

    vector<vector<Point2f>> mvImagePointL;   //左边摄像机所有照片角点的坐标集合
    vector<vector<Point2f>> mvImagePointR;   //右边摄像机所有照片角点的坐标集合

    string mLfilespath, mRfilespath;         //左右标定图像的路径
    double mdStereo_RMS;                     //返回总的均方根重投影误差
};

#endif