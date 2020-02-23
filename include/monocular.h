#ifndef MONOCULAR_H
#define MONOCULAR_H

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class monocular
{
public:
    /*构造函数*/
    monocular(int boardWidth, int boardHeight, int squareSize, string filespath);

    /*实现相机标定*/
    void calibrate();

    /*析构函数*/
    ~monocular();

private:
    /*计算标定板上模块的实际物理坐标*/
    void calRealPoint(vector<vector<Point3f> >& obj, int boardwidth, int boardheight, int imgNumber, int squaresize);

    /*设置相机的初始参数 也可以不估计*/
    void guessCameraParam();

    /*输出标定参数*/
    void saveCameraParam();

    /*计算重投影误差*/
    void reproject_error();

    /*原始图像的畸变矫正*/
    void distort_correct();

    /*读取相机内部参数，输出到界面*/
    void ReadCameraParam();

public:
    Mat mIntrinsic;                               //相机内参数
    Mat mDistortion_coeff;                        //相机畸变参数
    vector<Mat> mvRvecs;                          //旋转向量
    vector<Mat> mvTvecs;                          //平移向量
    vector<vector<Point2f> > mvCorners;           //各个图像找到的角点的集合 和objRealPoint 一一对应
    vector<vector<Point3f> > mvObjRealPoint;      //各副图像的角点的实际物理坐标集合
    vector<Point2f> mvCorner;                     //某一副图像找到的角点

    vector<String> mFiles;//所有标定图像的路径
    vector<Mat> mImages;  //所有标定图像

    int mWidth, mHeight;  //标定图像的大小

    int mBoardWidth;      //横向的角点数目  
    int mBoardHeight;     //纵向的角点数据  
    int mBoardCorner;     //总的角点数据   
    int mSquareSize;      //标定板黑白格子的大小 单位mm  
    Size mBoardSize;      //总的内角点

    double mdRMS_error;   //返回总的均方根重投影误差

    double mdtotal_err;   //返回总的重投影误差
    double mdave_error;   //返回平均的重投影误差

    string mFilespath;    //输入标定图像所在的文件夹路径
};
#endif