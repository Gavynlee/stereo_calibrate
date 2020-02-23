#include "stereo.h"

int main ( int argc, char** argv )
{
	std::string Lfilespath = "./data/left/";
	std::string Rfilespath = "./data/right/";

    //标定板的大小和小方格的尺寸
    stereo Stereo_cali(6, 9, 25, Lfilespath, Rfilespath);
    Stereo_cali.stereo_calibrate();

    return 0;
}