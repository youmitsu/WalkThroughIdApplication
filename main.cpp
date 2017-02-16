// WalkThroughIdApplication.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//
#include <Windows.h>
#include <Kinect.h>

#include "app.h"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int _tmain(int argc, _TCHAR* argv[])
{
	try{
		Kinect kinect;
		kinect.run();
	}
	catch (std::exception& ex){
		std::cout << ex.what() << std::endl;
	}

	return 0;
}

