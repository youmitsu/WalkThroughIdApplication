// WalkThroughIdApplication.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//
#include <Windows.h>
#include <Kinect.h>

#include "app.h"
#include "walk_through_id.h"

#include <array>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int _tmain(int argc, _TCHAR* argv[])
{
	try{
		WalkThroughId identifier; //�E�H�[�N�X���[�F�ؗp
		Kinect kinect;
		while (true){
			array<Joint, JointType::JointType_Count> joints;
			// Update Data
			kinect.update(joints);

			// Draw Data
			kinect.draw();

			// Show Data
			kinect.show();

			// Key Check
			const int key = cv::waitKey(10);
			if (key == VK_ESCAPE){
				break;
			}
		}
	}
	catch (std::exception& ex){
		std::cout << ex.what() << std::endl;
	}

	return 0;
}

