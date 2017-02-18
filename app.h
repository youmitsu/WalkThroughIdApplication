#define __APP__

#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>

#include <tchar.h>

#include <vector>
#include <array>

#include <wrl/client.h>

#include <tchar.h>
#include <assert.h>
#include <string>
#include <string.h>
#include <sstream>
#include <locale.h>
#include <conio.h>
#include <random>

using namespace Microsoft::WRL;

typedef std::basic_string<TCHAR>		tstring;
typedef std::basic_stringstream<TCHAR>	tstringstream;

class Kinect
{
private:

    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Coordinate Mapper
    ComPtr<ICoordinateMapper> coordinateMapper;

    // Reader
    ComPtr<IColorFrameReader> colorFrameReader;
    ComPtr<IBodyFrameReader> bodyFrameReader;
	ComPtr<IDepthFrameReader> depthFrameReader;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;
    cv::Mat colorMat;

    // Body Buffer
    std::array<IBody*, BODY_COUNT> bodies;
    std::array<cv::Vec3b, BODY_COUNT> colors;

	//Depth Buffer
	std::vector<UINT16> depthBuffer;
	int depthWidth;
	int depthHeight;
	unsigned int depthBytesPerPixel;
	cv::Mat depthMat;

public:
    // Constructor
    Kinect();

    // Destructor
    ~Kinect();

	// Update Data
	void update(array<Joint, JointType::JointType_Count>& joints);

	// Draw Data
	void draw();

	// Show Data
	void show();

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    inline void initializeSensor();

    // Initialize Color
    inline void initializeColor();

    // Initialize Body
    inline void initializeBody();

	//initialize Depth
	inline void initializeDepth();

    // Finalize
    void finalize();

    // Update Color
    inline void updateColor();

    // Update Body
	inline void updateBody(array<Joint, JointType::JointType_Count>& joints);

	//Update Depth
	inline void updateDepth();

    // Draw Color
    inline void drawColor();

    // Draw Body
    inline void drawBody();

	//Draw Depth
	inline void drawDepth();

    // Draw Circle
    inline void drawEllipse( cv::Mat& image, const Joint& joint, const int radius, const cv::Vec3b& color, const int thickness = -1 );

    // Draw Hand State
    inline void drawHandState( cv::Mat& image, const Joint& joint, HandState handState, TrackingConfidence handConfidence );

    // Show Body
    inline void showBody();

	inline void showDepth();
};
