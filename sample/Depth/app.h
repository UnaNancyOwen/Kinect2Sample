#ifndef __APP__
#define __APP__

#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>

#include <vector>

#include <wrl/client.h>
using namespace Microsoft::WRL;

class Kinect
{
private:
    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Reader
    ComPtr<IDepthFrameReader> depthFrameReader;

    // Depth Buffer
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

    // Processing
    void run();

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    inline void initializeSensor();

    // Initialize Depth
    inline void initializeDepth();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Depth
    inline void updateDepth();

    // Draw Data
    void draw();

    // Draw Depth
    inline void drawDepth();

    // Show Data
    void show();

    // Show Depth
    inline void showDepth();
};

#endif // __APP__