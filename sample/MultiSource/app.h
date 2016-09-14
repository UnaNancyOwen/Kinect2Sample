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
    ComPtr<IMultiSourceFrameReader> multiSourceFrameReader;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;
    cv::Mat colorMat;

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

    // Initialize Multi Source
    inline void initializeMultiSource();

    // Initialize Color
    inline void initializeColor();

    // Initialize Depth
    inline void initializeDepth();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Color
    inline void updateColor( const ComPtr<IMultiSourceFrame>& multiSourceFrame );

    // Update Depth
    inline void updateDepth( const ComPtr<IMultiSourceFrame>& multiSourceFrame );

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw Depth
    inline void drawDepth();

    // Show Data
    void show();

    // Show Color
    inline void showColor();

    // Show Depth
    inline void showDepth();
};

#endif // __APP__