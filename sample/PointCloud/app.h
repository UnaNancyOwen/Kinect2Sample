#ifndef __APP__
#define __APP__

#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <vector>

#include <wrl/client.h>
using namespace Microsoft::WRL;

class Kinect
{
private:
    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Coordinate Mapper
    ComPtr<ICoordinateMapper> coordinateMapper;

    // Reader
    ComPtr<IColorFrameReader> colorFrameReader;
    ComPtr<IDepthFrameReader> depthFrameReader;

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

    // Point Cloud Buffer
    cv::viz::Viz3d viewer;
    cv::Mat cloudMat;

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

    // Initialize Color
    inline void initializeColor();

    // Initialize Depth
    inline void initializeDepth();

    // Initialize Point Cloud
    inline void initializePointCloud();

    // Keyboard Callback Function
    static void keyboardCallback( const cv::viz::KeyboardEvent& event, void* cookie );

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Color
    inline void updateColor();

    // Update Depth
    inline void updateDepth();

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw Point Cloud
    inline void drawPointCloud();

    // Show Data
    void show();

    // Show Point Cloud
    inline void showPointCloud();
};

#endif // __APP__