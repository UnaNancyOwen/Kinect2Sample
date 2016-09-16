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

    // Coordinate Mapper
    ComPtr<ICoordinateMapper> coordinateMapper;

    // Reader
    ComPtr<IColorFrameReader> colorFrameReader;
    ComPtr<IDepthFrameReader> depthFrameReader;
    ComPtr<IBodyIndexFrameReader> bodyIndexFrameReader;

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

    // BodyIndex Buffer
    std::vector<BYTE> bodyIndexBuffer;
    int bodyIndexWidth;
    int bodyIndexHeight;
    unsigned int bodyIndexBytesPerPixel;
    cv::Mat bodyIndexMat;

    // ChromaKey Buffer
    cv::Mat chromaKeyMat;

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

    // Initialize BodyIndex
    inline void initializeBodyIndex();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Color
    inline void updateColor();

    // Update Depth
    inline void updateDepth();

    // Update BodyIndex
    inline void updateBodyIndex();

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw BodyIndex
    inline void drawBodyIndex();

    // Draw ChromaKey
    inline void drawChromaKey();

    // Show Data
    void show();

    // Show ChromaKey
    inline void showChromaKey();
};

#endif // __APP__