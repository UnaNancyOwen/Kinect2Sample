#ifndef __APP__
#define __APP__

#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>

#include <vector>

#include <wrl/client.h>
using namespace Microsoft::WRL;

#include <array>

class Kinect
{
private:
    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Reader
    ComPtr<IBodyIndexFrameReader> bodyIndexFrameReader;

    // BodyIndex Buffer
    std::vector<BYTE> bodyIndexBuffer;
    int bodyIndexWidth;
    int bodyIndexHeight;
    unsigned int bodyIndexBytesPerPixel;
    cv::Mat bodyIndexMat;
    std::array<cv::Vec3b, BODY_COUNT> colors;

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

    // Initialize BodyIndex
    inline void initializeBodyIndex();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update BodyIndex
    inline void updateBodyIndex();

    // Draw Data
    void draw();

    // Draw BodyIndex
    inline void drawBodyIndex();

    // Show Data
    void show();

    // Show BodyIndex
    inline void showBodyIndex();
};

#endif // __APP__