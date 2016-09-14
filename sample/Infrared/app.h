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
    ComPtr<IInfraredFrameReader> infraredFrameReader;

    // Infrared Buffer
    std::vector<UINT16> infraredBuffer;
    int infraredWidth;
    int infraredHeight;
    unsigned int infraredBytesPerPixel;
    cv::Mat infraredMat;

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

    // Initialize Infrared
    inline void initializeInfrared();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Infrared
    inline void updateInfrared();

    // Draw Data
    void draw();

    // Draw Infrared
    inline void drawInfrared();

    // Show Data
    void show();

    // Show Infrared
    inline void showInfrared();
};

#endif // __APP__