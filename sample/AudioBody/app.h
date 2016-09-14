#ifndef __APP__
#define __APP__

#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <array>

#include <wrl/client.h>
using namespace Microsoft::WRL;

class Kinect
{
private:
    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Reader
    ComPtr<IBodyFrameReader> bodyFrameReader;
    ComPtr<IBodyIndexFrameReader> bodyIndexFrameReader;
    ComPtr<IAudioBeamFrameReader> audioBeamFrameReader;

    // Body Buffer
    std::array<IBody*, BODY_COUNT> bodies = { nullptr };

    // BodyIndex Buffer
    std::vector<BYTE> bodyIndexBuffer;
    int bodyIndexWidth;
    int bodyIndexHeight;
    cv::Mat bodyIndexMat;
    std::array<cv::Vec3b, BODY_COUNT> colors;

    // Audio Buffer
    UINT64 audioTrackingId;
    int audioTrackingIndex;

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

    // Initialize Body
    inline void initializeBody();

    // Initialize BodyIndex
    inline void initializeBodyIndex();

    // Initialize Audio
    inline void initializeAudio();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Body
    inline void updateBody();

    // Update BodyIndex
    inline void updateBodyIndex();

    // Update Audio
    inline void updateAudio();

    // Draw Data
    void draw();

    // Draw BodyIndex
    inline void drawBodyIndex();

    // Draw Audio
    inline void drawAudio();

    // Show Data
    void show();

    // Show BodyIndex
    inline void showBodyIndex();
};

#endif // __APP__