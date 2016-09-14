#ifndef __APP__
#define __APP__

#define _USE_MATH_DEFINES
#include <Windows.h>
#include <Kinect.h>

#include <string>

#include <wrl/client.h>
using namespace Microsoft::WRL;

class Kinect
{
private:
    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Reader
    ComPtr<IAudioBeamFrameReader> audioBeamFrameReader;

    // Audio Buffer
    float beamAngle = 0.f;
    float beamAngleConfidence = 0.f;
    std::string beamAngleResult;
    const float confidenceThreshold = 0.3f;

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

    // Initialize Audio
    inline void initializeAudio();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Audio
    inline void updateAudio();

    // Draw Data
    void draw();

    // Draw Audio
    inline void drawAudio();

    // Show Data
    void show();

    // Show Audio
    inline void showAudio();
};

#endif // __APP__