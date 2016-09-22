#ifndef __APP__
#define __APP__

#include <Windows.h>
#include <Kinect.h>
#include <Kinect.Face.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <array>

#include <wrl/client.h>
using namespace Microsoft::WRL;

#include <array>

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
    std::array<ComPtr<IFaceFrameReader>, BODY_COUNT> faceFrameReader;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;
    cv::Mat colorMat;

    // Body Buffer
    std::array<IBody*, BODY_COUNT> bodies = { nullptr };

    // Face Buffer
    std::array<ComPtr<IFaceFrameResult>, BODY_COUNT> results;
    std::array<cv::Mat, BODY_COUNT> faceClipMat;

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

    // Initialize Body
    inline void initializeBody();

    // Initialize Face
    inline void initializeFace();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Color
    inline void updateColor();

    // Update Body
    inline void updateBody();

    // Update Face
    inline void updateFace();

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw Face Clip
    inline void drawFaceClip();

    // Retrieve Face Clip
    inline void retrieveFaceClip( cv::Mat& image, const RectI& box );

    // Show Data
    void show();

    // Show Face Clip
    inline void showFaceClip();
};

#endif // __APP__