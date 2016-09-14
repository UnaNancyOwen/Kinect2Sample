#ifndef __APP__
#define __APP__

#include <Windows.h>
#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>
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
    std::array<ComPtr<IVisualGestureBuilderFrameReader>, BODY_COUNT> gestureFrameReader;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;
    cv::Mat colorMat;

    // Gesture Buffer
    std::vector<ComPtr<IGesture>> gestures;
    std::array<std::vector<std::string>, BODY_COUNT> results;

    std::array<cv::Vec3b, BODY_COUNT> colors;
    int offset;

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

    // Initialize Gesture
    inline void initializeGesture();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Color
    inline void updateColor();

    // Update Body
    inline void updateBody();

    // Update Gesture
    inline void updateGesture();

    // Retrieve Discrete Gesture Result
    inline std::string retrieveDiscreteGestureResult( const ComPtr<IVisualGestureBuilderFrame>& gestureFrame, const ComPtr<IGesture>& gesture );

    // Retrieve Continuous Gesture Result
    inline std::string retrieveContinuousGestureResult( const ComPtr<IVisualGestureBuilderFrame>& gestureFrame, const ComPtr<IGesture>& gesture );

    // Retrive Gesture Name
    inline std::string gesture2string( const ComPtr<IGesture>& gesture );

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw Gesture
    inline void drawGesture();

    // Draw Results
    inline void drawResult( cv::Mat& image, const std::vector<std::string>& results, const cv::Point& point, const double scale, const cv::Vec3b& color, const int thickness = 2 );

    // Show Data
    void show();

    // Show Gesture
    inline void showGesture();
};

#endif // __APP__