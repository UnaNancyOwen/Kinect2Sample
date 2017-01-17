#ifndef __APP__
#define __APP__

#include <Windows.h>
#include <Kinect.h>
#include <Kinect.Face.h>
#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>

#include <vector>
#include <array>
#include <string>

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

    // Face Recognition
    cv::Ptr<cv::face::FaceRecognizer> recognizer;
    const std::string model = "../model.xml"; // Pre-Trained Model File Path ( *.xml or *.yaml )
    const double threshold = 40.0; // Max Matching Distance
    std::array<int, BODY_COUNT> labels;
    std::array<double, BODY_COUNT> distances;

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

    // Initialize Recognition
    inline void initializeRecognition();

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

    // Update Recognition
    inline void updateRecognition();

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw Recognition
    inline void drawRecognition();

    // Draw Face Bounding Box
    inline void drawFaceBoundingBox( cv::Mat& image, const RectI& box, const cv::Vec3b& color, const int thickness = 1 );

    // Draw Recognition Results
    inline void drawRecognitionResults( cv::Mat& image, const int label, const double distance, const cv::Point& point, const double scale, const cv::Vec3b& color, const int thickness = 2 );

    // Show Data
    void show();

    // Show Recognition
    inline void showRecognition();
};

#endif // __APP__