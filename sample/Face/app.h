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
    std::array<std::string, FaceProperty::FaceProperty_Count> labels;
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

    // Draw Face
    inline void drawFace();

    // Draw Face Points
    inline void drawFacePoints( cv::Mat& image, const std::array<PointF, FacePointType::FacePointType_Count>& points, const int radius, const cv::Vec3b& color, const int thickness = -1 );

    // Draw Face Bounding Box
    inline void drawFaceBoundingBox( cv::Mat& image, const RectI& box, const cv::Vec3b& color, const int thickness = 1 );

    // Draw Face Rotation
    inline void drawFaceRotation( cv::Mat& image, Vector4& quaternion, const RectI& box, const double fontScale, const cv::Vec3b& color, const int thickness = 2 );

    // Convert Quaternion to Degree
    inline void quaternion2degree( const Vector4* quaternion, int* pitch, int* yaw, int* roll );

    // Draw Face Properties
    inline void drawFaceProperties( cv::Mat& image, std::array<DetectionResult, FaceProperty::FaceProperty_Count>& detections, const RectI& box, const double fontScale, const cv::Vec3b& color, const int thickness = 2 );

    // Convert Detection Result to String
    inline std::string Kinect::result2string( DetectionResult& result );

    // Show Data
    void show();

    // Show Face
    inline void showFace();
};

#endif // __APP__