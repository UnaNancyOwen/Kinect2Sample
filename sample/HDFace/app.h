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
    ComPtr<IHighDefinitionFaceFrameReader> hdFaceFrameReader;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;
    cv::Mat colorMat;

    // HDFace Buffer
    ComPtr<IFaceModelBuilder> faceModelBuilder;
    ComPtr<IFaceAlignment> faceAlignment;
    ComPtr<IFaceModel> faceModel;
    std::array<float, FaceShapeDeformations::FaceShapeDeformations_Count> faceShapeUnits = { 0.0f };
    UINT32 vertexCount;
    UINT64 trackingId;
    int trackingCount = 0;
    bool produced = false;

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

    // Initialize HDFace
    inline void initializeHDFace();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Color
    inline void updateColor();

    // Update Body
    inline void updateBody();

    // Find Closest Body
    inline void findClosestBody( const std::array<ComPtr<IBody>, BODY_COUNT>& bodies );

    // Update HDFace
    inline void updateHDFace();

    // Draw Data
    void draw();

    // Draw Color
    inline void drawColor();

    // Draw HDFace
    inline void drawHDFace();

    // Draw Face Model Builder Status
    inline void drawFaceModelBuilderStatus( cv::Mat& image, const cv::Point& point, const double scale, const cv::Vec3b& color, const int thickness = 2 );

    // Convert Collection Status to String
    inline std::string status2string( const FaceModelBuilderCollectionStatus collection );

    // Convert Capture Status to String
    inline std::string status2string( const FaceModelBuilderCaptureStatus capture );

    // Draw Vertexes
    inline void Kinect::drawVertexes( cv::Mat& image, const std::vector<CameraSpacePoint> vertexes, const int radius, const cv::Vec3b& color, const int thickness = -1 );

    // Show Data
    void show();

    // Show HDFace
    inline void showHDFace();
};

#endif // __APP__