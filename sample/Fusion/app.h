#ifndef __APP__
#define __APP__

#include <Windows.h>
#include <Kinect.h>
#include <NuiKinectFusionApi.h>
// Quote from Kinect for Windows SDK v2.0 - Samples/Native/KinectFusionExplorer-D2D, and Partial Modification
// KinectFusionHelper is: Copyright (c) Microsoft Corporation. All rights reserved.
#include "KinectFusionHelper.h"
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

    // Fusion
    ComPtr<INuiFusionColorReconstruction> reconstruction;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;

    // Depth Buffer
    std::vector<UINT16> depthBuffer;
    int depthWidth;
    int depthHeight;
    unsigned int depthBytesPerPixel;

    // Fusion Buffer
    NUI_FUSION_IMAGE_FRAME* depthImageFrame;
    NUI_FUSION_IMAGE_FRAME* smoothDepthImageFrame;
    NUI_FUSION_IMAGE_FRAME* colorImageFrame;
    NUI_FUSION_IMAGE_FRAME* pointCloudImageFrame;
    NUI_FUSION_IMAGE_FRAME* surfaceImageFrame;
    /*NUI_FUSION_IMAGE_FRAME* normalImageFrame;*/
    NUI_FUSION_RECONSTRUCTION_PARAMETERS reconstructionParameters;
    NUI_FUSION_CAMERA_PARAMETERS cameraParameters;
    Matrix4 worldToCameraTransform;
    cv::Mat surfaceMat;
    /*cv::Mat normalMat;*/

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

    // Initialize Fusion
    inline void initializeFusion();

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Color
    inline void updateColor();

    // Update Depth
    inline void updateDepth();

    // Update Fusion
    inline void updateFusion();

    // Reset Reconstruction
    inline void reset();

    // Draw Data
    void draw();

    // Draw Fusion
    inline void drawFusion();

    // Show Data
    void show();

    // Show Fusion
    inline void showFusion();

    // Save Mesh
    inline void save();
};

#endif // __APP__