#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>

#include <ppl.h>
#include <atlbase.h>

// Constructor
Kinect::Kinect()
{
    // Initialize
    initialize();
}

// Destructor
Kinect::~Kinect()
{
    // Finalize
    finalize();
}

// Processing
void Kinect::run()
{
    // Main Loop
    while( true ){
        // Update Data
        update();

        // Draw Data
        draw();

        // Show Data
        show();

        // Key Check
        const int key = cv::waitKey( 10 );
        if( key == VK_ESCAPE ){
            break;
        }
        else if( key == 'r' ){
            std::cout << "Reset Reconstruction" << std::endl;
            reset();
        }
        else if( key == 's' ){
            std::cout << "Save Mesh Data to File" << std::endl;
            save();
        }
    }
}

// Initialize
void Kinect::initialize()
{
    cv::setUseOptimized( true );

    // Initialize Sensor
    initializeSensor();

    // Initialize Color
    initializeColor();

    // Initialize Depth
    initializeDepth();

    // Initialize Fusion
    initializeFusion();

    // Wait a Few Seconds until begins to Retrieve Data from Sensor ( about 2000-[ms] )
    std::this_thread::sleep_for( std::chrono::seconds( 2 ) );
}

// Initialize Sensor
inline void Kinect::initializeSensor()
{
    // Open Sensor
    ERROR_CHECK( GetDefaultKinectSensor( &kinect ) );

    ERROR_CHECK( kinect->Open() );

    // Check Open
    BOOLEAN isOpen = FALSE;
    ERROR_CHECK( kinect->get_IsOpen( &isOpen ) );
    if( !isOpen ){
        throw std::runtime_error( "failed IKinectSensor::get_IsOpen( &isOpen )" );
    }

    // Retrieve Coordinate Mapper
    ERROR_CHECK( kinect->get_CoordinateMapper( &coordinateMapper ) );
}

// Initialize Color
inline void Kinect::initializeColor()
{
    // Open Color Reader
    ComPtr<IColorFrameSource> colorFrameSource;
    ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );
    ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

    // Retrieve Color Description
    ComPtr<IFrameDescription> colorFrameDescription;
    ERROR_CHECK( colorFrameSource->CreateFrameDescription( ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription ) );
    ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) ); // 1920
    ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) ); // 1080
    ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) ); // 4

    // Allocation Color Buffer
    colorBuffer.resize( colorWidth * colorHeight * colorBytesPerPixel );
}

// Initialize Depth
inline void Kinect::initializeDepth()
{
    // Open Depth Reader
    ComPtr<IDepthFrameSource> depthFrameSource;
    ERROR_CHECK( kinect->get_DepthFrameSource( &depthFrameSource ) );
    ERROR_CHECK( depthFrameSource->OpenReader( &depthFrameReader ) );

    // Retrieve Depth Description
    ComPtr<IFrameDescription> depthFrameDescription;
    ERROR_CHECK( depthFrameSource->get_FrameDescription( &depthFrameDescription ) );
    ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) ); // 512
    ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) ); // 424
    ERROR_CHECK( depthFrameDescription->get_BytesPerPixel( &depthBytesPerPixel ) ); // 2

    // Allocation Depth Buffer
    depthBuffer.resize( depthWidth * depthHeight );
}

// Initialize Fusion
inline void Kinect::initializeFusion()
{
    // Set Reconstruction Parameters
    reconstructionParameters.voxelsPerMeter = 256;
    reconstructionParameters.voxelCountX = 512;
    reconstructionParameters.voxelCountY = 384;
    reconstructionParameters.voxelCountZ = 512;

    // Create Reconstruction
    SetIdentityMatrix( worldToCameraTransform );
    ERROR_CHECK( NuiFusionCreateColorReconstruction( &reconstructionParameters, NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE::NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP, -1, &worldToCameraTransform, &reconstruction ) );

    // Set Camera Parameters
    cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
    cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
    cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
    cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;

    // Create Image Frame Buffers
    ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_FLOAT, depthWidth, depthHeight, &cameraParameters, &depthImageFrame ) );
    ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_FLOAT, depthWidth, depthHeight, &cameraParameters, &smoothDepthImageFrame ) );
    ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, &cameraParameters, &colorImageFrame ) );
    ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, depthWidth, depthHeight, &cameraParameters, &pointCloudImageFrame ) );
    ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, &cameraParameters, &surfaceImageFrame ) );
    /*ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, &cameraParameters, &normalImageFrame ) );*/
}

// Finalize
void Kinect::finalize()
{
    cv::destroyAllWindows();

    // Release Image Frame Buffers
    ERROR_CHECK( NuiFusionReleaseImageFrame( depthImageFrame ) );
    ERROR_CHECK( NuiFusionReleaseImageFrame( smoothDepthImageFrame ) );
    ERROR_CHECK( NuiFusionReleaseImageFrame( colorImageFrame ) );
    ERROR_CHECK( NuiFusionReleaseImageFrame( pointCloudImageFrame ) );
    ERROR_CHECK( NuiFusionReleaseImageFrame( surfaceImageFrame ) );
    /*ERROR_CHECK( NuiFusionReleaseImageFrame( normalImageFrame ) );*/

    // Close Sensor
    if( kinect != nullptr ){
        kinect->Close();
    }
}

// Update Data
void Kinect::update()
{
    // Update Color
    updateColor();

    // Update Depth
    updateDepth();

    // Update Fusion
    updateFusion();
}

// Update Color
inline void Kinect::updateColor()
{
    // Retrieve Color Frame
    ComPtr<IColorFrame> colorFrame;
    const HRESULT ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Convert Format ( YUY2 -> BGRA )
    ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray( static_cast<UINT>( colorBuffer.size() ), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );
}

// Update Depth
inline void Kinect::updateDepth()
{
    // Retrieve Depth Frame
    ComPtr<IDepthFrame> depthFrame;
    const HRESULT ret = depthFrameReader->AcquireLatestFrame( &depthFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Retrieve Depth Data
    ERROR_CHECK( depthFrame->CopyFrameDataToArray( static_cast<UINT>( depthBuffer.size() ), &depthBuffer[0] ) );
}

// Update Fusion
inline void Kinect::updateFusion()
{
    // Set Depth Data to Depth Float Frame Buffer
    ERROR_CHECK( reconstruction->DepthToDepthFloatFrame( &depthBuffer[0], static_cast<UINT>( depthBuffer.size() * depthBytesPerPixel ), depthImageFrame, NUI_FUSION_DEFAULT_MINIMUM_DEPTH/* 0.5[m] */, NUI_FUSION_DEFAULT_MAXIMUM_DEPTH/* 8.0[m] */, true ) );

    // Smoothing Depth Float Frame
    ERROR_CHECK( reconstruction->SmoothDepthFloatFrame( depthImageFrame, smoothDepthImageFrame, NUI_FUSION_DEFAULT_SMOOTHING_KERNEL_WIDTH, NUI_FUSION_DEFAULT_SMOOTHING_DISTANCE_THRESHOLD ) );

    // Retrieve Mapped Coordinates
    std::vector<ColorSpacePoint> points( depthWidth * depthHeight );
    ERROR_CHECK( coordinateMapper->MapDepthFrameToColorSpace( depthWidth * depthHeight, &depthBuffer[0], depthWidth * depthHeight, &points[0] ) );

    // Mapping Color to Depth Resolution and Set Color Data to Color Frame Buffer
    NUI_FUSION_BUFFER* colorImageFrameBuffer = colorImageFrame->pFrameBuffer;
    RGBQUAD* src = reinterpret_cast<RGBQUAD*>( &colorBuffer[0] );
    RGBQUAD* dst = reinterpret_cast<RGBQUAD*>( colorImageFrameBuffer->pBits );
    Concurrency::parallel_for( 0, depthHeight, [&]( const int y ){
        for( int x = 0; x < depthWidth; x++ ){
            unsigned int index = y * depthWidth + x;
            const ColorSpacePoint point = points[index];
            int colorX = static_cast<int>( point.X + 0.5f );
            int colorY = static_cast<int>( point.Y + 0.5f );
            if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                dst[index] = src[colorY * colorWidth + colorX];
            }
            else{
                dst[index] = {};
            }
        }
    } );

    // Retrieve Transformation Matrix to Camera Coordinate System from World Coordinate System
    ERROR_CHECK( reconstruction->GetCurrentWorldToCameraTransform( &worldToCameraTransform ) );

    // Reconstruction Frame Process 
    HRESULT ret = reconstruction->ProcessFrame( smoothDepthImageFrame, colorImageFrame, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT, NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES, nullptr, &worldToCameraTransform );
    if( FAILED( ret ) ){
        // Reset Reconstruction when Retrived Many Accumulated Error Frames ( Over 100 Error Frames )
        static unsigned int errorCount = 0;
        if( ++errorCount >= 100 ){
            errorCount = 0;
            reset();
        }
    }

    // Calculate Point Cloud
    ERROR_CHECK( reconstruction->CalculatePointCloud( pointCloudImageFrame, surfaceImageFrame, &worldToCameraTransform ) );

    /*
    // Shading Color Transform Matrix
    Matrix4 worldToBGRTransform = { 0.0f };
    worldToBGRTransform.M11 = reconstructionParameters.voxelsPerMeter / reconstructionParameters.voxelCountX;
    worldToBGRTransform.M22 = reconstructionParameters.voxelsPerMeter / reconstructionParameters.voxelCountY;
    worldToBGRTransform.M33 = reconstructionParameters.voxelsPerMeter / reconstructionParameters.voxelCountZ;
    worldToBGRTransform.M41 = 0.5f;
    worldToBGRTransform.M42 = 0.5f;
    worldToBGRTransform.M43 = 0.0f;
    worldToBGRTransform.M44 = 1.0f;

    // Shading Point Cloud
    ERROR_CHECK( NuiFusionShadePointCloud( pointCloudImageFrame, &worldToCameraTransform, &worldToBGRTransform, surfaceImageFrame, normalImageFrame ) );
    */
}

// Reset Reconstruction
inline void Kinect::reset()
{
    // Set Identity Matrix
    SetIdentityMatrix( worldToCameraTransform );

    // Reset Reconstruction
    ERROR_CHECK( reconstruction->ResetReconstruction( &worldToCameraTransform, nullptr ) );
}

// Draw Data
void Kinect::draw()
{
    // Draw Fusion
    drawFusion();
}

// Draw Fusion
inline void Kinect::drawFusion()
{
    // Retrive Surface Image from Surface Frame Buffer
    NUI_FUSION_BUFFER* surfaceImageFrameBuffer = surfaceImageFrame->pFrameBuffer;
    surfaceMat = cv::Mat( depthHeight, depthWidth, CV_8UC4, surfaceImageFrameBuffer->pBits );

    /*
    // Retrive Normal Image from Normal Frame Buffer
    NUI_FUSION_BUFFER* normalImageFrameBuffer = normalImageFrame->pFrameBuffer;
    normalMat = cv::Mat( depthHeight, depthWidth, CV_8UC4, normalImageFrameBuffer->pBits );
    */
}

// Show Data
void Kinect::show()
{
    // Show Fusion
    showFusion();
}

// Show Fusion
inline void Kinect::showFusion()
{
    if( surfaceMat.empty() ){
        return;
    }

    // Show Surface Image
    cv::imshow( "Surface", surfaceMat );

    /*
    if( normalMat.empty() ){
        return;
    }

    // Show Normal Image
    cv::imshow( "Normal", normalMat );
    */
}

// Save Mesh
inline void Kinect::save()
{
    // Calculate Mesh Data
    ComPtr<INuiFusionColorMesh> mesh;
    ERROR_CHECK( reconstruction->CalculateMesh( 1, &mesh ) );

    // Save Mesh Data to PLY File
    wchar_t* fileName = L"../mesh.ply";
    WriteAsciiPlyMeshFile( mesh.Get(), W2OLE( fileName ), true, true );

    /*
    // Save Mesh Data to STL File
    wchar_t* fileName = L"../mesh.stl";
    ERROR_CHECK( WriteBinarySTLMeshFile( mesh, W2OLE( fileName ), true ) );
    */

    /*
    // Save Mesh Data to Obj File
    wchar_t* fileName = L"../mesh.obj";
    ERROR_CHECK( WriteAsciiObjMeshFile( mesh, W2OLE( fileName ), true ) );
    */
}