#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>

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
    }
}

// Initialize
void Kinect::initialize()
{
    cv::setUseOptimized( true );

    // Initialize Sensor
    initializeSensor();

    // Initialize Multi Source
    initializeMultiSource();

    // Initialize Color
    initializeColor();

    // Initialize Depth
    initializeDepth();

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
}

// Initialize Multi Source
inline void Kinect::initializeMultiSource()
{
    // Open Multi Source Reader
    DWORD types = FrameSourceTypes::FrameSourceTypes_Color
                | FrameSourceTypes::FrameSourceTypes_Depth;

    ERROR_CHECK( kinect->OpenMultiSourceFrameReader( types, &multiSourceFrameReader ) );
}

// Initialize Color
inline void Kinect::initializeColor()
{
    // Open Color Reader
    ComPtr<IColorFrameSource> colorFrameSource;
    ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );

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

    // Retrieve Depth Description
    ComPtr<IFrameDescription> depthFrameDescription;
    ERROR_CHECK( depthFrameSource->get_FrameDescription( &depthFrameDescription ) );
    ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) ); // 512
    ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) ); // 424
    ERROR_CHECK( depthFrameDescription->get_BytesPerPixel( &depthBytesPerPixel ) ); // 2

    // Allocation Depth Buffer
    depthBuffer.resize( depthWidth * depthHeight );
}

// Finalize
void Kinect::finalize()
{
    cv::destroyAllWindows();

    // Close Sensor
    if( kinect != nullptr ){
        kinect->Close();
    }
}

// Update Data
void Kinect::update()
{
    // Retrieve Multi Source Frame
    ComPtr<IMultiSourceFrame> multiSourceFrame;
    const HRESULT ret = multiSourceFrameReader->AcquireLatestFrame( &multiSourceFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Update Color
    updateColor( multiSourceFrame );

    // Update Depth
    updateDepth( multiSourceFrame );
}

// Update Color
inline void Kinect::updateColor( const ComPtr<IMultiSourceFrame>& multiSourceFrame )
{
    if( multiSourceFrame == nullptr ){
        return;
    }

    // Retrieve Color Frame Reference
    ComPtr<IColorFrameReference> colorFrameReference;
    HRESULT ret = multiSourceFrame->get_ColorFrameReference( &colorFrameReference );
    if( FAILED( ret ) ){
        return;
    }

    // Retrieve Color Frame
    ComPtr<IColorFrame> colorFrame;
    ret = colorFrameReference->AcquireFrame( &colorFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Convert Format ( YUY2 -> BGRA )
    ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray( static_cast<UINT>( colorBuffer.size() ), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );
}

// Update Depth
inline void Kinect::updateDepth( const ComPtr<IMultiSourceFrame>& multiSourceFrame )
{
    if( multiSourceFrame == nullptr ){
        return;
    }

    // Retrieve Depth Frame Reference
    ComPtr<IDepthFrameReference> depthFrameReference;
    HRESULT ret = multiSourceFrame->get_DepthFrameReference( &depthFrameReference );
    if( FAILED( ret ) ){
        return;
    }

    // Retrieve Depth Frame
    ComPtr<IDepthFrame> depthFrame;
    ret = depthFrameReference->AcquireFrame( &depthFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Retrieve Depth Data
    ERROR_CHECK( depthFrame->CopyFrameDataToArray( static_cast<UINT>( depthBuffer.size() ), &depthBuffer[0] ) );
}

// Draw Data
void Kinect::draw()
{
    // Draw Color
    drawColor();

    // Draw Depth
    drawDepth();
}

// Draw Color
inline void Kinect::drawColor()
{
    // Create cv::Mat from Color Buffer
    colorMat = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
}

// Draw Depth
inline void Kinect::drawDepth()
{
    // Create cv::Mat from Depth Buffer
    depthMat = cv::Mat( depthHeight, depthWidth, CV_16UC1, &depthBuffer[0] );
}

// Show Data
void Kinect::show()
{
    // Show Color
    showColor();

    // Show Depth
    showDepth();
}

// Show Color
inline void Kinect::showColor()
{
    if( colorMat.empty() ){
        return;
    }

    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( colorMat, resizeMat, cv::Size(), scale, scale );

    // Show Image
    cv::imshow( "Color", resizeMat );
}

// Show Depth
inline void Kinect::showDepth()
{
    if( depthMat.empty() ){
        return;
    }

    // Scaling ( 0-8000 -> 255-0 )
    cv::Mat scaleMat;
    depthMat.convertTo( scaleMat, CV_8U, -255.0 / 8000.0, 255.0 );
    //cv::applyColorMap( scaleMat, scaleMat, cv::COLORMAP_BONE );

    // Show Image
    cv::imshow( "Depth", scaleMat );
}