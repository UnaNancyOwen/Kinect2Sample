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

    // Retrieve Depth Reliable Range
    UINT16 minReliableDistance;
    UINT16 maxReliableDistance;
    ERROR_CHECK( depthFrameSource->get_DepthMinReliableDistance( &minReliableDistance ) ); // 500
    ERROR_CHECK( depthFrameSource->get_DepthMaxReliableDistance( &maxReliableDistance ) ); // 4500
    std::cout << "Depth Reliable Range : " << minReliableDistance << " - " << maxReliableDistance << std::endl;

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
    // Update Depth
    updateDepth();
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

// Draw Data
void Kinect::draw()
{
    // Draw Depth
    drawDepth();
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
    // Show Depth
    showDepth();
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