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

    // Initialize Infrared
    initializeInfrared();

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

// Initialize Infrared
inline void Kinect::initializeInfrared()
{
    // Open Infrared Reader
    ComPtr<IInfraredFrameSource> infraredFrameSource;
    ERROR_CHECK( kinect->get_InfraredFrameSource( &infraredFrameSource ) );
    ERROR_CHECK( infraredFrameSource->OpenReader( &infraredFrameReader ) );

    // Retrieve Infrared Description
    ComPtr<IFrameDescription> infraredFrameDescription;
    ERROR_CHECK( infraredFrameSource->get_FrameDescription( &infraredFrameDescription ) );
    ERROR_CHECK( infraredFrameDescription->get_Width( &infraredWidth ) ); // 512
    ERROR_CHECK( infraredFrameDescription->get_Height( &infraredHeight ) ); // 424
    ERROR_CHECK( infraredFrameDescription->get_BytesPerPixel( &infraredBytesPerPixel ) ); // 2

    // Allocation Depth Buffer
    infraredBuffer.resize( infraredWidth * infraredHeight );
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
    // Update Infrared
    updateInfrared();
}

// Update Infrared
inline void Kinect::updateInfrared()
{
    // Retrieve Infrared Frame
    ComPtr<IInfraredFrame> infraredFrame;
    const HRESULT ret = infraredFrameReader->AcquireLatestFrame( &infraredFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Retrieve Infrared Data
    ERROR_CHECK( infraredFrame->CopyFrameDataToArray( static_cast<UINT>( infraredBuffer.size() ), &infraredBuffer[0] ) );
}

// Draw Data
void Kinect::draw()
{
    // Draw Infrared
    drawInfrared();
}

// Draw Infrared
inline void Kinect::drawInfrared()
{
    // Create cv::Mat from Infrared Buffer
    infraredMat = cv::Mat( infraredHeight, infraredWidth, CV_16UC1, &infraredBuffer[0] );
}

// Show Data
void Kinect::show()
{
    // Show Infrared
    showInfrared();
}

// Show Infrared
inline void Kinect::showInfrared()
{
    if( infraredMat.empty() ){
        return;
    }

    // Scaling ( 0b1111'1111'0000'0000 -> 0b1111'1111 )
    cv::Mat scaleMat( infraredHeight, infraredWidth, CV_8UC1 );
    scaleMat.forEach<uchar>([&]( uchar &p, const int* position ){
        p = infraredMat.at<ushort>( position[0], position[1] ) >> 8;
    });

    // Show Image
    cv::imshow( "Infrared", scaleMat );
}