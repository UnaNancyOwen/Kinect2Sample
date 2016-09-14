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

    // Initialize BodyIndex
    initializeBodyIndex();

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

// Initialize BodyIndex
inline void Kinect::initializeBodyIndex()
{
    // Open BodyIndex Reader
    ComPtr<IBodyIndexFrameSource> bodyIndexFrameSource;
    ERROR_CHECK( kinect->get_BodyIndexFrameSource( &bodyIndexFrameSource ) );
    ERROR_CHECK( bodyIndexFrameSource->OpenReader( &bodyIndexFrameReader ) );

    // Retrieve BodyIndex Description
    ComPtr<IFrameDescription> bodyIndexFrameDescription;
    ERROR_CHECK( bodyIndexFrameSource->get_FrameDescription( &bodyIndexFrameDescription ) );
    ERROR_CHECK( bodyIndexFrameDescription->get_Width( &bodyIndexWidth ) ); // 512
    ERROR_CHECK( bodyIndexFrameDescription->get_Height( &bodyIndexHeight ) ); // 424
    ERROR_CHECK( bodyIndexFrameDescription->get_BytesPerPixel( &bodyIndexBytesPerPixel ) ); // 1

    // Allocation BodyIndex Buffer
    bodyIndexBuffer.resize( bodyIndexWidth * bodyIndexHeight );

    // Color Table for Visualization
    colors[0] = cv::Vec3b( 255,   0,   0 ); // Blue
    colors[1] = cv::Vec3b(   0, 255,   0 ); // Green
    colors[2] = cv::Vec3b(   0,   0, 255 ); // Red
    colors[3] = cv::Vec3b( 255, 255,   0 ); // Cyan
    colors[4] = cv::Vec3b( 255,   0, 255 ); // Magenta
    colors[5] = cv::Vec3b(   0, 255, 255 ); // Yellow
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
    // Update BodyIndex
    updateBodyIndex();
}

// Update BodyIndex
inline void Kinect::updateBodyIndex()
{
    // Retrieve BodyIndex Frame
    ComPtr<IBodyIndexFrame> bodyIndexFrame;
    const HRESULT ret = bodyIndexFrameReader->AcquireLatestFrame( &bodyIndexFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Retrieve BodyIndex Data
    ERROR_CHECK( bodyIndexFrame->CopyFrameDataToArray( static_cast<UINT>( bodyIndexBuffer.size() ), &bodyIndexBuffer[0] ) );
}

// Draw Data
void Kinect::draw()
{
    // Draw BodyIndex
    drawBodyIndex();
}

// Draw BodyIndex
inline void Kinect::drawBodyIndex()
{
    // Visualization Color to Each Index
    bodyIndexMat = cv::Mat::zeros( bodyIndexHeight, bodyIndexWidth, CV_8UC3 );
    bodyIndexMat.forEach<cv::Vec3b>( [&]( cv::Vec3b &p, const int* position ){
        uchar index = bodyIndexBuffer[position[0] * bodyIndexWidth + position[1]];
        if( index != 0xff ){
            p = colors[index];
        }
    } );
}

// Show Data
void Kinect::show()
{
    // Show BodyIndex
    showBodyIndex();
}

// Show BodyIndex
inline void Kinect::showBodyIndex()
{
    // Show Image
    cv::imshow( "BodyIndex", bodyIndexMat );
}