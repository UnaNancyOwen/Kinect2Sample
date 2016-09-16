#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>

#include <ppl.h>

// Choose Resolution
#define COLOR
//#define DEPTH

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

    // Initialize Color
    initializeColor();

    // Initialize Depth
    initializeDepth();

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
    // Update Color
    updateColor();

    // Update Depth
    updateDepth();

    // Update BodyIndex
    updateBodyIndex();
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
    // Draw Color
    drawColor();

    // Draw BodyIndex
    drawBodyIndex();

    // Draw ChromaKey
    drawChromaKey();
}

// Draw Color
inline void Kinect::drawColor()
{
#ifdef COLOR
    // Create cv::Mat from Color Buffer
    colorMat = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
#endif

#ifdef DEPTH
    // Retrieve Mapped Coordinates
    std::vector<ColorSpacePoint> colorSpacePoints( depthWidth * depthHeight );
    ERROR_CHECK( coordinateMapper->MapDepthFrameToColorSpace( depthBuffer.size(), &depthBuffer[0], colorSpacePoints.size(), &colorSpacePoints[0] ) );

    // Mapping Color to Depth Resolution
    std::vector<BYTE> buffer( depthWidth * depthHeight * colorBytesPerPixel );

    Concurrency::parallel_for( 0, depthHeight, [&]( const int depthY ){
        const unsigned int depthOffset = depthY * depthWidth;
        for( int depthX = 0; depthX < depthWidth; depthX++ ){
            unsigned int depthIndex = depthOffset + depthX;
            const int colorX = static_cast<int>( colorSpacePoints[depthIndex].X + 0.5f );
            const int colorY = static_cast<int>( colorSpacePoints[depthIndex].Y + 0.5f );
            if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                const unsigned int colorIndex = ( colorY * colorWidth + colorX ) * colorBytesPerPixel;
                depthIndex = depthIndex * colorBytesPerPixel;
                buffer[depthIndex + 0] = colorBuffer[colorIndex + 0];
                buffer[depthIndex + 1] = colorBuffer[colorIndex + 1];
                buffer[depthIndex + 2] = colorBuffer[colorIndex + 2];
                buffer[depthIndex + 3] = colorBuffer[colorIndex + 3];
            }
        }
    } );

    // Create cv::Mat from Coordinate Buffer
    colorMat = cv::Mat( depthHeight, depthWidth, CV_8UC4, &buffer[0] ).clone();
#endif
}

// Draw BodyIndex
inline void Kinect::drawBodyIndex()
{
#ifdef COLOR
    // Retrieve Mapped Coordinates
    std::vector<DepthSpacePoint> bodyIndexSpacePoints( colorWidth * colorHeight );
    ERROR_CHECK( coordinateMapper->MapColorFrameToDepthSpace( depthBuffer.size(), &depthBuffer[0], bodyIndexSpacePoints.size(), &bodyIndexSpacePoints[0] ) );

    // Mapping BodyIndex to Color Resolution
    std::vector<BYTE> buffer( colorWidth * colorHeight, 0xff );

    Concurrency::parallel_for( 0, colorHeight, [&]( const int colorY ){
        const unsigned int colorOffset = colorY * colorWidth;
        for( int colorX = 0; colorX < colorWidth; colorX++ ){
            const unsigned int colorIndex = colorOffset + colorX;
            const int bodyIndexX = static_cast<int>( bodyIndexSpacePoints[colorIndex].X + 0.5f );
            const int bodyIndexY = static_cast<int>( bodyIndexSpacePoints[colorIndex].Y + 0.5f );
            if( ( 0 <= bodyIndexX ) && ( bodyIndexX < bodyIndexWidth ) && ( 0 <= bodyIndexY ) && ( bodyIndexY < bodyIndexHeight ) ){
                const unsigned char bodyIndex = bodyIndexBuffer[bodyIndexY * bodyIndexWidth + bodyIndexX];
                buffer[colorIndex] = bodyIndex;
            }
        }
    } );

    // Create cv::Mat from Coordinate Buffer
    bodyIndexMat = cv::Mat( colorHeight, colorWidth, CV_8UC1, &buffer[0] ).clone();
#endif

#ifdef DEPTH
    // Create cv::Mat from BodyIndex Buffer
    bodyIndexMat = cv::Mat( bodyIndexHeight, bodyIndexWidth, CV_8UC1, &bodyIndexBuffer[0] );
#endif
}

// Draw ChromaKey
inline void Kinect::drawChromaKey()
{
    if( colorMat.empty() ){
        return;
    }

    if( bodyIndexMat.empty() ){
        return;
    }

    // ChromaKey
#ifdef COLOR
    chromaKeyMat = cv::Mat::zeros( colorHeight, colorWidth, CV_8UC4 );
#endif
#ifdef DEPTH
    chromaKeyMat = cv::Mat::zeros( depthHeight, depthWidth, CV_8UC4 );
#endif
    chromaKeyMat.forEach<cv::Vec4b>( [&]( cv::Vec4b &p, const int* position ){
        uchar bodyIndex = bodyIndexMat.at<uchar>( position[0], position[1] );
        if( bodyIndex != 0xff ){
            p = colorMat.at<cv::Vec4b>( position[0], position[1] );
        }
    } );
}

// Show Data
void Kinect::show()
{
    // Show ChromaKey
    showChromaKey();
}

// Show ChromaKey
inline void Kinect::showChromaKey()
{
    if( chromaKeyMat.empty() ){
        return;
    }

#ifdef COLOR
    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( chromaKeyMat, resizeMat, cv::Size(), scale, scale );

    // Show Image
    cv::imshow( "ChromaKey", resizeMat );
#endif

#ifdef DEPTH
    // Show Image
    cv::imshow( "ChromaKey", chromaKeyMat );
#endif
}