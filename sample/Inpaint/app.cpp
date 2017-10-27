#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>

#include <ppl.h>

// Choose Resolution
//#define COLOR
#define DEPTH

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

        // Inpaint Depth
        inpaintDepth();

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
#else
    // Create cv::Mat from Color Buffer
    colorMat = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
#endif

}

// Draw Depth
inline void Kinect::drawDepth()
{
#ifdef COLOR
    // Retrieve Mapped Coordinates
    std::vector<DepthSpacePoint> depthSpacePoints( colorWidth * colorHeight );
    ERROR_CHECK( coordinateMapper->MapColorFrameToDepthSpace( depthBuffer.size(), &depthBuffer[0], depthSpacePoints.size(), &depthSpacePoints[0] ) );

    // Mapping Depth to Color Resolution
    std::vector<UINT16> buffer( colorWidth * colorHeight );

    Concurrency::parallel_for( 0, colorHeight, [&]( const int colorY ){
        const unsigned int colorOffset = colorY * colorWidth;
        for( int colorX = 0; colorX < colorWidth; colorX++ ){
            const unsigned int colorIndex = colorOffset + colorX;
            const int depthX = static_cast<int>( depthSpacePoints[colorIndex].X + 0.5f );
            const int depthY = static_cast<int>( depthSpacePoints[colorIndex].Y + 0.5f );
            if( ( 0 <= depthX ) && ( depthX < depthWidth ) && ( 0 <= depthY ) && ( depthY < depthHeight ) ){
                const unsigned int depthIndex = depthY * depthWidth + depthX;
                buffer[colorIndex] = depthBuffer[depthIndex];
            }
        }
    } );

    // Create cv::Mat from Coordinate Buffer
    depthMat = cv::Mat( colorHeight, colorWidth, CV_16UC1, &buffer[0] ).clone();
#else
    // Create cv::Mat from Depth Buffer
    depthMat = cv::Mat( depthHeight, depthWidth, CV_16UC1, &depthBuffer[0]);
#endif
}

// Inpaint Depth
void Kinect::inpaintDepth()
{
    if( depthMat.empty() ){
        return;
    }

    // Create Inpaint Mask ( This mask is area where depth couldn't be retrieved becauses shadow, noise, or outside of range. )
    cv::Mat maskMat;
    cv::threshold( depthMat, maskMat, 500, std::numeric_limits<unsigned short>::max(), cv::THRESH_BINARY_INV );
    maskMat.convertTo( maskMat, CV_8U, 255.0 / std::numeric_limits<unsigned short>::max() );

#ifdef COLOR
    // Scale Down
    const double scale = 0.3;
    cv::Mat resizeDepthMat;
    cv::Mat resizeMaskMat;
    cv::resize( depthMat, resizeDepthMat, cv::Size(), scale, scale );
    cv::resize( maskMat, resizeMaskMat, cv::Size(), scale, scale );

    // Inpaint Depth
    const double radius = 5.0;
    cv::inpaint( resizeDepthMat, resizeMaskMat, inpaintMat, radius, cv::INPAINT_NS );

    // Scale Up
    cv::resize( inpaintMat, inpaintMat, depthMat.size() );

    // Add Copy
    cv::Mat tmpMat = depthMat.clone();
    inpaintMat.copyTo( tmpMat, maskMat );
    inpaintMat = tmpMat.clone();
#else
    // Inpaint Depth
    const double radius = 5.0;
    cv::inpaint( depthMat, maskMat, inpaintMat, radius, cv::INPAINT_NS );
#endif
}

// Show Data
void Kinect::show()
{
    // Show Color
    showColor();

    // Show Depth
    showDepth();

    // Show Inpaint
    showInpaint();
}

// Show Color
inline void Kinect::showColor()
{
    if( colorMat.empty() ){
        return;
    }

#ifdef COLOR
    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( colorMat, resizeMat, cv::Size(), scale, scale );

    // Show Image
    cv::imshow( "Color", resizeMat );
#else
    // Show Image
    cv::imshow( "Color", colorMat );
#endif
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

#ifdef COLOR
    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( scaleMat, resizeMat, cv::Size(), scale, scale );

    // Show Image
    cv::imshow( "Depth", resizeMat );
#else
    // Show Image
    cv::imshow( "Depth", scaleMat );
#endif
}

// Show Inpaint
inline void Kinect::showInpaint()
{
    if( inpaintMat.empty() ){
        return;
    }

    // Scaling ( 0-8000 -> 255-0 )
    cv::Mat scaleMat;
    inpaintMat.convertTo( scaleMat, CV_8U, -255.0 / 8000.0, 255.0 );
    //cv::applyColorMap( scaleMat, scaleMat, cv::COLORMAP_BONE );

#ifdef COLOR
    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( scaleMat, resizeMat, cv::Size(), scale, scale );

    // Show Image
    cv::imshow( "Inpaint", resizeMat );
#else
    // Show Image
    cv::imshow( "Inpaint", scaleMat );
#endif
}