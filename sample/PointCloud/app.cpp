#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <string>

#include <ppl.h>

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
    while( !viewer.wasStopped() ){
        // Update Data
        update();

        // Draw Data
        draw();

        // Show Data
        show();

        // Wait a Few Time
        std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
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

    // Initialize Point Cloud
    initializePointCloud();

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

// Initialize Point Cloud
inline void Kinect::initializePointCloud()
{
    // Create Window
    viewer = cv::viz::Viz3d( "Point Cloud" );

    // Register Keyboard Callback Function
    viewer.registerKeyboardCallback( &keyboardCallback, this );

    // Show Coordinate System
    viewer.showWidget( "CoordinateSystem", cv::viz::WCameraPosition::WCameraPosition( 0.5 ) );
}

// Keyboard Callback Function
void Kinect::keyboardCallback( const cv::viz::KeyboardEvent& event, void* cookie )
{
    // Exit Viewer when Pressed ESC key
    if( event.code == VK_ESCAPE && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
        // Retrieve Viewer
        cv::viz::Viz3d viewer = static_cast<Kinect*>( cookie )->viewer;

        // Close Viewer
        viewer.close();
    }
    // Save Point Cloud to File when Pressed 's' key
    else if( event.code == 's' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
        // Retrieve Point Cloud and Color
        cv::Mat cloud = static_cast<Kinect*>( cookie )->cloudMat;
        cv::Mat color = static_cast<Kinect*>( cookie )->colorMat;

        // Generate File Name
        static int i = 0;
        std::ostringstream oss;
        oss << std::setfill( '0' ) << std::setw( 3 ) << i++;
        std::string file = oss.str() + ".ply";

        // Write Point Cloud to File
        cv::viz::writeCloud( file, cloud, color, cv::noArray(), false );
    }
};

// Finalize
void Kinect::finalize()
{
    cv::viz::unregisterAllWindows();

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

    // Draw Point Cloud
    drawPointCloud();
}

// Draw Color
inline void Kinect::drawColor()
{
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
}

// Draw Point Cloud
inline void Kinect::drawPointCloud()
{
    // Retrieve Mapped Coordinates
    std::vector<CameraSpacePoint> cameraSpacePoints( depthWidth * depthHeight );
    ERROR_CHECK( coordinateMapper->MapDepthFrameToCameraSpace( depthBuffer.size(), &depthBuffer[0], cameraSpacePoints.size(), &cameraSpacePoints[0] ) );

    // Mapping Color to Depth Resolution
    std::vector<cv::Vec3f> buffer( depthWidth * depthHeight, cv::Vec3f::all( std::numeric_limits<float>::quiet_NaN() ) );

    Concurrency::parallel_for( 0, depthHeight, [&]( const int depthY ){
        const unsigned int depthOffset = depthY * depthWidth;
        for( int depthX = 0; depthX < depthWidth; depthX++ ){
            unsigned int depthIndex = depthOffset + depthX;
            UINT16 depth = depthBuffer[depthIndex];
            if( 500 <= depth && depth < 8000 ){
                CameraSpacePoint cameraSpacePoint = cameraSpacePoints[depthIndex];
                buffer[depthIndex] = cv::Vec3f( cameraSpacePoint.X, cameraSpacePoint.Y, cameraSpacePoint.Z );
            }
        }
    } );

    // Create cv::Mat from Coordinate Buffer
    cloudMat = cv::Mat( depthHeight, depthWidth, CV_32FC3, &buffer[0] ).clone();
}

// Show Data
void Kinect::show()
{
    // Show Point Cloud
    showPointCloud();
}

// Show Point Cloud
inline void Kinect::showPointCloud()
{
    if( colorMat.empty() ){
        return;
    }

    if( cloudMat.empty() ){
        return;
    }

    // Create Point Cloud
    cv::viz::WCloud cloud( cloudMat, colorMat );

    // Show Point Cloud
    viewer.showWidget( "Cloud", cloud );
    viewer.spinOnce();
}