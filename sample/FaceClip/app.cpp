#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>
#define _USE_MATH_DEFINES
#include <math.h>

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
    while( true ){
        // Update Data
        update();

        // Draw Data
        draw();

        // Show Data
        show();

        // Key Check
        const int key = cv::waitKey( 10 );
        if( key == VK_ESCAPE || GetKeyState( VK_ESCAPE ) < 0 ){
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

    // Initialize Body
    initializeBody();

    // Initialize Face
    initializeFace();

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

// Initialize Body
inline void Kinect::initializeBody()
{
    // Open Body Reader
    ComPtr<IBodyFrameSource> bodyFrameSource;
    ERROR_CHECK( kinect->get_BodyFrameSource( &bodyFrameSource ) );
    ERROR_CHECK( bodyFrameSource->OpenReader( &bodyFrameReader ) );

    // Initialize Body Buffer
    Concurrency::parallel_for_each( bodies.begin(), bodies.end(), []( IBody*& body ){
        SafeRelease( body );
    } );
}

// Initialize Face
inline void Kinect::initializeFace()
{
    // Set Face Features to Enable
    const DWORD features =
        FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace;

    Concurrency::parallel_for( 0, BODY_COUNT, [&]( const int count ){
        // Create Face Sources
        ComPtr<IFaceFrameSource> faceFrameSource;
        ERROR_CHECK( CreateFaceFrameSource( kinect.Get(), 0, features, &faceFrameSource ) );

        // Open Face Readers
        ERROR_CHECK( faceFrameSource->OpenReader( &faceFrameReader[count] ) );
    } );
}

// Finalize
void Kinect::finalize()
{
    cv::destroyAllWindows();

    // Release Body Buffer
    Concurrency::parallel_for_each( bodies.begin(), bodies.end(), []( IBody*& body ){
        SafeRelease( body );
    } );

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

    // Update Body
    updateBody();

    // Update Face
    updateFace();
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

// Update Body
inline void Kinect::updateBody()
{
    // Retrieve Body Frame
    ComPtr<IBodyFrame> bodyFrame;
    const HRESULT ret = bodyFrameReader->AcquireLatestFrame( &bodyFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Release Previous Bodies
    Concurrency::parallel_for_each( bodies.begin(), bodies.end(), []( IBody*& body ){
        SafeRelease( body );
    } );

    // Retrieve Body Data
    ERROR_CHECK( bodyFrame->GetAndRefreshBodyData( static_cast<UINT>( bodies.size() ), &bodies[0] ) );
    Concurrency::parallel_for( 0, BODY_COUNT, [&]( const int count ){
        const ComPtr<IBody> body = bodies[count];
        BOOLEAN tracked;
        ERROR_CHECK( body->get_IsTracked( &tracked ) );
        if( !tracked ){
            return;
        }

        // Retrieve Tracking ID
        UINT64 trackingId;
        ERROR_CHECK( body->get_TrackingId( &trackingId ) );

        // Registration Tracking ID
        ComPtr<IFaceFrameSource> faceFrameSource;
        ERROR_CHECK( faceFrameReader[count]->get_FaceFrameSource( &faceFrameSource ) );
        ERROR_CHECK( faceFrameSource->put_TrackingId( trackingId ) );
    } );
}

// Update Face
inline void Kinect::updateFace()
{
    // ReSet Results
    results.fill( nullptr );

    Concurrency::parallel_for( 0, BODY_COUNT, [&]( const int count ){
        // Retrieve Face Frame
        ComPtr<IFaceFrame> faceFrame;
        const HRESULT ret = faceFrameReader[count]->AcquireLatestFrame( &faceFrame );
        if( FAILED( ret ) ){
            return;
        }

        // Check Tracking ID is Valid
        BOOLEAN tracked;
        ERROR_CHECK( faceFrame->get_IsTrackingIdValid( &tracked ) );
        if( !tracked ){
            return;
        }

        // Release Previous Face Result and Retrieve Face Result
        ERROR_CHECK( faceFrame->get_FaceFrameResult( &results[count] ) );
    } );
}

// Draw Data
void Kinect::draw()
{
    // Draw Color
    drawColor();

    // Draw Face Clip
    drawFaceClip();
}

// Draw Color
inline void Kinect::drawColor()
{
    // Create cv::Mat from Color Buffer
    colorMat = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
}

// Draw Face
inline void Kinect::drawFaceClip()
{
    Concurrency::parallel_for( 0, BODY_COUNT, [&]( const int count ){
        const ComPtr<IFaceFrameResult> result = results[count];
        if( result == nullptr ){
            return;
        }

        // Retrieve Bounding Box
        RectI boundingBox;
        ERROR_CHECK( result->get_FaceBoundingBoxInColorSpace( &boundingBox ) );

        // Retrieve Face Clip using Bounding Box
        retrieveFaceClip( faceClipMat[count], boundingBox );
    } );
}

// Retrieve Face Clip
inline void Kinect::retrieveFaceClip( cv::Mat& image, const RectI& box )
{
    if( colorMat.empty() ){
        return;
    }

    // Retrieve Face Clip using Bounding Box
    const int width = box.Right - box.Left;
    const int height = box.Bottom - box.Top;
    image = colorMat( cv::Rect( box.Left, box.Top, width, height ) ).clone();
}

// Show Data
void Kinect::show()
{
    // Show Face Clip
    showFaceClip();
}

// Show Face Clip
inline void Kinect::showFaceClip()
{
    for( int count = 0; count < BODY_COUNT; count++ ){
        if( faceClipMat[count].empty() ){
            cv::destroyWindow( "Face" + std::to_string( count ) );
            continue;
        }

        // Resize Clip to Constant Size
        cv::resize( faceClipMat[count], faceClipMat[count], cv::Size( 200, 200 ) );

        // Show Image
        cv::imshow( "Face" + std::to_string( count ), faceClipMat[count] );
    }
}