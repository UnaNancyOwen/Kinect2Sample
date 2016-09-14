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
        FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
      | FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
      | FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
      | FaceFrameFeatures::FaceFrameFeatures_Happy
      | FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
      | FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
      | FaceFrameFeatures::FaceFrameFeatures_MouthOpen
      | FaceFrameFeatures::FaceFrameFeatures_MouthMoved
      | FaceFrameFeatures::FaceFrameFeatures_LookingAway
      | FaceFrameFeatures::FaceFrameFeatures_Glasses
      | FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

    Concurrency::parallel_for( 0, BODY_COUNT, [&]( const int count ){
        // Create Face Sources
        ComPtr<IFaceFrameSource> faceFrameSource;
        ERROR_CHECK( CreateFaceFrameSource( kinect.Get(), 0, features, &faceFrameSource ) );

        // Open Face Readers
        ERROR_CHECK( faceFrameSource->OpenReader( &faceFrameReader[count] ) );
    } );

    // Color Table for Visualization
    colors[0] = cv::Vec3b( 255,   0,   0 ); // Blue
    colors[1] = cv::Vec3b(   0, 255,   0 ); // Green
    colors[2] = cv::Vec3b(   0,   0, 255 ); // Red
    colors[3] = cv::Vec3b( 255, 255,   0 ); // Cyan
    colors[4] = cv::Vec3b( 255,   0, 255 ); // Magenta
    colors[5] = cv::Vec3b(   0, 255, 255 ); // Yellow

    // Face Property Label Text Table for Display
    labels[0] = "Happy";
    labels[1] = "Engaged";
    labels[2] = "WearingGlasses";
    labels[3] = "LeftEyeClosed";
    labels[4] = "RightEyeClosed";
    labels[5] = "MouthOpen";
    labels[6] = "MouthMoved";
    labels[7] = "LookingAway";
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

    // Draw Face
    drawFace();
}

// Draw Color
inline void Kinect::drawColor()
{
    // Create cv::Mat from Color Buffer
    colorMat = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
}

// Draw Face
inline void Kinect::drawFace()
{
    if( colorMat.empty() ){
        return;
    }

    Concurrency::parallel_for( 0, BODY_COUNT, [&]( const int count ){
        const ComPtr<IFaceFrameResult> result = results[count];
        if( result == nullptr ){
            return;
        }

        // Retrieve Face Points
        std::array<PointF, FacePointType::FacePointType_Count> facePoints;
        ERROR_CHECK( result->GetFacePointsInColorSpace( FacePointType::FacePointType_Count, &facePoints[0] ) );
        drawFacePoints( colorMat, facePoints, 5, colors[count] );

        // Retrieve Face Bounding Box
        RectI boundingBox;
        ERROR_CHECK( result->get_FaceBoundingBoxInColorSpace( &boundingBox ) );
        drawFaceBoundingBox( colorMat, boundingBox, colors[count] );

        // Retrieve Face Rotation Quaternion
        Vector4 rotationQuaternion;
        ERROR_CHECK( result->get_FaceRotationQuaternion( &rotationQuaternion ) );
        drawFaceRotation( colorMat, rotationQuaternion, boundingBox, 1.0, colors[count] );

        // Retrieve Face Properties
        std::array<DetectionResult, FaceProperty::FaceProperty_Count> detectionResults;
        ERROR_CHECK( result->GetFaceProperties( FaceProperty::FaceProperty_Count, &detectionResults[0] ) );
        drawFaceProperties( colorMat, detectionResults, boundingBox, 1.0, colors[count] );
    } );
}

// Draw Face Points
inline void Kinect::drawFacePoints( cv::Mat& image, const std::array<PointF, FacePointType::FacePointType_Count>& points, const int radius, const cv::Vec3b& color, const int thickness )
{
    if( image.empty() ){
        return;
    }

    // Draw Points
    Concurrency::parallel_for_each( points.begin(), points.end(), [&]( const PointF point ){
        const int x = static_cast<int>( point.X + 0.5f );
        const int y = static_cast<int>( point.Y + 0.5f );
        cv::circle( image, cv::Point( x, y ), radius, static_cast<cv::Scalar>( color ), thickness, cv::LINE_AA );
    } );
}

// Draw Face Bounding Box
inline void Kinect::drawFaceBoundingBox( cv::Mat& image, const RectI& box, const cv::Vec3b& color, const int thickness )
{
    if( image.empty() ){
        return;
    }

    // Draw Bounding Box
    const int width = box.Right - box.Left;
    const int height = box.Bottom - box.Top;
    cv::rectangle( image, cv::Rect( box.Left, box.Top, width, height ), color, thickness, cv::LINE_AA );
}

// Draw Face Rotation Quaternion
inline void Kinect::drawFaceRotation( cv::Mat& image, Vector4& quaternion, const RectI& box, const double fontScale,const cv::Vec3b& color, const int thickness )
{
    if( image.empty() ){
        return;
    }

    // Convert Quaternion to Degree
    int pitch, yaw, roll;
    quaternion2degree( &quaternion, &pitch, &yaw, &roll );

    // Draw Rotation
    const int offset = 30;
    if( box.Left && box.Bottom ){
        std::string rotation = "Pitch, Yaw, Roll : " + std::to_string( pitch ) + ", " + std::to_string( yaw ) + ", " + std::to_string( roll );
        cv::putText( image, rotation, cv::Point( box.Left, box.Bottom + offset ), cv::FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv::LINE_AA );
    }
}

// Convert Quaternion to Degree
inline void Kinect::quaternion2degree( const Vector4* quaternion, int* pitch, int* yaw, int* roll )
{
    const double x = quaternion->x;
    const double y = quaternion->y;
    const double z = quaternion->z;
    const double w = quaternion->w;

    *pitch = static_cast<int>( std::atan2( 2 * ( y * z + w * x ), w * w - x * x - y * y + z * z ) / M_PI * 180.0f );
    *yaw = static_cast<int>( std::asin( 2 * ( w * y - x * z ) ) / M_PI * 180.0f );
    *roll = static_cast<int>( std::atan2( 2 * ( x * y + w * z ), w * w + x * x - y * y - z * z ) / M_PI * 180.0f );
}

// Draw Face Properties
inline void Kinect::drawFaceProperties( cv::Mat& image, std::array<DetectionResult, FaceProperty::FaceProperty_Count>& results, const RectI& box, const double fontScale, const cv::Vec3b& color, const int thickness )
{
    if( image.empty() ){
        return;
    }

    // Draw Properties
    int offset = 30;
    for( int count = 0; count < FaceProperty::FaceProperty_Count; count++ ){
        if( box.Left && box.Bottom ){
            offset += 30;
            std::string result = labels[count] + " : " + result2string( results[count] );
            cv::putText( image, result, cv::Point( box.Left, box.Bottom + offset ), cv::FONT_HERSHEY_SIMPLEX, fontScale, color, thickness, cv::LINE_AA );
        }
    }
}

// Convert Detection Result to String
inline std::string Kinect::result2string( DetectionResult& result )
{
    switch( result ){
        case DetectionResult::DetectionResult_Yes:
            return "Yes";
        case DetectionResult::DetectionResult_Maybe:
            return "Maybe";
        case DetectionResult::DetectionResult_No:
            return "No";
        case DetectionResult::DetectionResult_Unknown:
            return "Unknown";
        default:
            std::runtime_error( "not detection result of face property" );
    }

    return "";
}

// Show Data
void Kinect::show()
{
    // Show Face
    showFace();
}

// Show Face
inline void Kinect::showFace()
{
    if( colorMat.empty() ){
        return;
    }

    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( colorMat, resizeMat, cv::Size(), scale, scale );

    // Show Image
    cv::imshow( "Face", resizeMat );
}