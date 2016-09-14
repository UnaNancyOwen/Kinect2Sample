#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>
#include <limits>
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

    // Initialize HDFace
    initializeHDFace();

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
}

// Initialize HDFace
inline void Kinect::initializeHDFace()
{
    // Create HDFace Sources
    ComPtr<IHighDefinitionFaceFrameSource> hdFaceFrameSource;
    ERROR_CHECK( CreateHighDefinitionFaceFrameSource( kinect.Get(), &hdFaceFrameSource ) );

    // Open HDFace Readers
    ERROR_CHECK( hdFaceFrameSource->OpenReader( &hdFaceFrameReader ) );

    // Create Face Alignment
    ERROR_CHECK( CreateFaceAlignment( &faceAlignment ) );

    // Create Face Model and Retrieve Vertex Count
    ERROR_CHECK( CreateFaceModel( 1.0f, FaceShapeDeformations::FaceShapeDeformations_Count, &faceShapeUnits[0], &faceModel ) );
    ERROR_CHECK( GetFaceModelVertexCount( &vertexCount ) ); // 1347

    // Create and Start Face Model Builder
    FaceModelBuilderAttributes attribures = FaceModelBuilderAttributes::FaceModelBuilderAttributes_None;
    ERROR_CHECK( hdFaceFrameSource->OpenModelBuilder( attribures, &faceModelBuilder ) );
    ERROR_CHECK( faceModelBuilder->BeginFaceDataCollection() );

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
    // Update Color
    updateColor();

    // Update Body
    updateBody();

    // Update HDFace
    updateHDFace();
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

    // Retrieve Body Data
    std::array<ComPtr<IBody>, BODY_COUNT> bodies;
    ERROR_CHECK( bodyFrame->GetAndRefreshBodyData( static_cast<UINT>( bodies.size() ), &bodies[0] ) );

    // Find Closest Body
    findClosestBody( bodies );
}

// Find Closest Body
inline void Kinect::findClosestBody( const std::array<ComPtr<IBody>, BODY_COUNT>& bodies )
{
    float closestDistance = std::numeric_limits<float>::max();
    for( int count = 0; count < BODY_COUNT; count++ ){
        const ComPtr<IBody> body = bodies[count];
        BOOLEAN tracked;
        ERROR_CHECK( body->get_IsTracked( &tracked ) );
        if( !tracked ){
            continue;
        }

        // Retrieve Joint (Head)
        std::array<Joint, JointType::JointType_Count> joints;
        ERROR_CHECK( body->GetJoints( static_cast<UINT>( joints.size() ), &joints[0] ) );
        const Joint joint = joints[JointType::JointType_Head];
        if( joint.TrackingState == TrackingState::TrackingState_NotTracked ){
            continue;
        }

        // Calculate Distance from Sensor ( ��( x^2 + y^2 + z^2 ) )
        const CameraSpacePoint point = joint.Position;
        const float distance = std::sqrt( std::pow( point.X, 2 ) + std::pow( point.Y, 2 ) + std::pow( point.Z, 2 ) );
        if( closestDistance <= distance ){
            continue;
        }
        closestDistance = distance;

        // Retrieve Tracking ID
        UINT64 trackingId;
        ERROR_CHECK( body->get_TrackingId( &trackingId ) );
        if( this->trackingId == trackingId ){
            continue;
        }

        // Registration Tracking ID
        ComPtr<IHighDefinitionFaceFrameSource> hdFaceFrameSource;
        ERROR_CHECK( hdFaceFrameReader->get_HighDefinitionFaceFrameSource( &hdFaceFrameSource ) );
        ERROR_CHECK( hdFaceFrameSource->put_TrackingId( trackingId ) );

        // Update Current
        this->trackingId = trackingId;
        this->trackingCount = count;
        this->produced = false;
    }
}

// Update HDFace
inline void Kinect::updateHDFace()
{
    // Retrieve HDFace Frame
    ComPtr<IHighDefinitionFaceFrame> hdFaceFrame;
    const HRESULT ret = hdFaceFrameReader->AcquireLatestFrame( &hdFaceFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Check Traced
    BOOLEAN tracked;
    ERROR_CHECK( hdFaceFrame->get_IsFaceTracked( &tracked ) );
    if( !tracked ){
        return;
    }

    // Retrieve Face Alignment Result
    ERROR_CHECK( hdFaceFrame->GetAndRefreshFaceAlignmentResult( faceAlignment.Get() ) );

    // Check Face Model Builder Status
    FaceModelBuilderCollectionStatus collection;
    ERROR_CHECK( faceModelBuilder->get_CollectionStatus( &collection ) );
    if( collection ){
        return;
    }

    // Retrieve Fitting Face Model
    ComPtr<IFaceModelData> faceModelData;
    ERROR_CHECK( faceModelBuilder->GetFaceData( &faceModelData ) );
    ERROR_CHECK( faceModelData->ProduceFaceModel( &faceModel ) );
    produced = true;
}

// Draw Data
void Kinect::draw()
{
    // Draw Color
    drawColor();

    // Draw HDFace
    drawHDFace();
}

// Draw Color
inline void Kinect::drawColor()
{
    // Create cv::Mat from Color Buffer
    colorMat = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
}

// Draw HDFace
inline void Kinect::drawHDFace()
{
    if( colorMat.empty() ){
        return;
    }

    // Draw Face Model Builder Status
    drawFaceModelBuilderStatus( colorMat, cv::Point( 50, 50 ), 1.0, colors[trackingCount] );

    // Retrieve Vertexes
    std::vector<CameraSpacePoint> vertexes( vertexCount );
    ERROR_CHECK( faceModel->CalculateVerticesForAlignment( faceAlignment.Get(), vertexCount, &vertexes[0] ) );
    drawVertexes( colorMat, vertexes, 2, colors[trackingCount] );

    /*
    // Retrieve Head Pivot Point
    CameraSpacePoint point;
    ERROR_CHECK( faceAlignment->get_HeadPivotPoint( &point ) );
    std::cout << point.X << ", " << point.Y << ", " << point.Z << std::endl;
    */

    /*
    // Retrieve Animation Units ... Motion of Face Parts that Represent Expression (17 AUs)
    std::array<float, FaceShapeAnimations::FaceShapeAnimations_Count> animationUnits;
    ERROR_CHECK( faceAlignment->GetAnimationUnits( FaceShapeAnimations::FaceShapeAnimations_Count, &animationUnits[0] ) );
    for( const float animationUnit : animationUnits ){
        std::cout << std::to_string( animationUnit ) << std::endl;
    }
    */

    /*
    // Retrieve Shape Units ... Deformations from Default Face Model (94 SUs)
    ERROR_CHECK( faceModel->GetFaceShapeDeformations( FaceShapeDeformations::FaceShapeDeformations_Count, &shapeUnits[0] ) );
    for( const float shapeUnit : shapeUnits ){
        std::cout << std::to_string( shapeUnit ) << std::endl;
    }
    */

    /*
    // Retrieve Face Model Scale
    float scale;
    ERROR_CHECK( faceModel->get_Scale( &scale ) );
    std::cout << std::to_string( scale ) << std::endl;
    */

    /*
    // Retrieve Hair Color (XBGR)
    // Set FaceModelBuilderAttributes::FaceModelBuilderAttributes_HairColor to IHighDefinitionFaceFrameSource::OpenModelBuilder()
    UINT32 hairColor;
    ERROR_CHECK( faceModel->get_HairColor( &hairColor ) );
    std::cout << ( ( hairColor & 0xff000000 ) >> 24 ) << std::endl; // X
    std::cout << ( ( hairColor & 0x00ff0000 ) >> 16 ) << std::endl; // B
    std::cout << ( ( hairColor & 0x0000ff00 ) >>  8 ) << std::endl; // G
    std::cout << ( ( hairColor & 0x000000ff ) >>  0 ) << std::endl; // R
    */

    /*
    // Retrieve Skin Color (XBGR)
    // Set FaceModelBuilderAttributes::FaceModelBuilderAttributes_SkinColor to IHighDefinitionFaceFrameSource::OpenModelBuilder()
    UINT32 skinColor;
    ERROR_CHECK( faceModel->get_SkinColor( &skinColor ) );
    std::cout << ( ( skinColor & 0xff000000 ) >> 24 ) << std::endl; // X
    std::cout << ( ( skinColor & 0x00ff0000 ) >> 16 ) << std::endl; // B
    std::cout << ( ( skinColor & 0x0000ff00 ) >>  8 ) << std::endl; // G
    std::cout << ( ( skinColor & 0x000000ff ) >>  0 ) << std::endl; // R
    */
}

// Draw Face Model Builder Status
inline void Kinect::drawFaceModelBuilderStatus( cv::Mat& image, const cv::Point& point, const double scale, const cv::Vec3b& color, const int thickness )
{
    if( image.empty() ){
        return;
    }

    // Check Produced
    if( produced ){
        cv::putText( image, "Collection Complete", cv::Point( point.x, point.y ), cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA );
        return;
    }

    // Retrieve Face Model Builder Collection Status
    FaceModelBuilderCollectionStatus collection;
    ERROR_CHECK( faceModelBuilder->get_CollectionStatus( &collection ) );

    // Retrieve Face Model Builder Capture Status
    FaceModelBuilderCaptureStatus capture;
    ERROR_CHECK( faceModelBuilder->get_CaptureStatus( &capture ) );

    // Draw Status
    cv::putText( image, status2string( collection ), cv::Point( point.x, point.y ), cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA );
    cv::putText( image, status2string( capture ), cv::Point( point.x, point.y + 30 ), cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA );
}

// Convert Collection Status to String
inline std::string Kinect::status2string( const FaceModelBuilderCollectionStatus collection )
{
    std::string status;
    if( collection & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded ){
        status = "Collection Status : Needed Tilted Up Views";
    }
    else if( collection & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_RightViewsNeeded ){
        status = "Collection Status : Needed Right Views";
    }
    else if( collection & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_LeftViewsNeeded ){
        status = "Collection Status : Needed Left Views";
    }
    else if( collection & FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_FrontViewFramesNeeded ){
        status = "Collection Status : Needed Front View Frames";
    }

    return status;
}

// Convert Capture Status to String
inline std::string Kinect::status2string( const FaceModelBuilderCaptureStatus capture )
{
    std::string status;
    switch( capture ){
        case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooFar:
            status = "Capture Status : Warning Face Too Far from Camera";
            break;
        case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooNear:
            status = "Capture Status : WWarning Face Too Near to Camera";
            break;
        case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_MovingTooFast:
            status = "Capture Status : WWarning Moving Too Fast";
            break;
        default:
            status = "";
            break;
    }

    return status;
}

// Draw Vertexes
inline void Kinect::drawVertexes( cv::Mat& image, const std::vector<CameraSpacePoint> vertexes, const int radius, const cv::Vec3b& color, const int thickness )
{
    if( image.empty() ){
        return;
    }

    // Draw Vertex Points Converted to Color Coordinate System
    Concurrency::parallel_for_each( vertexes.begin(), vertexes.end(), [&]( const CameraSpacePoint vertex ){
        ColorSpacePoint point;
        ERROR_CHECK( coordinateMapper->MapCameraPointToColorSpace( vertex, &point ) );
        const int x = static_cast<int>( point.X + 0.5f );
        const int y = static_cast<int>( point.Y + 0.5f );
        if( ( 0 <= x ) && ( x < image.cols ) && ( 0 <= y ) && ( y < image.rows ) ){
            cv::circle( image, cv::Point( x, y ), radius, color, thickness, cv::LINE_AA );
        }
    } );
}

// Show Data
void Kinect::show()
{
    // Show HDFace
    showHDFace();
}

// Show HDFace
inline void Kinect::showHDFace()
{
    if( colorMat.empty() ){
        return;
    }

    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( colorMat, resizeMat, cv::Size(), scale, scale );

    // Show Image
    cv::imshow( "HDFace", resizeMat );
}