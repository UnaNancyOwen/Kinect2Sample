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

    // Initialize Recognition
    initializeRecognition();

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
    const DWORD features = FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace;

    Concurrency::parallel_for( 0, BODY_COUNT, [&]( const int count ){
        // Create Face Sources
        ComPtr<IFaceFrameSource> faceFrameSource;
        ERROR_CHECK( CreateFaceFrameSource( kinect.Get(), 0, features, &faceFrameSource ) );

        // Open Face Readers
        ERROR_CHECK( faceFrameSource->OpenReader( &faceFrameReader[count] ) );
    } );
}

// Initialize Recognition
inline void Kinect::initializeRecognition()
{
    // Create Recognizer
    //recognizer = cv::face::createFisherFaceRecognizer();
    //recognizer = cv::face::createEigenFaceRecognizer();
    recognizer = cv::face::createLBPHFaceRecognizer();

    // Load Recognizer
    recognizer->load( model );
    if( recognizer.empty() ){
        throw std::runtime_error( "failed cv::face::FaceRecognizer::load()" );
    }

    // Set Distance Threshold
    recognizer->setThreshold( threshold );
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

    // Update Recognition
    updateRecognition();
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

// Update Recognition
inline void Kinect::updateRecognition()
{
    // Create cv::Mat from Color Buffer
    cv::Mat colorMat = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
    if( colorMat.empty() ){
        return;
    }

    // ReSet Labels and Distances
    labels.fill( -1 );
    distances.fill( 0.0 );

    Concurrency::parallel_for( 0, BODY_COUNT, [&]( const int count ){
        const ComPtr<IFaceFrameResult> result = results[count];
        if( result == nullptr ){
            return;
        }

        // Retrieve Face Bounding Box
        RectI boundingBox;
        ERROR_CHECK( result->get_FaceBoundingBoxInColorSpace( &boundingBox ) );

        // Retrieve Face
        const cv::Rect roi = { boundingBox.Left, boundingBox.Top, ( boundingBox.Right - boundingBox.Left ), ( boundingBox.Bottom - boundingBox.Top ) };
        cv::Mat faceMat = colorMat( roi ).clone();
        if( faceMat.empty() ){
            return;
        }

        // Resize
        //cv::resize( faceMat, faceMat, cv::Size( 200, 200 ) );

        // Convert BGRA to Gray
        cv::cvtColor( faceMat, faceMat, cv::COLOR_BGRA2GRAY );

        // Recognition
        recognizer->predict( faceMat, labels[count], distances[count] );
    } );
}

// Draw Data
void Kinect::draw()
{
    // Draw Color
    drawColor();

    // Draw Recognition
    drawRecognition();
}

// Draw Color
inline void Kinect::drawColor()
{
    // Create cv::Mat from Color Buffer
    colorMat = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
}

// Draw Recognition
inline void Kinect::drawRecognition()
{
    if( colorMat.empty() ){
        return;
    }

    Concurrency::parallel_for( 0, BODY_COUNT, [&]( const int count ){
        const ComPtr<IFaceFrameResult> result = results[count];
        if( result == nullptr ){
            return;
        }

        std::cout << count << std::endl;

        // Retrieve Label and Distance
        const int label = labels[count];
        const double distance = distances[count];

        // Set Draw Color by Recognition Results
        const cv::Vec3b color = ( label != -1 ) ? cv::Vec3b( 0, 255, 0 ) : cv::Vec3b( 0, 0, 255 );

        // Draw Face Bounding Box
        RectI boundingBox;
        ERROR_CHECK( result->get_FaceBoundingBoxInColorSpace( &boundingBox ) );
        drawFaceBoundingBox( colorMat, boundingBox, color );

        // Draw Recognition Results
        drawRecognitionResults( colorMat, label, distance, cv::Point( boundingBox.Left, boundingBox.Top ), 1.0, color );
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

// Draw Recognition Results
inline void Kinect::drawRecognitionResults( cv::Mat& image, const int label, const double distance, const cv::Point& point, const double scale, const cv::Vec3b& color, const int thickness )
{
    if( image.empty() ){
        return;
    }

    // Set Recognition Results
    std::string result;
    if( label != -1 ){
        result = std::to_string( label ) + " (" + std::to_string( distance ) + ")";
        //result = recognizer->getLabelInfo( label );
    }
    else{
        result = "Unknown";
    }

    // Draw Recognition Results
    cv::putText( image, result, point, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA );
}

// Show Data
void Kinect::show()
{
    // Show Recognition
    showRecognition();
}

// Show Recognition
inline void Kinect::showRecognition()
{
    if( colorMat.empty() ){
        return;
    }

    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( colorMat, resizeMat, cv::Size(), scale, scale );

    // Show Image
    cv::imshow( "Recognition", resizeMat );
}