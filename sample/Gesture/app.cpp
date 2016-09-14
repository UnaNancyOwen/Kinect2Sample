#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>
#include <iostream>
#include <iomanip>

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

    // Initialize Gesture
    initializeGesture();

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

// Initialize Gesture
inline void Kinect::initializeGesture()
{
    for( int count = 0; count < BODY_COUNT; count++ ){
        // Create Gesture Source
        ComPtr<IVisualGestureBuilderFrameSource> gestureFrameSource;
        ERROR_CHECK( CreateVisualGestureBuilderFrameSource( kinect.Get(), 0, &gestureFrameSource ) );

        // Open Gesture Reader
        ERROR_CHECK( gestureFrameSource->OpenReader( &gestureFrameReader[count] ) );
    };

    // Read Gesture Databese from File (*.gdb)
    // SampleDatabase recognize gestures that is steering wheel operations of car.
    ComPtr<IVisualGestureBuilderDatabase> gestureDatabase;
    ERROR_CHECK( CreateVisualGestureBuilderDatabaseInstanceFromFile( L"../SampleDatabase.gbd", &gestureDatabase ) );

    // Retrive Number of Gestures that included in Gesture Database
    UINT gestureCount;
    ERROR_CHECK( gestureDatabase->get_AvailableGesturesCount( &gestureCount ) );

    // Retrive Gestures
    gestures.resize( gestureCount );
    ERROR_CHECK( gestureDatabase->get_AvailableGestures( gestureCount, &gestures[0] ) );

    for( int count = 0; count < BODY_COUNT; count++ ){
        // Create Gesture Source
        ComPtr<IVisualGestureBuilderFrameSource> gestureFrameSource;
        ERROR_CHECK( gestureFrameReader[count]->get_VisualGestureBuilderFrameSource( &gestureFrameSource ) );

        // Registration Gestures
        ERROR_CHECK( gestureFrameSource->AddGestures( gestureCount, gestures[0].GetAddressOf() ) );

        // Set Gesture Detection to Enable
        Concurrency::parallel_for_each( gestures.begin(), gestures.end(), [&]( const ComPtr<IGesture>& gesture ){
            ERROR_CHECK( gestureFrameSource->SetIsEnabled( gesture.Get(), TRUE ) );
        } );
    }

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

    // Update Gesture
    updateGesture();
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
    std::array<ComPtr<IBody>, BODY_COUNT> bodies = { nullptr };
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
        ComPtr<IVisualGestureBuilderFrameSource> gestureFrameSource;
        ERROR_CHECK( gestureFrameReader[count]->get_VisualGestureBuilderFrameSource( &gestureFrameSource ) );
        gestureFrameSource->put_TrackingId( trackingId );
    } );
}

// Update Gesture
inline void Kinect::updateGesture()
{
    Concurrency::parallel_for( 0, BODY_COUNT, [&]( const int count ){
        // Clear Gesture Result Buffer
        std::vector<std::string>& result = results[count];
        result.clear();

        // Retrieve Gesture Frame
        ComPtr<IVisualGestureBuilderFrame> gestureFrame;
        HRESULT ret = gestureFrameReader[count]->CalculateAndAcquireLatestFrame( &gestureFrame );
        if( FAILED( ret ) ){
            return;
        }

        // Check Tracking ID is Valid
        BOOLEAN tracked;
        ERROR_CHECK( gestureFrame->get_IsTrackingIdValid( &tracked ) );
        if( !tracked ){
            return;
        }

        // Retrieve Gesture Result
        Concurrency::parallel_for_each( gestures.begin(), gestures.end(), [&]( const ComPtr<IGesture>& gesture ){
            // Switch Processing of Retrieve Gesture Result by Gesture Type
            GestureType gestureType;
            ERROR_CHECK( gesture->get_GestureType( &gestureType ) );

            switch( gestureType ){
                case GestureType::GestureType_Discrete:
                {
                    // Retrieve Discrete Gesture Result
                    const std::string gestureResult = retrieveDiscreteGestureResult( gestureFrame, gesture );

                    // Add Gesture Result to Buffer
                    if( !gestureResult.empty() ){
                        result.push_back( gestureResult );
                    }

                    break;
                }
                case GestureType::GestureType_Continuous:
                {
                    // Retrieve Continuous Gesture Result
                    const std::string gestureResult = retrieveContinuousGestureResult( gestureFrame, gesture );

                    // Add Gesture Result Buffer
                    result.push_back( gestureResult );

                    break;
                }
            }
        } );
    } );
}

// Retrieve Discrete Gesture Result
inline std::string Kinect::retrieveDiscreteGestureResult( const ComPtr<IVisualGestureBuilderFrame>& gestureFrame, const ComPtr<IGesture>& gesture )
{
    // Retrieve Discrete Gesture Result
    ComPtr<IDiscreteGestureResult> gestureResult;
    ERROR_CHECK( gestureFrame->get_DiscreteGestureResult( gesture.Get(), &gestureResult ) );

    // Check Detected
    BOOLEAN detected;
    ERROR_CHECK( gestureResult->get_Detected( &detected ) );
    if( !detected ){
        return "";
    }

    // Retrieve Confidence ( 0.0f - 1.0f )
    float confidence;
    ERROR_CHECK( gestureResult->get_Confidence( &confidence ) );
    const std::string gestureConfidence = std::to_string( confidence );

    // Retrive Gesture Name
    const std::string gestureName = gesture2string( gesture );

    return gestureName + " : Detected (" + gestureConfidence + ")";
}

// Retrieve Continuous Gesture Result
inline std::string Kinect::retrieveContinuousGestureResult( const ComPtr<IVisualGestureBuilderFrame>& gestureFrame, const ComPtr<IGesture>& gesture )
{
    // Retrieve Continuous Gesture Result
    ComPtr<IContinuousGestureResult> gestureResult;
    ERROR_CHECK( gestureFrame->get_ContinuousGestureResult( gesture.Get(), &gestureResult ) );

    // Retrieve Progress ( 0.0f - 1.0f )
    float progress;
    ERROR_CHECK( gestureResult->get_Progress( &progress ) );

    // Adjustment Decimal Point Format ( Visualization to Two Decimal Places )
    std::ostringstream oss;
    oss << std::fixed << std::setprecision( 2 ) << ( progress * 100.0f );
    const std::string gestureProgress = oss.str();

    // Retrive Gesture Name
    const std::string gestureName = gesture2string( gesture );

    return gestureName + " : Progress " + gestureProgress + "%";
}

// Retrive Gesture Name
inline std::string Kinect::gesture2string( const ComPtr<IGesture>& gesture )
{
    // Retrive Gesture Name
    std::wstring buffer( BUFSIZ, L' ' );
    ERROR_CHECK( gesture->get_Name( BUFSIZ, &buffer[0] ) );

    // Trim Valid String Except Blank
    const std::wstring::size_type last = buffer.find_last_not_of( L' ' );
    if( last == std::wstring::npos ){
        throw std::runtime_error( "failed " __FUNCTION__ );
    }
    const std::wstring temp = buffer.substr( 0, last );

    // Convert Wide String to String
    const std::string name( temp.begin(), temp.end() );

    return name;
}

// Draw Data
void Kinect::draw()
{
    // Draw Color
    drawColor();

    // Draw Gesture
    drawGesture();
}

// Draw Color
inline void Kinect::drawColor()
{
    // Create cv::Mat from Color Buffer
    colorMat = cv::Mat( colorHeight, colorWidth, CV_8UC4, &colorBuffer[0] );
}

// Draw Gesture
inline void Kinect::drawGesture()
{
    if( colorMat.empty() ){
        return;
    }

    // Reset New Line Offset for Visualization
    offset = 0;

    // Draw Gesture Results
    Concurrency::parallel_for( 0, BODY_COUNT, [&]( const int count ){
        const std::vector<std::string>& result = results[count];
        drawResult( colorMat, result, cv::Point( 50, 50 ), 1.0, colors[count] );
    } );
}

// Draw Results
inline void Kinect::drawResult( cv::Mat& image, const std::vector<std::string>& results, const cv::Point& point, const double scale, const cv::Vec3b& color, const int thickness )
{
    if( image.empty() ){
        return;
    }

    // Check Empty Gesture Result Buffer
    if( results.empty() ){
        return;
    }

    // Draw Results
    for( const std::string result : results ){
        cv::putText( image, result, cv::Point( point.x, point.y + offset ), cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA );
        offset += 30;
    }
}

// Show Data
void Kinect::show()
{
    // Show Gesture
    showGesture();
}

// Show Gesture
inline void Kinect::showGesture()
{
    if( colorMat.empty() ){
        return;
    }

    // Resize Image
    cv::Mat resizeMat;
    const double scale = 0.5;
    cv::resize( colorMat, resizeMat, cv::Size(), scale, scale );

    // Show Image
    cv::imshow( "Gesture", resizeMat );
}