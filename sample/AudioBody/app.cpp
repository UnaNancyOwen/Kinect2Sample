#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>
#include <limits>

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

    // Initialize Audio
    initializeAudio();

    // Initialize Body
    initializeBody();

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

// Initialize Audio
inline void Kinect::initializeAudio()
{
    // Retrieve Audio Source
    ComPtr<IAudioSource> audioSource;
    ERROR_CHECK( kinect->get_AudioSource( &audioSource ) );

    // Open Audio Beam Reader
    ERROR_CHECK( audioSource->OpenReader( &audioBeamFrameReader ) );
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
    // Update Audio
    updateAudio();

    // Update Body
    updateBody();

    // Update BodyIndex
    updateBodyIndex();
}

// Update Audio
inline void Kinect::updateAudio()
{
    // Initialize Tracking ID
    audioTrackingId = std::numeric_limits<unsigned long long>::max() - 1;

    // Retrieve Audio Beam Frame List
    ComPtr<IAudioBeamFrameList> audioBeamFrameList;
    const HRESULT ret = audioBeamFrameReader->AcquireLatestBeamFrames( &audioBeamFrameList );
    if( FAILED( ret ) ){
        return;
    }

    // Retrieve Audio Beam Frame Count
    UINT beamCount = 0;
    ERROR_CHECK( audioBeamFrameList->get_BeamCount( &beamCount ) );

    for( int i = 0; i < beamCount; i++ ){
        // Retrieve Audio Beam Frame
        ComPtr<IAudioBeamFrame> audioBeamFrame;
        ERROR_CHECK( audioBeamFrameList->OpenAudioBeamFrame( i, &audioBeamFrame ) );

        // Retrieve Audio Beam SubFrame Count
        UINT subFrameCount = 0;
        ERROR_CHECK( audioBeamFrame->get_SubFrameCount( &subFrameCount ) );

        for( int j = 0; j < subFrameCount; j++ ){
            // Retrieve Audio Beam SubFrame
            ComPtr<IAudioBeamSubFrame> audioBeamSubFrame;
            ERROR_CHECK( audioBeamFrame->GetSubFrame( j, &audioBeamSubFrame ) );

            // Retrieve Audio Body Correlation Count
            UINT32 correlationCount;
            ERROR_CHECK( audioBeamSubFrame->get_AudioBodyCorrelationCount( &correlationCount ) );

            // Check Correlation Count
            if( correlationCount == 0 ){
                return;
            }

            // Retrieve First Audio Body Correlation
            ComPtr<IAudioBodyCorrelation> audioBodyCorrelation;
            ERROR_CHECK( audioBeamSubFrame->GetAudioBodyCorrelation( 0, &audioBodyCorrelation ) );

            // Retrieve Tracking ID
            ERROR_CHECK( audioBodyCorrelation->get_BodyTrackingId( &audioTrackingId ) );
        }
    }
}

// Update Body
inline void Kinect::updateBody()
{
    // Initialize Tracking Index
    audioTrackingIndex = -1;

    // Check Tracking ID
    if( audioTrackingId == std::numeric_limits<unsigned long long>::max() - 1 ){
        return;
    }

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

    for( int count = 0; count < BODY_COUNT; count++ ){
        // Retrive Tracking ID
        UINT64 trackingId;
        bodies[count]->get_TrackingId( &trackingId );

        // Check Tracking ID
        if( trackingId == audioTrackingId ){
            audioTrackingIndex = count;
            break;
        }
    }
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
    // Check Tracking Index
    if( audioTrackingIndex == -1 ){
        return;
    }

    // Visualization BodyIndex
    bodyIndexMat = cv::Mat::zeros( bodyIndexHeight, bodyIndexWidth, CV_8UC3 );
    bodyIndexMat.forEach<cv::Vec3b>( [&]( cv::Vec3b &p, const int* position ){
        uchar index = bodyIndexBuffer[position[0] * bodyIndexWidth + position[1]];
        if( index == audioTrackingIndex ){
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
    if( bodyIndexMat.empty() ){
        return;
    }

    // Show Image
    cv::imshow( "AudiBody", bodyIndexMat );
}