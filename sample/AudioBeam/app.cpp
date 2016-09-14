#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>

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
        if( GetKeyState( VK_ESCAPE ) < 0 ){
            break;
        }
    }
}

// Initialize
void Kinect::initialize()
{
    // Initialize Sensor
    initializeSensor();

    // Initialize Audio
    initializeAudio();

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

// Finalize
void Kinect::finalize()
{
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
}

// Update Audio
inline void Kinect::updateAudio()
{
    // Retrieve Audio Beam Frame List
    ComPtr<IAudioBeamFrameList> audioBeamFrameList;
    const HRESULT ret = audioBeamFrameReader->AcquireLatestBeamFrames( &audioBeamFrameList );
    if( FAILED( ret ) ){
        return;
    }

    //  Retrieve Audio Beam Frame Count
    UINT beamCount = 0;
    ERROR_CHECK( audioBeamFrameList->get_BeamCount( &beamCount ) );

    Concurrency::parallel_for( 0, static_cast<int>( beamCount ), [&]( int i ){
        // Retrieve Audio Beam Frame
        ComPtr<IAudioBeamFrame> audioBeamFrame;
        ERROR_CHECK( audioBeamFrameList->OpenAudioBeamFrame( i, &audioBeamFrame ) );

        // Retrieve Audio Beam SubFrame Count
        UINT subFrameCount = 0;
        ERROR_CHECK( audioBeamFrame->get_SubFrameCount( &subFrameCount ) );

        Concurrency::parallel_for( 0, static_cast<int>( subFrameCount ), [&]( int j ){
            // Retrieve Audio Beam SubFrame
            ComPtr<IAudioBeamSubFrame> audioBeamSubFrame;
            ERROR_CHECK( audioBeamFrame->GetSubFrame( j, &audioBeamSubFrame ) );

            // Retrieve Beam Angle ( Radian +/- 1.0 )
            ERROR_CHECK( audioBeamSubFrame->get_BeamAngle( &beamAngle ) );

            // Retrieve Beam Angle Confidence ( 0.0 - 1.0 )
            ERROR_CHECK( audioBeamSubFrame->get_BeamAngleConfidence( &beamAngleConfidence ) );
        } );
    } );
}

// Draw Data
void Kinect::draw()
{
    // Draw Audio
    drawAudio();
}

// Draw Audio
inline void Kinect::drawAudio()
{
    // Clear Beam Angle Result Buffer
    beamAngleResult.clear();

    // Check Beam Angle Confidence
    if( beamAngleConfidence > confidenceThreshold ){
        // Convert Degree from Radian
        const float degree = static_cast<float>( beamAngle * 180.0 / M_PI );

        // Add Beam Angle to Result Buffer
        beamAngleResult = std::to_string( degree ) + " (" + std::to_string( beamAngleConfidence ) + ")";
    }
}

// Show Data
void Kinect::show()
{
    // Show Audio
    showAudio();
}

// Show Audio
inline void Kinect::showAudio()
{
    // Check Empty Result Buffer
    if( !beamAngleResult.size() ){
        return;
    }

    // Show Result
    std::cout << beamAngleResult << std::endl;
}