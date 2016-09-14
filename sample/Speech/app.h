#ifndef __APP__
#define __APP__

#include <Windows.h>
#include <Kinect.h>
// Quote from Kinect for Windows SDK v2.0 - Sample/Native/SpeechBasics-D2D
// KinectAudioStream.h and .cpp : Copyright (c) Microsoft Corporation.  All rights reserved.
#include "KinectAudioStream.h"
#include <sapi.h>

#include <vector>
#include <string>

#include <wrl/client.h>
using namespace Microsoft::WRL;

class Kinect
{
private:
    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Speech
    ComPtr<IAudioBeam> audioBeam;
    ComPtr<IStream> inputStream;
    ComPtr<KinectAudioStream> audioStream;
    ComPtr<ISpStream> speechStream;
    ComPtr<ISpRecognizer> speechRecognizer;
    ComPtr<ISpRecoContext> speechContext;
    std::vector<ComPtr<ISpRecoGrammar>> speechGrammar;
    HANDLE speechEvent;
    DWORD objects = 0;

    // Speech Buffer
    std::wstring recognizeResult;
    const float confidenceThreshold = 0.3f;
    bool exit = false;

public:
    // Constructor
    Kinect();

    // Destructor
    ~Kinect();

    // Processing
    void run();

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    inline void initializeSensor();

    // Initialize Audio
    inline void initializeAudio();

    // Initialize Speech
    inline void initializeSpeech();

    // Initialize Speech Stream
    inline void initializeSpeechStream();

    // Create Speech Recognizer
    inline void createSpeechRecognizer( const std::string& language = "en-US" );

    // Load Speech Recognition Grammar from Grammar File (*.grxml)
    inline void loadSpeechGrammar( const ULONGLONG id, const std::wstring& grammar );

    // Finalize
    void finalize();

    // Start Speech Recognition
    void start();

    // Stop Speech Recognition
    void stop();

    // Update Data
    void update();

    // Update Speech
    inline void updateSpeech();

    // Draw Data
    void draw();

    // Draw Speech
    inline void drawSpeech();

    // Retrive Speech Recognition Result
    inline void result();

    // Show Data
    void show();

    // Show Speech
    inline void showSpeech();
};

#endif // __APP__