#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>
#include <iostream>

#pragma warning(disable: 4996) // for error GetVersionExW() of sphelper.h
#include <sphelper.h> // for SpFindBestToken()
#include <locale.h>

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
    // Start Speech Recognition
    start();

    // Main Loop
    while( true ){
        // Update Data
        update();

        // Draw Data
        draw();

        // Show Data
        show();

        // Key Check
        if( GetKeyState( VK_ESCAPE ) < 0 || exit ){
            break;
        }
    }

    // Stop Speech Recognition
    stop();
}

// Initialize
void Kinect::initialize()
{
    // Initialize Sensor
    initializeSensor();

    // Initialize Audio
    initializeAudio();

    // Initialize Speech Recognition
    initializeSpeech();

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

    // Retrieve Audio Beam List
    ComPtr<IAudioBeamList> audioBeamList;
    ERROR_CHECK( audioSource->get_AudioBeams( &audioBeamList ) );

    // Open Audio Beam
    ERROR_CHECK( audioBeamList->OpenAudioBeam( 0, &audioBeam ) );

    // Open Audio Input Stream and Create Audio Stream
    ERROR_CHECK( audioBeam->OpenInputStream( &inputStream ) );
    audioStream = new KinectAudioStream( inputStream.Get() );
}

// Initialize Speech Recognition
inline void Kinect::initializeSpeech()
{
    CoInitialize( NULL );

    // Initialize Speech Stream
    initializeSpeechStream();

    // Create Speech Recognizer
    // "en-US" ... English, "ja-JP" ... Japanese
    createSpeechRecognizer( "en-US" );

    // Load Speech Recognition Grammar from Grammar File (*.grxml)
    // Grammar ID, Grammar File Name
    loadSpeechGrammar( 0, L"../Grammar_enUS.grxml" );
    /*loadSpeechGrammar( 1, L"../Grammar_Additional.grxml" );*/

    CoUninitialize();
}

// Initialize Speech Stream
inline void Kinect::initializeSpeechStream()
{
    // Create Speech Stream Instance
    CoCreateInstance( CLSID_SpStream, NULL, CLSCTX_INPROC_SERVER, __uuidof( ISpStream ), reinterpret_cast<void**>( speechStream.GetAddressOf() ) );

    // Set Wave Format
    WORD AudioFormat = WAVE_FORMAT_PCM;
    WORD AudioChannels = 1;
    DWORD AudioSamplesPerSecond = 16000;
    DWORD AudioAverageBytesPerSecond = 32000;
    WORD AudioBlockAlign = 2;
    WORD AudioBitsPerSample = 16;

    WAVEFORMATEX waveFormat = { AudioFormat, AudioChannels, AudioSamplesPerSecond, AudioAverageBytesPerSecond, AudioBlockAlign, AudioBitsPerSample, 0 };

    // Registration Base Stream
    ERROR_CHECK( speechStream->SetBaseStream( audioStream.Get(), SPDFID_WaveFormatEx, &waveFormat ) );
}

// Create Speech Recognizer
inline void Kinect::createSpeechRecognizer( const std::string& language )
{
    // Create Speech Recognizer Instance
    ERROR_CHECK( CoCreateInstance( CLSID_SpInprocRecognizer, NULL, CLSCTX_INPROC_SERVER, __uuidof( ISpRecognizer ), reinterpret_cast<void**>( speechRecognizer.GetAddressOf() ) ) );

    // Registration Input Stream
    ERROR_CHECK( speechRecognizer->SetInput( speechStream.Get(), TRUE ) );

    // Retrieve Language Attribute (Hexadecimal Value;Kinect Support)
    // Kinect for Windows SDK 2.0 Language Packs http://www.microsoft.com/en-us/download/details.aspx?id=43662
    // L"Language=409;Kinect=True" ... English | United States (MSKinectLangPack_enUS.msi)
    // L"Language=411;Kinect=True" ... Japanese | Japan (MSKinectLangPack_jaJP.msi)
    // Other Languages Hexadecimal Value, Please see here https://msdn.microsoft.com/en-us/library/hh378476(v=office.14).aspx
    std::wstring attribute;
    if( language == "de-DE" ){
        attribute = L"Language=C07;Kinect=True";
    }
    else if( language == "en-AU" ){
        attribute = L"Language=C09;Kinect=True";
    }
    else if( language == "en-CA" ){
        attribute = L"Language=1009;Kinect=True";
    }
    else if( language == "en-GB" ){
        attribute = L"Language=809;Kinect=True";
    }
    else if( language == "en-IE" ){
        attribute = L"Language=1809;Kinect=True";
    }
    else if( language == "en-NZ" ){
        attribute = L"Language=1409;Kinect=True";
    }
    else if( language == "en-US" ){
        attribute = L"Language=409;Kinect=True";
    }
    else if( language == "es-ES" ){
        attribute = L"Language=2C0A;Kinect=True";
    }
    else if( language == "es-MX" ){
        attribute = L"Language=80A;Kinect=True";
    }
    else if( language == "fr-CA" ){
        attribute = L"Language=C0C;Kinect=True";
    }
    else if( language == "fr-FR" ){
        attribute = L"Language=40c;Kinect=True";
    }
    else if( language == "it-IT" ){
        attribute = L"Language=410;Kinect=True";
    }
    else if( language == "ja-JP" ){
        attribute = L"Language=411;Kinect=True";
    }
    else{
        throw std::runtime_error( "failed " __FUNCTION__ );
    }

    // Set Local
    setlocale( LC_CTYPE, language.c_str() );

    // Retrieve and Registration Speech Recognizer Engine
    CComPtr<ISpObjectToken> engineToken;
    ERROR_CHECK( SpFindBestToken( SPCAT_RECOGNIZERS, attribute.c_str(), NULL, &engineToken ) );
    ERROR_CHECK( speechRecognizer->SetRecognizer( engineToken ) );

    // Create Speech Recognizer Context
    ERROR_CHECK( speechRecognizer->CreateRecoContext( &speechContext ) );

    // Set Adaptation of Acoustic Model to OFF (0)
    // (For Long Time (few hours~) Running Program of Speech Recognition)
    ERROR_CHECK( speechRecognizer->SetPropertyNum( L"AdaptationOn", 0 ) );
}

// Load Speech Recognition Grammar from Grammar File (*.grxml)
inline void Kinect::loadSpeechGrammar( const ULONGLONG id, const std::wstring& grammar )
{
    // Load Speech Recognition Grammar from Grammar File (*.grxml)
    speechGrammar.push_back( nullptr );
    ERROR_CHECK( speechContext->CreateGrammar( id, &speechGrammar.back() ) );
    ERROR_CHECK( speechGrammar.back()->LoadCmdFromFile( grammar.c_str(), SPLOADOPTIONS::SPLO_STATIC ) );
}

// Finalize
void Kinect::finalize()
{
    // Close Sensor
    if( kinect != nullptr ){
        kinect->Close();
    }
}

// Start Speech Recognition
void Kinect::start()
{
    std::cout << "start speech recognition..." << std::endl;

    // Set Audio Input Stream to Start
    audioStream->SetSpeechState( true );

    // Set Speech Recognition Grammar to Enable
    for( const ComPtr<ISpRecoGrammar> grammar : speechGrammar ){
        ERROR_CHECK( grammar->SetRuleState( NULL, NULL, SPRULESTATE::SPRS_ACTIVE ) );
    }

    // Set Recognition Status to Active
    ERROR_CHECK( speechRecognizer->SetRecoState( SPRECOSTATE::SPRST_ACTIVE_ALWAYS ) );

    // Set Event Generation Timing to Complete Speech Recognition
    ERROR_CHECK( speechContext->SetInterest( SPFEI( SPEVENTENUM::SPEI_RECOGNITION ), SPFEI( SPEVENTENUM::SPEI_RECOGNITION ) ) );

    // Set Speech Recognition to Resume
    ERROR_CHECK( speechContext->Resume( 0 ) );

    // Retrieve Speech Recognition Event Handle
    speechEvent = speechContext->GetNotifyEventHandle();
}

// Stop Speech Recognition
void Kinect::stop()
{
    // Set Audio Input Strem to Stop
    audioStream->SetSpeechState( false );

    // Set Recognition Status to Inactive
    ERROR_CHECK( speechRecognizer->SetRecoState( SPRECOSTATE::SPRST_INACTIVE_WITH_PURGE ) );

    // Set Speech Recognition to Pause
    ERROR_CHECK( speechContext->Pause( 0 ) );

    // Close Speech Recognition Event Handle
    CloseHandle( speechEvent );
}

// Update Data
void Kinect::update()
{
    // Update Speech
    updateSpeech();
}

// Update Speech
inline void Kinect::updateSpeech()
{
    // Wait Speech Recognition Event
    ResetEvent( speechEvent );
    const HANDLE events[1] = { speechEvent };
    objects = MsgWaitForMultipleObjectsEx( ARRAYSIZE( events ), events, 50, QS_ALLINPUT, MWMO_INPUTAVAILABLE );
}

// Draw Data
void Kinect::draw()
{
    // Draw Speech
    drawSpeech();
}

// Draw Speech
inline void Kinect::drawSpeech()
{
    // Clear Recognition Result Buffer
    recognizeResult.clear();

    switch( objects ){
        // Raising Speech Recognition Event
        case WAIT_OBJECT_0:
            // Retrive Speech Recognition Result
            result();
            break;
        default:
            break;
    }
}

// Retrive Speech Recognition Result
inline void Kinect::result()
{
    // Retrive Speech Recognition Event Status
    SPEVENT eventStatus;
    ULONG eventFetch;
    ERROR_CHECK( speechContext->GetEvents( 1, &eventStatus, &eventFetch ) );

    // Retrive Results
    while( eventFetch > 0 ){
        switch( eventStatus.eEventId ){
            // Speech Recognition Event Status
            case SPEVENTENUM::SPEI_RECOGNITION: 
                if( eventStatus.elParamType == SPET_LPARAM_IS_OBJECT ){
                    // Retrive Speech Recognition Results
                    ComPtr<ISpRecoResult> speechResult = reinterpret_cast<ISpRecoResult*>( eventStatus.lParam );

                    // Retrive Phrase
                    // <tag>PHRASE</tag>
                    SPPHRASE* phrase;
                    ERROR_CHECK( speechResult->GetPhrase( &phrase ) );
                    const SPPHRASEPROPERTY* semantic = phrase->pProperties->pFirstChild;
                    const std::wstring tag = semantic->pszValue;

                    // Retrive Text
                    // <item>TEXT</item>
                    wchar_t* text;
                    ERROR_CHECK( speechResult->GetText( SP_GETWHOLEPHRASE, SP_GETWHOLEPHRASE, FALSE, &text, NULL ) );

                    // Check Speech Recognition Confidence
                    if( semantic->SREngineConfidence > confidenceThreshold ){
                        // Add Phrase and Text to Result Buffer
                        recognizeResult = L"Phrase : " + tag + L"\t Text : " + text; 

                        // If Tag is "EXIT" Set Exit Flag to True
                        if( tag == L"EXIT" ){
                            exit = true;
                        }
                    }

                    // Release Memory
                    CoTaskMemFree( phrase );
                    CoTaskMemFree( text );
                }
                break;
            default:
                break;
        }
        ERROR_CHECK( speechContext->GetEvents( 1, &eventStatus, &eventFetch ) );
    }
}

// Show Data
void Kinect::show()
{
    // Show Speech
    showSpeech();
}

// Show Speech
inline void Kinect::showSpeech()
{
    // Check Empty Result Buffer
    if( !recognizeResult.size() ){
        return;
    }

    // Show Result
    std::wcout << recognizeResult << std::endl;

}