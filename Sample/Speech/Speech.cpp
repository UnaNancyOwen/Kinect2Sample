// Speech.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
// This source code is licensed under the MIT license. Please see the License in License.txt.
// "This is preliminary software and/or hardware and APIs are preliminary and subject to change."
//

#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <iostream>

// Quote by Kinect for Windows SDK v2.0 Public Preview - Sample/Native/SpeechBasics-D2D
// KinectAudioStream.h and .cpp : Copyright (c) Microsoft Corporation.  All rights reserved.
#include "KinectAudioStream.h"

// Microsoft Speech Platform SDK 11
#include <sapi.h>
//#include <sphelper.h> // SpFindBestToken()
#include <strsafe.h>
#include <intsafe.h>

#pragma comment( lib, "sapi.lib" )


template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != NULL ){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

int _tmain( int argc, _TCHAR* argv[] )
{
	// Sensor
	IKinectSensor* pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor( &pSensor );
	if( FAILED( hResult ) ){
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return -1;
	}

	hResult = pSensor->Open();
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	// Source
	IAudioSource* pAudioSource;
	hResult = pSensor->get_AudioSource( &pAudioSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_AudioSource()" << std::endl;
		return -1;
	}

	// Get Audio Beam List
	IAudioBeamList* pAudioBeamList;
	hResult = pAudioSource->get_AudioBeams( &pAudioBeamList );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IAudioSource::get_AudioBeams()" << std::endl;
		return -1;
	}

	// Open Audio Beam
	IAudioBeam* pAudioBeam;
	hResult = pAudioBeamList->OpenAudioBeam( 0, &pAudioBeam );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IAudioBeamList::OpenAudioBeam()" << std::endl;
		return -1;
	}

	// Open Audio Input Stream 
	IStream* pAudioStream;
	hResult = pAudioBeam->OpenInputStream( &pAudioStream );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IAudioBeam::OpenInputStream()" << std::endl;
		return -1;
	}

	// Audio Stream Class from KinectAudioStream
	KinectAudioStream* audioStream = new KinectAudioStream( pAudioStream );

	// Initialize COM
	hResult = CoInitializeEx( NULL, COINIT_MULTITHREADED );
	if( FAILED( hResult ) ){
		std::cerr << "Error : CoInitializeEx()" << std::endl;
		return -1;
	}

	// Create Speech Stream Instance
	ISpStream* pSpeechStream;
	hResult = CoCreateInstance( CLSID_SpStream, NULL, CLSCTX_INPROC_SERVER, __uuidof( ISpStream ), ( void** )&pSpeechStream );
	if( FAILED( hResult ) ){
		std::cerr << "Error : CoCreateInstance( CLSID_SpStream )" << std::endl;
		return -1;
	}

	// Initialize Speech Stream
	WORD AudioFormat = WAVE_FORMAT_PCM;
	WORD AudioChannels = 1;
	DWORD AudioSamplesPerSecond = 16000;
	DWORD AudioAverageBytesPerSecond = 32000;
	WORD AudioBlockAlign = 2;
	WORD AudioBitsPerSample = 16;

	WAVEFORMATEX waveFormat = { AudioFormat, AudioChannels, AudioSamplesPerSecond, AudioAverageBytesPerSecond, AudioBlockAlign, AudioBitsPerSample, 0 };

	audioStream->SetSpeechState( true );
	hResult = pSpeechStream->SetBaseStream( audioStream, SPDFID_WaveFormatEx, &waveFormat );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpStream::SetBaseStream()" << std::endl;
		return -1;
	}

	/*** Create Speech Recognizer ***/
	std::cout << "Create Speech Recognizer" << std::endl;

	// Create Speech Recognizer Instance
	ISpRecognizer* pSpeechRecognizer;
	hResult = CoCreateInstance( CLSID_SpInprocRecognizer, NULL, CLSCTX_INPROC_SERVER, __uuidof( ISpRecognizer ), ( void** )&pSpeechRecognizer );
	if( FAILED( hResult ) ){
		std::cerr << "Error : CoCreateInstance( CLSID_SpInprocRecognizer )" << std::endl;
		return -1;
	}

	// Set Input Stream
	hResult = pSpeechRecognizer->SetInput( pSpeechStream, TRUE );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpRecognizer::SetInput()" << std::endl;
		return -1;
	}

	/*
	// If can use ATL, easier to using SpFindBestToken(sphelper.h). When using Visual Studio Professional or more.
	// ISpObjectToken* pEngineToken = nullptr;
	// hResult = SpFindBestToken( SPCAT_RECOGNIZERS, L"Language=409;Kinect=True", NULL, &pEngineToken );
	*/

	///*
	// If can't use ATL, alternative to using SpFIndBestToken(sphelper.h). When using Visual Studio Express.
	// Create Token Category Instance
	ISpObjectTokenCategory* pTokenCategory = nullptr;
	hResult = CoCreateInstance( CLSID_SpObjectTokenCategory, nullptr, CLSCTX_ALL, __uuidof( ISpObjectTokenCategory ), reinterpret_cast<void**>( &pTokenCategory ) );
	if( FAILED( hResult ) ){
		std::cerr << "Error : CoCreateInstance( CLSID_SpObjectTokenCategory )" << std::endl;
		return -1;
	}

	// Set Recognizers Catedory ID
	hResult = pTokenCategory->SetId( SPCAT_RECOGNIZERS, false );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpObjectTokenCategory::SetId()" << std::endl;
		return -1;
	}

	// Create Enum Tokens Instance
	IEnumSpObjectTokens* pEnumTokens = nullptr;
	hResult = CoCreateInstance( CLSID_SpMMAudioEnum, nullptr, CLSCTX_ALL, __uuidof( IEnumSpObjectTokens ), reinterpret_cast<void**>( &pEnumTokens ) );
	if( FAILED( hResult ) ){
		std::cerr << "Error : CoCreateInstance( CLSID_SpMMAudioEnum )" << std::endl;
		return -1;
	}

	// Find Best Token
	const wchar_t* pVendorPreferred = L"VendorPreferred";
	const unsigned long lengthVendorPreferred = static_cast<unsigned long>( wcslen( pVendorPreferred ) );
	unsigned long length;
	ULongAdd( lengthVendorPreferred, 1, &length );
	wchar_t* pAttribsVendorPreferred = new wchar_t[length];
	StringCchCopyW( pAttribsVendorPreferred, length, pVendorPreferred );

	hResult = pTokenCategory->EnumTokens( L"Language=409;Kinect=True", pAttribsVendorPreferred, &pEnumTokens ); // Japanese "Language=411;Kinect=True" English "Language=409;Kinect=True"
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpObjectTokenCategory::EnumTokens()" << std::endl;
		return -1;
	}
	SafeRelease( pTokenCategory );
	delete[] pAttribsVendorPreferred;

	// Retrieved Speech Recognize Engine
	ISpObjectToken* pEngineToken = nullptr;
	hResult = pEnumTokens->Next( 1, &pEngineToken, nullptr );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpObjectToken::Next()" << std::endl;
		return -1;
	}
	SafeRelease( pEnumTokens );
	//*/

	// Set Speech Recognizer
	hResult = pSpeechRecognizer->SetRecognizer( pEngineToken );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpRecognizer::SetRecognizer()" << std::endl;
		return -1;
	}
	SafeRelease( pEngineToken );

	// Create Context
	ISpRecoContext* pSpeechContext;
	hResult = pSpeechRecognizer->CreateRecoContext( &pSpeechContext );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpRecognizer::CreateRecoContext()" << std::endl;
		return -1;
	}

	// Set Property Name and Number
	hResult = pSpeechRecognizer->SetPropertyNum( L"AdaptationOn", 0 );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpObjectToken::Next()" << std::endl;
		return -1;
	}

	/*** LoadSpeechGrammar ***/
	std::cout << "Load Speech Grammar" << std::endl;

	// Create Grammar
	ISpRecoGrammar* pSpeechGrammar;
	hResult = pSpeechContext->CreateGrammar( 1, &pSpeechGrammar );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpRecoContext::CreateGrammar()" << std::endl;
		return -1;
	}

	// Load Grammar File (*.grxml)
	hResult = pSpeechGrammar->LoadCmdFromFile( L"SpeechRecognition.grxml", SPLO_STATIC ); // http://www.w3.org/TR/speech-grammar/ (UTF-8/CRLF)
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpRecoGrammar::LoadCmdFromFile()" << std::endl;
		return -1;
	}

	/*** Start Speech Recognition ***/
	std::cout << "Start Speech Recognition" << std::endl;

	hResult = pSpeechGrammar->SetRuleState( NULL, NULL, SPRS_ACTIVE );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpRecoGrammar::SetRuleState()" << std::endl;
		return -1;
	}

	hResult = pSpeechRecognizer->SetRecoState( SPRST_ACTIVE_ALWAYS );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpRecognizer::SetRecoState()" << std::endl;
		return -1;
	}

	hResult = pSpeechContext->SetInterest( SPFEI( SPEI_RECOGNITION ), SPFEI( SPEI_RECOGNITION ) );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpRecoContext::SetInterest()" << std::endl;
		return -1;
	}

	hResult = pSpeechContext->Resume( 0 );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ISpRecoContext::Resume()" << std::endl;
		return -1;
	}

	HANDLE hSpeechEvent = INVALID_HANDLE_VALUE;
	hSpeechEvent = pSpeechContext->GetNotifyEventHandle();
	HANDLE hEvents[1] = { hSpeechEvent };

	bool exit = false;

	while( 1 ){
		// Waitable Events
		ResetEvent( hSpeechEvent );
		unsigned long waitObject = MsgWaitForMultipleObjectsEx( ARRAYSIZE( hEvents ), hEvents, INFINITE, QS_ALLINPUT, MWMO_INPUTAVAILABLE );

		if( waitObject == WAIT_OBJECT_0 ){
			// Retrieved Event
			const float confidenceThreshold = 0.3f;
			SPEVENT eventStatus;
			unsigned long eventFetch = 0;
			pSpeechContext->GetEvents( 1, &eventStatus, &eventFetch );
			while( eventFetch > 0 ){
				switch( eventStatus.eEventId ){
					// Speech Recognition Events
					//   SPEI_HYPOTHESIS  : Estimate
					//   SPEI_RECOGNITION : Recognition
					case SPEI_HYPOTHESIS:
					case SPEI_RECOGNITION:
						if( eventStatus.elParamType == SPET_LPARAM_IS_OBJECT ){
							// Retrieved Phrase
							ISpRecoResult* pRecoResult = reinterpret_cast< ISpRecoResult* >( eventStatus.lParam );
							SPPHRASE* pPhrase = nullptr;
							hResult = pRecoResult->GetPhrase( &pPhrase );
							if( SUCCEEDED( hResult ) ){
								if( ( pPhrase->pProperties != nullptr ) && ( pPhrase->pProperties->pFirstChild != nullptr ) ){
									// Compared with the Phrase Tag in the grammar file
									const SPPHRASEPROPERTY* pSemantic = pPhrase->pProperties->pFirstChild;
									if( pSemantic->SREngineConfidence > confidenceThreshold ){
										if( wcscmp( L"Red", pSemantic->pszValue ) == 0 ){
											std::cout << "Red" << std::endl;
										}
										else if( wcscmp( L"Green", pSemantic->pszValue ) == 0 ){
											std::cout << "Green" << std::endl;
										}
										else if( wcscmp( L"Blue", pSemantic->pszValue ) == 0 ){
											std::cout << "Blue" << std::endl;
										}
										else if( wcscmp( L"Exit", pSemantic->pszValue ) == 0 ){
											std::cout << "Exit" << std::endl;
											exit = true;
										}
									}
								}
								CoTaskMemFree( pPhrase );
							}
						}
						break;

					default:
						break;
				}
				pSpeechContext->GetEvents( 1, &eventStatus, &eventFetch );
			}
		}

		// Input Key ( Exit ESC key )
		if( GetKeyState( VK_ESCAPE ) < 0 || exit ){
			break;
		}
	}

	pSpeechRecognizer->SetRecoState( SPRST_INACTIVE_WITH_PURGE );
	audioStream->SetSpeechState( false );
	SafeRelease( pSpeechStream );
	SafeRelease( pSpeechRecognizer );
	SafeRelease( pSpeechContext );
	SafeRelease( pSpeechGrammar );
	CoUninitialize();
	if( audioStream != NULL ){
		delete audioStream;
		audioStream = NULL;
	}

	SafeRelease( pAudioStream );
	SafeRelease( pAudioBeam );
	SafeRelease( pAudioBeamList );
	SafeRelease( pAudioSource );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	CloseHandle( hSpeechEvent );

	return 0;
}

