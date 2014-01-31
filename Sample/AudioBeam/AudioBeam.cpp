// AudioBeam.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
// This source code is licensed under the MIT license. Please see the License in License.txt.
// "This is preliminary software and/or hardware and APIs are preliminary and subject to change."
//

#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>


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

	// Reader
	IAudioBeamFrameReader* pAudioReader;
	hResult = pAudioSource->OpenReader( &pAudioReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IAudioSource::OpenReader()" << std::endl;
		return -1;
	}

	while( 1 ){
		// Frame List
		IAudioBeamFrameList* pAudioFrameList = nullptr;
		hResult = pAudioReader->AcquireLatestBeamFrames( &pAudioFrameList );
		if( SUCCEEDED( hResult ) ){
			UINT count = 0;
			hResult = pAudioFrameList->get_BeamCount( &count );
			if( SUCCEEDED( hResult ) ){
				for( int index = 0; index < count; index++ ){
					// Frame
					IAudioBeamFrame* pAudioFrame = nullptr;
					hResult = pAudioFrameList->OpenAudioBeamFrame( index, &pAudioFrame );
					if( SUCCEEDED( hResult ) ){
						// Get Beam Angle and Confidence
						IAudioBeam* pAudioBeam = nullptr;
						hResult = pAudioFrame->get_AudioBeam( &pAudioBeam );
						if( SUCCEEDED( hResult ) ){
							FLOAT angle = 0.0f;
							FLOAT confidence = 0.0f;
							pAudioBeam->get_BeamAngle( &angle ); // radian [-0.872665f, 0.872665f]
							pAudioBeam->get_BeamAngleConfidence( &confidence ); // confidence [0.0f, 1.0f]

							// Convert from radian to degree : degree = radian * 180 / Pi
							if( confidence > 0.5f ){
								std::cout << "Index : " << index << ", Angle : " << angle * 180.0f / M_PI << ", Confidence : " << confidence << std::endl;
							}
						}
						SafeRelease( pAudioBeam );
					}
					SafeRelease( pAudioFrame );
				}
			}
		}
		SafeRelease( pAudioFrameList );

		// Input Key ( Exit ESC key )
		if( GetKeyState( VK_ESCAPE ) < 0 ){
			break;
		}
	}

	SafeRelease( pAudioSource );
	SafeRelease( pAudioReader );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );

	return 0;
}

