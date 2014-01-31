// AudioBody.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
// This source code is licensed under the MIT license. Please see the License in License.txt.
// "This is preliminary software and/or hardware and APIs are preliminary and subject to change."
//

#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>

#define NOT_TRACKING_ID    (_UI64_MAX - 1)
#define NOT_TRACKING_INDEX (-1)


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
	cv::setUseOptimized( true );

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

	IBodyFrameSource* pBodySource;
	hResult = pSensor->get_BodyFrameSource( &pBodySource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return -1;
	}

	IBodyIndexFrameSource* pBodyIndexSource;
	hResult = pSensor->get_BodyIndexFrameSource( &pBodyIndexSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_BodyIndexFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IAudioBeamFrameReader* pAudioBeamReader;
	hResult = pAudioSource->OpenReader( &pAudioBeamReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IAudioSource::OpenReader()" << std::endl;
		return -1;
	}

	IBodyFrameReader* pBodyReader;
	hResult = pBodySource->OpenReader( &pBodyReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IBodyIndexFrameReader* pBodyIndexReader;
	hResult = pBodyIndexSource->OpenReader( &pBodyIndexReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IBodyIndexFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	IFrameDescription* pDescription;
	hResult = pBodyIndexSource->get_FrameDescription( &pDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IBodyIndexFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int width = 0;
	int height = 0;
	pDescription->get_Width( &width ); // 512
	pDescription->get_Height( &height ); // 424

	cv::Mat bodyIndexMat( height, width, CV_8UC3 );
	cv::namedWindow( "AudioBody" );

	// Color Table
	cv::Vec3b color[BODY_COUNT];
	color[0] = cv::Vec3b( 255, 0, 0 );
	color[1] = cv::Vec3b( 0, 255, 0 );
	color[2] = cv::Vec3b( 0, 0, 255 );
	color[3] = cv::Vec3b( 255, 255, 0 );
	color[4] = cv::Vec3b( 255, 0, 255 );
	color[5] = cv::Vec3b( 0, 255, 255 );

	while( 1 ){
		bodyIndexMat = cv::Scalar( 0, 0, 0 );
		UINT64 audioTrackingId = NOT_TRACKING_ID;
		int trackingIndex = NOT_TRACKING_INDEX;

		// AudioBeam Frame
		IAudioBeamFrameList* pAudioBeamList = nullptr;
		hResult = pAudioBeamReader->AcquireLatestBeamFrames( &pAudioBeamList );
		if( SUCCEEDED( hResult ) ){
			IAudioBeamFrame* pAudioBeamFrame = nullptr;
			hResult = pAudioBeamList->OpenAudioBeamFrame( 0, &pAudioBeamFrame );
			if( SUCCEEDED( hResult ) ){
				IAudioBeamSubFrame* pAudioBeamSubFrame = nullptr;
				hResult = pAudioBeamFrame->GetSubFrame( 0, &pAudioBeamSubFrame );
				if( SUCCEEDED( hResult ) ){
					UINT32 correlationCount = 0;
					hResult = pAudioBeamSubFrame->get_AudioBodyCorrelationCount( &correlationCount );
					if( SUCCEEDED( hResult ) && ( correlationCount != 0 ) ){
						IAudioBodyCorrelation* pAudioBodyCorrelation = nullptr;
						hResult = pAudioBeamSubFrame->GetAudioBodyCorrelation( 0, &pAudioBodyCorrelation );
						if( SUCCEEDED( hResult ) ){
							hResult = pAudioBodyCorrelation->get_BodyTrackingId( &audioTrackingId );
						}
						SafeRelease( pAudioBodyCorrelation );
					}
				}
				SafeRelease( pAudioBeamSubFrame );
			}
			SafeRelease( pAudioBeamFrame );
		}
		SafeRelease( pAudioBeamList );

		// Body Frame
		IBodyFrame* pBodyFrame = nullptr;
		hResult = pBodyReader->AcquireLatestFrame( &pBodyFrame );
		if( SUCCEEDED( hResult ) ){
			IBody* pBody[BODY_COUNT] = { 0 };
			hResult = pBodyFrame->GetAndRefreshBodyData( BODY_COUNT, pBody );
			if( SUCCEEDED( hResult ) ){
				for( int count = 0; count < BODY_COUNT; count++ ){
					BOOLEAN bTracked = false;
					hResult = pBody[count]->get_IsTracked( &bTracked );
					if( SUCCEEDED( hResult ) && bTracked ){
						UINT64 bodyTrackingId = 0;
						hResult = pBody[count]->get_TrackingId( &bodyTrackingId );
						if( SUCCEEDED( hResult ) ){
							if( bodyTrackingId == audioTrackingId ){
								trackingIndex = count;
							}
						}
					}
				}
			}
			for( int count = 0; count < BODY_COUNT; count++ ){
				SafeRelease( pBody[count] );
			}
		}
		SafeRelease( pBodyFrame );

		// BodyIndex Frame
		IBodyIndexFrame* pBodyIndexFrame = nullptr;
		hResult = pBodyIndexReader->AcquireLatestFrame( &pBodyIndexFrame );
		if( SUCCEEDED( hResult ) ){
			unsigned int bufferSize = 0;
			unsigned char* buffer = nullptr;
			hResult = pBodyIndexFrame->AccessUnderlyingBuffer( &bufferSize, &buffer );
			if( SUCCEEDED( hResult ) ){
				for( int y = 0; y < height; y++ ){
					for( int x = 0; x < width; x++ ){
						unsigned int index = y * width + x;
						if( buffer[index] == trackingIndex ){
							bodyIndexMat.at<cv::Vec3b>( y, x ) = color[buffer[index]];
						}
						else{
							bodyIndexMat.at<cv::Vec3b>( y, x ) = cv::Vec3b( 0, 0, 0 );
						}
					}
				}
			}
		}
		SafeRelease( pBodyIndexFrame );

		// Draw Text Information
		if( trackingIndex != NOT_TRACKING_INDEX ){
			std::ostringstream stream;
			stream << audioTrackingId;
			cv::putText( bodyIndexMat, "trackingId : " + stream.str(), cv::Point( 20, 20 ), cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar( 255, 255, 255 ), 1, CV_AA );
			stream.str( "" );
			stream << trackingIndex;
			cv::putText( bodyIndexMat, "trackingIndex : " + stream.str(), cv::Point( 20, 40 ), cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar( 255, 255, 255 ), 1, CV_AA );
		}

		cv::imshow( "AudioBody", bodyIndexMat );

		// Input Key ( Exit ESC key )
		if( cv::waitKey( 30 ) == VK_ESCAPE ){
			break;
		}
	}

	SafeRelease( pAudioSource );
	SafeRelease( pBodySource );
	SafeRelease( pBodyIndexSource );
	SafeRelease( pAudioBeamReader );
	SafeRelease( pBodyReader );
	SafeRelease( pBodyIndexReader );
	SafeRelease( pDescription );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	cv::destroyAllWindows();

	return 0;
}

