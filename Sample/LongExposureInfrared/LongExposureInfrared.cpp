// LongExposureInfrared.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
// This source code is licensed under the MIT license. Please see the License in License.txt.
// "This is preliminary software and/or hardware and APIs are preliminary and subject to change."
//

#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>


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
	ILongExposureInfraredFrameSource* pLongExposureInfraredSource;
	hResult = pSensor->get_LongExposureInfraredFrameSource( &pLongExposureInfraredSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_LongExposureInfraredFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	ILongExposureInfraredFrameReader* pLongExposureInfraredReader;
	hResult = pLongExposureInfraredSource->OpenReader( &pLongExposureInfraredReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ILongExposureInfraredFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	IFrameDescription* pDescription;
	hResult = pLongExposureInfraredSource->get_FrameDescription( &pDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : ILongExposureInfraredFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int width = 0;
	int height = 0;
	pDescription->get_Width( &width ); // 512
	pDescription->get_Height( &height ); // 424

	cv::Mat infraredMat( height, width, CV_8UC1 );
	cv::namedWindow( "LongExposureInfrared" );

	while( 1 ){
		// Frame
		ILongExposureInfraredFrame* pLongExposureInfraredFrame = nullptr;
		hResult = pLongExposureInfraredReader->AcquireLatestFrame( &pLongExposureInfraredFrame );
		if( SUCCEEDED( hResult ) ){
			unsigned int bufferSize = 0;
			unsigned short* buffer = nullptr;
			hResult = pLongExposureInfraredFrame->AccessUnderlyingBuffer( &bufferSize, &buffer );
			if( SUCCEEDED( hResult ) ){
				for( int y = 0; y < height; y++ ){
					for( int x = 0; x < width; x++ ){
						unsigned int index = y * width + x;
						infraredMat.at<unsigned char>( y, x ) = buffer[index] >> 8;
					}
				}
			}
		}
		SafeRelease( pLongExposureInfraredFrame );

		cv::imshow( "LongExposureInfrared", infraredMat );

		if( cv::waitKey( 30 ) == VK_ESCAPE ){
			break;
		}
	}

	SafeRelease( pLongExposureInfraredSource );
	SafeRelease( pLongExposureInfraredReader );
	SafeRelease( pDescription );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	cv::destroyAllWindows();

	return 0;
}

