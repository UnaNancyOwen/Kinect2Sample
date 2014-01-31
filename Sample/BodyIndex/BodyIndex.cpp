// BodyIndex.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
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
	IBodyIndexFrameSource* pBodyIndexSource;
	hResult = pSensor->get_BodyIndexFrameSource( &pBodyIndexSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_BodyIndexFrameSource()" << std::endl;
		return -1;
	}

	// Reader
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
	cv::namedWindow( "BodyIndex" );

	// Color Table
	cv::Vec3b color[BODY_COUNT];
	color[0] = cv::Vec3b( 255,   0,   0 );
	color[1] = cv::Vec3b(   0, 255,   0 );
	color[2] = cv::Vec3b(   0,   0, 255 );
	color[3] = cv::Vec3b( 255, 255,   0 );
	color[4] = cv::Vec3b( 255,   0, 255 );
	color[5] = cv::Vec3b(   0, 255, 255 );

	while( 1 ){
		// Frame
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
						if( buffer[index] != 0xff ){
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

		cv::imshow( "BodyIndex", bodyIndexMat );

		if( cv::waitKey( 30 ) == VK_ESCAPE ){
			break;
		}
	}

	SafeRelease( pBodyIndexSource );
	SafeRelease( pBodyIndexReader );
	SafeRelease( pDescription );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	cv::destroyAllWindows();

	return 0;
}

