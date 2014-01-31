// Event.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
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

template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease, WAITABLE_HANDLE & pWaitableHandleToRelease )
{
	if( pInterfaceToRelease != NULL && pWaitableHandleToRelease != NULL ){
		pInterfaceToRelease->UnsubscribeFrameArrived( pWaitableHandleToRelease );
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
		pWaitableHandleToRelease = NULL;
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
	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource( &pColorSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader( &pColorReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	IFrameDescription* pDescription;
	hResult = pColorSource->get_FrameDescription( &pDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int width = 0;
	int height = 0;
	pDescription->get_Width( &width ); // 1920
	pDescription->get_Height( &height ); // 1080
	unsigned int bufferSize = width * height * 4 * sizeof( unsigned char );

	cv::Mat bufferMat( height, width, CV_8UC4 );
	cv::Mat colorMat( height / 2, width / 2, CV_8UC4 );
	cv::namedWindow( "Color" );

	// Subscribe Handle
	WAITABLE_HANDLE hColorWaitable;
	hResult = pColorReader->SubscribeFrameArrived( &hColorWaitable );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameReader::SubscribeFrameArrived()" << std::endl;
		return -1;
	}

	while( 1 ){
		// Waitable Events
		HANDLE hEvents[ ] = { reinterpret_cast<HANDLE>( hColorWaitable ) };
		WaitForMultipleObjects( ARRAYSIZE( hEvents ), hEvents, true, INFINITE );

		// Arrived Data
		IColorFrameArrivedEventArgs* pColorArgs = nullptr;
		hResult = pColorReader->GetFrameArrivedEventData( hColorWaitable, &pColorArgs );
		if( SUCCEEDED( hResult ) ){
			// Reference
			IColorFrameReference* pColorReference = nullptr;
			hResult = pColorArgs->get_FrameReference( &pColorReference );
			if( SUCCEEDED( hResult ) ){
				// Frame
				IColorFrame* pColorFrame = nullptr;
				hResult = pColorReference->AcquireFrame( &pColorFrame );
				if( SUCCEEDED( hResult ) ){
					hResult = pColorFrame->CopyConvertedFrameDataToArray( bufferSize, reinterpret_cast<BYTE*>( bufferMat.data ), ColorImageFormat::ColorImageFormat_Bgra );
					if( SUCCEEDED( hResult ) ){
						cv::resize( bufferMat, colorMat, cv::Size(), 0.5, 0.5 );
					}
				}
				SafeRelease( pColorFrame );
			}
			SafeRelease( pColorReference );
		}
		SafeRelease( pColorArgs );

		cv::imshow( "Color", colorMat );

		if( cv::waitKey( 30 ) == VK_ESCAPE ){
			break;
		}
	}

	SafeRelease( pColorSource );
	SafeRelease( pColorReader, hColorWaitable );
	SafeRelease( pDescription );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	cv::destroyAllWindows();

	return 0;
}

