// Controls.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <opencv2/opencv.hpp>

#define WAIT_OBJECT_1 WAIT_OBJECT_0 + 1
#define WAIT_OBJECT_2 WAIT_OBJECT_0 + 2


template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != NULL ){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

int _tmain(int argc, _TCHAR* argv[])
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

	// Core Window
	IKinectCoreWindow* pCoreWindow;
	hResult = GetKinectCoreWindowForCurrentThread( &pCoreWindow );
	if( FAILED( hResult ) ){
		std::cerr << "Error : GetKinectCoreWindowForCurrentThread()" << std::endl;
		return -1;
	}

	// Subscribe Handle
	// for Moved pointer
	WAITABLE_HANDLE hMoved;
	hResult = pCoreWindow->SubscribePointerMoved( &hMoved );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectCoreWindow::SubscribePointerMoved()" << std::endl;
		return -1;
	}

	// for Entered pointer to in of the range
	WAITABLE_HANDLE hEntered;
	hResult = pCoreWindow->SubscribePointerEntered( &hEntered );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectCoreWindow::SubscribePointerEntered()" << std::endl;
		return -1;
	}

	// for Exited pointer to out of the range
	WAITABLE_HANDLE hExited;
	hResult = pCoreWindow->SubscribePointerExited( &hExited );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectCoreWindow::SubscribePointerExited()" << std::endl;
		return -1;
	}

	int width = 960;
	int height = 540;
	cv::Mat controlsMat = cv::Mat::zeros( height, width, CV_8UC3 );
	cv::namedWindow( "Controls" );

	while( 1 ){
		// Waitable Events
		HANDLE hEvents[] = { reinterpret_cast<HANDLE>( hMoved ), reinterpret_cast<HANDLE>( hEntered ), reinterpret_cast<HANDLE>( hExited ) };
		switch( WaitForMultipleObjects( ARRAYSIZE( hEvents ), hEvents, false, 1000 ) ){
			case WAIT_OBJECT_0:{
				IKinectPointerEventArgs* pPointerEvent = nullptr;
				hResult = pCoreWindow->GetPointerMovedEventData( hMoved, &pPointerEvent );
				if( SUCCEEDED( hResult ) && pPointerEvent != nullptr ){
					IKinectPointerPoint* pPointer = nullptr;
					hResult = pPointerEvent->get_CurrentPoint( &pPointer );
					if( SUCCEEDED( hResult ) && pPointer != nullptr ){
						IKinectPointerPointProperties* pProperty = nullptr;
						hResult = pPointer->get_Properties( &pProperty );
						if( SUCCEEDED( hResult ) ){
							boolean isInRange = 0;
							hResult = pProperty->get_IsInRange( &isInRange );
							if( SUCCEEDED( hResult ) && isInRange ){
								HandType handType;
								hResult = pProperty->get_HandType( &handType );
								if( SUCCEEDED( hResult ) ){
									controlsMat = cv::Scalar( 0, 0, 0 );
									// Left Hand
									if( handType == HandType::HandType_LEFT ){
										PointF point;
										hResult = pPointer->get_Position( &point );
										if( SUCCEEDED( hResult ) ){
											int x = static_cast<int>( point.X * width + 0.5f );
											int y = static_cast<int>( point.Y * height + 0.5f );
											cv::circle( controlsMat, cv::Point( x, y ), 5, cv::Scalar( 255, 255, 255 ), -1, CV_AA );
										}
									}
									// Right Hand
									else if( handType == HandType::HandType_RIGHT ){
										PointF point;
										hResult = pPointer->get_Position( &point );
										if( SUCCEEDED( hResult ) ){
											int x = static_cast<int>( point.X * width + 0.5f );
											int y = static_cast<int>( point.Y * height + 0.5f );
											cv::circle( controlsMat, cv::Point( x, y ), 5, cv::Scalar( 255, 255, 255 ), -1, CV_AA );
										}
									}
								}
							}
						}
						SafeRelease( pProperty );
					}
					SafeRelease( pPointer );
				}
				SafeRelease( pPointerEvent );
				break;
			}

			case WAIT_OBJECT_1:
				/* This handle called when pointer entered to in of the range. */
				break;

			case WAIT_OBJECT_2:
				/* This handle called when pointer exited to out of the range. */
				break;

			default:
				break;
		}

		cv::imshow( "Controls", controlsMat );
		if( cv::waitKey( 10 ) == VK_ESCAPE ){
			break;
		}
	}

	pCoreWindow->UnsubscribePointerMoved( hMoved );
	CloseHandle( reinterpret_cast<HANDLE>( hMoved ) );
	pCoreWindow->UnsubscribePointerEntered( hEntered );
	CloseHandle( reinterpret_cast<HANDLE>( hEntered ) );
	pCoreWindow->UnsubscribePointerExited( hExited );
	CloseHandle( reinterpret_cast<HANDLE>( hExited ) );
	SafeRelease( pCoreWindow );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	cv::destroyAllWindows();

	return 0;
}

