// Gesture.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
// This source code is licensed under the MIT license. Please see the License in License.txt.
// "This is preliminary software and/or hardware and APIs are preliminary and subject to change."
//

#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>
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
	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource( &pColorSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	IBodyFrameSource* pBodySource;
	hResult = pSensor->get_BodyFrameSource( &pBodySource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return -1;
	}



	// Reader
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader( &pColorReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IBodyFrameReader* pBodyReader;
	hResult = pBodySource->OpenReader( &pBodyReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
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
	cv::Mat bodyMat( height / 2, width / 2, CV_8UC4 );
	cv::namedWindow( "Gesture" );

	// Color Table
	cv::Vec3b color[BODY_COUNT];
	color[0] = cv::Vec3b( 255, 0, 0 );
	color[1] = cv::Vec3b( 0, 255, 0 );
	color[2] = cv::Vec3b( 0, 0, 255 );
	color[3] = cv::Vec3b( 255, 255, 0 );
	color[4] = cv::Vec3b( 255, 0, 255 );
	color[5] = cv::Vec3b( 0, 255, 255 );

	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	IVisualGestureBuilderFrameSource* pGestureSource[BODY_COUNT];
	IVisualGestureBuilderFrameReader* pGestureReader[BODY_COUNT];
	for( int count = 0; count < BODY_COUNT; count++ ){
		// Source
		hResult = CreateVisualGestureBuilderFrameSource( pSensor, 0, &pGestureSource[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : CreateVisualGestureBuilderFrameSource()" << std::endl;
			return -1;
		}

		// Reader
		hResult = pGestureSource[count]->OpenReader( &pGestureReader[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IVisualGestureBuilderFrameSource::OpenReader()" << std::endl;
			return -1;
		}
	}

	// Create Gesture Dataase from File (*.gba)
	IVisualGestureBuilderDatabase* pGestureDatabase;
	hResult = CreateVisualGestureBuilderDatabaseInstanceFromFile( L"HandUp.gba"/*L"Swipe.gba"*/, &pGestureDatabase );
	if( FAILED( hResult ) ){
		std::cerr << "Error : CreateVisualGestureBuilderDatabaseInstanceFromFile()" << std::endl;
		return -1;
	}

	// Add Gesture
	UINT gestureCount = 0;
	hResult = pGestureDatabase->get_AvailableGesturesCount( &gestureCount );
	if( FAILED( hResult ) || !gestureCount ){
		std::cerr << "Error : IVisualGestureBuilderDatabase::get_AvailableGesturesCount()" << std::endl;
		return -1;
	}

	IGesture* pGesture;
	hResult = pGestureDatabase->get_AvailableGestures( gestureCount, &pGesture );
	if( SUCCEEDED( hResult ) && pGesture != nullptr ){
		for( int count = 0; count < BODY_COUNT; count++ ){
			hResult = pGestureSource[count]->AddGesture( pGesture );
			if( FAILED( hResult ) ){
				std::cerr << "Error : IVisualGestureBuilderFrameSource::AddGesture()" << std::endl;
				return -1;
			}

			hResult = pGestureSource[count]->SetIsEnabled( pGesture, true );
			if( FAILED( hResult ) ){
				std::cerr << "Error : IVisualGestureBuilderFrameSource::SetIsEnabled()" << std::endl;
				return -1;
			}
		}
	}

	while( 1 ){
		// Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame( &pColorFrame );
		if( SUCCEEDED( hResult ) ){
			hResult = pColorFrame->CopyConvertedFrameDataToArray( bufferSize, reinterpret_cast<BYTE*>( bufferMat.data ), ColorImageFormat::ColorImageFormat_Bgra );
			if( SUCCEEDED( hResult ) ){
				cv::resize( bufferMat, bodyMat, cv::Size(), 0.5, 0.5 );
			}
		}
		SafeRelease( pColorFrame );

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
						// Joint
						Joint joint[JointType::JointType_Count];
						hResult = pBody[count]->GetJoints( JointType::JointType_Count, joint );
						if( SUCCEEDED( hResult ) ){
							for( int type = 0; type < JointType::JointType_Count; type++ ){
								ColorSpacePoint colorSpacePoint = { 0 };
								pCoordinateMapper->MapCameraPointToColorSpace( joint[type].Position, &colorSpacePoint );
								int x = static_cast<int>( colorSpacePoint.X );
								int y = static_cast<int>( colorSpacePoint.Y );
								if( ( x >= 0 ) && ( x < width ) && ( y >= 0 ) && ( y < height ) ){
									cv::circle( bufferMat, cv::Point( x, y ), 5, static_cast< cv::Scalar >( color[count] ), -1, CV_AA );
								}
							}
						}

						// Set TrackingID to Detect Gesture
						UINT64 trackingId = _UI64_MAX;
						hResult = pBody[count]->get_TrackingId( &trackingId );
						if( SUCCEEDED( hResult ) ){
							pGestureSource[count]->put_TrackingId( trackingId );
						}
					}
				}
				cv::resize( bufferMat, bodyMat, cv::Size(), 0.5, 0.5 );
			}
			for( int count = 0; count < BODY_COUNT; count++ ){
				SafeRelease( pBody[count] );
			}
		}
		SafeRelease( pBodyFrame );

		// Detect Gesture
		std::system( "cls" );
		for( int count = 0; count < BODY_COUNT; count++ ){
			IVisualGestureBuilderFrame* pGestureFrame = nullptr;
			hResult = pGestureReader[count]->CalculateAndAcquireLatestFrame( &pGestureFrame );
			if( SUCCEEDED( hResult ) && pGestureFrame != nullptr ){
				BOOLEAN bGestureTracked = false;
				hResult = pGestureFrame->get_IsTrackingIdValid( &bGestureTracked );
				if( SUCCEEDED( hResult ) && bGestureTracked ){
					// Discrete Gesture (Sample HandUp.gba is Action to Hand Up above the head.)
					IDiscreteGestureResult* pGestureResult = nullptr;
					hResult = pGestureFrame->get_DiscreteGestureResult( pGesture, &pGestureResult );
					if( SUCCEEDED( hResult ) && pGestureResult != nullptr ){
						BOOLEAN bDetected = false;
						hResult = pGestureResult->get_Detected( &bDetected );
						if( SUCCEEDED( hResult ) && bDetected ){
							std::cout << "Detected Gesture" << std::endl;
						}
					}

					/*// Continuous Gesture (Sample Swipe.gba is Action to Swipe the hand in horizontal direction.)
					IContinuousGestureResult* pGestureResult = nullptr;
					hResult = pGestureFrame->get_ContinuousGestureResult( pGesture, &pGestureResult );
					if( SUCCEEDED( hResult ) && pGestureResult != nullptr ){
						float progress = 0.0f;
						hResult = pGestureResult->get_Progress( &progress );
						if( SUCCEEDED( hResult ) ){
							std::cout << "Progress: " + std::to_string( progress ) << std::endl;
						}
					}*/

					SafeRelease( pGestureResult );
				}
			}
			SafeRelease( pGestureFrame );
		}

		cv::imshow( "Gesture", bodyMat );

		if( cv::waitKey( 10 ) == VK_ESCAPE ){
			break;
		}
	}

	SafeRelease( pColorSource );
	SafeRelease( pBodySource );
	SafeRelease( pColorReader );
	SafeRelease( pBodyReader );
	SafeRelease( pDescription );
	SafeRelease( pCoordinateMapper );
	for( int count = 0; count < BODY_COUNT; count++ ){
		SafeRelease( pGestureSource[count] );
		SafeRelease( pGestureReader[count] );
	}
	SafeRelease( pGestureDatabase );
	SafeRelease( pGesture );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	cv::destroyAllWindows();

	return 0;
}

