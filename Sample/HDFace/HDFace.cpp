// HDFace.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
// This source code is licensed under the MIT license. Please see the License in License.txt.
// "This is preliminary software and/or hardware and APIs are preliminary and subject to change."
//

#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <Kinect.Face.h>
#include <opencv2/opencv.hpp>
#include <bitset>

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
	cv::Mat faceMat( height / 2, width / 2, CV_8UC4 );
	cv::namedWindow( "HDFace" );

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

	IHighDefinitionFaceFrameSource* pHDFaceSource[BODY_COUNT];
	IHighDefinitionFaceFrameReader* pHDFaceReader[BODY_COUNT];
	IFaceModelBuilder* pFaceModelBuilder[BODY_COUNT];
	bool produce[BODY_COUNT] = { false };
	IFaceAlignment* pFaceAlignment[BODY_COUNT];
	IFaceModel* pFaceModel[BODY_COUNT];
	std::vector<std::vector<float>> deformations( BODY_COUNT, std::vector<float>( FaceShapeDeformations::FaceShapeDeformations_Count ) );
	for( int count = 0; count < BODY_COUNT; count++ ){
		// Source
		hResult = CreateHighDefinitionFaceFrameSource( pSensor, &pHDFaceSource[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : CreateHighDefinitionFaceFrameSource()" << std::endl;
			return -1;
		}

		// Reader
		hResult = pHDFaceSource[count]->OpenReader( &pHDFaceReader[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IHighDefinitionFaceFrameSource::OpenReader()" << std::endl;
			return -1;
		}

		// Open Face Model Builder
		hResult = pHDFaceSource[count]->OpenModelBuilder( FaceModelBuilderAttributes::FaceModelBuilderAttributes_None, &pFaceModelBuilder[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : IHighDefinitionFaceFrameSource::OpenModelBuilder()" << std::endl;
			return -1;
		}

		// Start Collection Face Data
		hResult = pFaceModelBuilder[count]->BeginFaceDataCollection();
		if( FAILED( hResult ) ){
			std::cerr << "Error : IFaceModelBuilder::BeginFaceDataCollection()" << std::endl;
			return -1;
		}

		// Create Face Alignment
		hResult = CreateFaceAlignment( &pFaceAlignment[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : CreateFaceAlignment()" << std::endl;
			return -1;
		}

		// Create Face Model
		hResult = CreateFaceModel( 1.0f, FaceShapeDeformations::FaceShapeDeformations_Count, &deformations[count][0], &pFaceModel[count] );
		if( FAILED( hResult ) ){
			std::cerr << "Error : CreateFaceModel()" << std::endl;
			return -1;
		}
	}

	UINT32 vertex = 0;
	hResult = GetFaceModelVertexCount( &vertex ); // 1347
	if( FAILED( hResult ) ){
		std::cerr << "Error : GetFaceModelVertexCount()" << std::endl;
		return -1;
	}

	while( 1 ){
		// Color Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame( &pColorFrame );
		if( SUCCEEDED( hResult ) ){
			hResult = pColorFrame->CopyConvertedFrameDataToArray( bufferSize, reinterpret_cast<BYTE*>( bufferMat.data ), ColorImageFormat::ColorImageFormat_Bgra );
			if( SUCCEEDED( hResult ) ){
				cv::resize( bufferMat, faceMat, cv::Size(), 0.5, 0.5 );
			}
		}
		SafeRelease( pColorFrame );

		// Body Frame
		IBodyFrame* pBodyFrame = nullptr;
		hResult = pBodyReader->AcquireLatestFrame( &pBodyFrame );
		if( SUCCEEDED( hResult ) ){
			IBody* pBody[BODY_COUNT] = { 0 };
			hResult = pBodyFrame->GetAndRefreshBodyData( BODY_COUNT, pBody );
			if( SUCCEEDED( hResult ) ){
				for( int count = 0; count < BODY_COUNT; count++ ){
					BOOLEAN bTrackingIdValid = false;
					hResult = pHDFaceSource[count]->get_IsTrackingIdValid( &bTrackingIdValid );
					if( !bTrackingIdValid ){
						BOOLEAN bTracked = false;
						hResult = pBody[count]->get_IsTracked( &bTracked );
						if( SUCCEEDED( hResult ) && bTracked ){
							/*// Joint
							Joint joint[JointType::JointType_Count];
							hResult = pBody[count]->GetJoints( JointType::JointType_Count, joint );
							if( SUCCEEDED( hResult ) ){
								for( int type = 0; type < JointType::JointType_Count; type++ ){
									ColorSpacePoint colorSpacePoint = { 0 };
									pCoordinateMapper->MapCameraPointToColorSpace( joint[type].Position, &colorSpacePoint );
									int x = static_cast<int>( colorSpacePoint.X );
									int y = static_cast<int>( colorSpacePoint.Y );
									if( ( x >= 0 ) && ( x < width ) && ( y >= 0 ) && ( y < height ) ){
										cv::circle( bufferMat, cv::Point( x, y ), 5, static_cast<cv::Scalar>( color[count] ), -1, CV_AA );
									}
								}
							}*/

							// Set TrackingID to Detect Face
							UINT64 trackingId = _UI64_MAX;
							hResult = pBody[count]->get_TrackingId( &trackingId );
							if( SUCCEEDED( hResult ) ){
								pHDFaceSource[count]->put_TrackingId( trackingId );
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

		// HD Face Frame
		for( int count = 0; count < BODY_COUNT; count++ ){
			IHighDefinitionFaceFrame* pHDFaceFrame = nullptr;
			hResult = pHDFaceReader[count]->AcquireLatestFrame( &pHDFaceFrame );
			if( SUCCEEDED( hResult ) && pHDFaceFrame != nullptr ){
				BOOLEAN bFaceTracked = false;
				hResult = pHDFaceFrame->get_IsFaceTracked( &bFaceTracked );
				if( SUCCEEDED( hResult ) && bFaceTracked ){
					hResult = pHDFaceFrame->GetAndRefreshFaceAlignmentResult( pFaceAlignment[count] );
					if( SUCCEEDED( hResult ) && pFaceAlignment[count] != nullptr ){
						// Face Model Building
						if( !produce[count] ){
							std::system( "cls" );
							FaceModelBuilderCollectionStatus collection;
							hResult = pFaceModelBuilder[count]->get_CollectionStatus( &collection );
							if( collection == FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_Complete ){
								std::cout << "Status : Complete" << std::endl;
								cv::putText( bufferMat, "Status : Complete", cv::Point( 50, 50 ), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>( color[count] ), 2, CV_AA );
								IFaceModelData* pFaceModelData = nullptr;
								hResult = pFaceModelBuilder[count]->GetFaceData( &pFaceModelData );
								if( SUCCEEDED( hResult ) && pFaceModelData != nullptr ){
									hResult = pFaceModelData->ProduceFaceModel( &pFaceModel[count] );
									if( SUCCEEDED( hResult ) && pFaceModel[count] != nullptr ){
										produce[count] = true;
									}
								}
								SafeRelease( pFaceModelData );
							}
							else{
								std::cout << "Status : " << collection << std::endl;
								cv::putText( bufferMat, "Status : " + std::to_string( collection ), cv::Point( 50, 50 ), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>( color[count] ), 2, CV_AA );

								// Collection Status
								if( collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded ){
									std::cout << "Need : Tilted Up Views" << std::endl;
									cv::putText( bufferMat, "Need : Tilted Up Views", cv::Point( 50, 100 ), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>( color[count] ), 2, CV_AA );
								}
								else if( collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_RightViewsNeeded ){
									std::cout << "Need : Right Views" << std::endl;
									cv::putText( bufferMat, "Need : Right Views", cv::Point( 50, 100 ), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>( color[count] ), 2, CV_AA );
								}
								else if( collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_LeftViewsNeeded ){
									std::cout << "Need : Left Views" << std::endl;
									cv::putText( bufferMat, "Need : Left Views", cv::Point( 50, 100 ), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>( color[count] ), 2, CV_AA );
								}
								else if( collection >= FaceModelBuilderCollectionStatus::FaceModelBuilderCollectionStatus_FrontViewFramesNeeded ){
									std::cout << "Need : Front ViewFrames" << std::endl;
									cv::putText( bufferMat, "Need : Front ViewFrames", cv::Point( 50, 100 ), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>( color[count] ), 2, CV_AA );
								}

								// Capture Status
								FaceModelBuilderCaptureStatus capture;
								hResult = pFaceModelBuilder[count]->get_CaptureStatus( &capture );
								switch( capture ){
									case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooFar:
										std::cout << "Error : Face Too Far from Camera" << std::endl;
										cv::putText( bufferMat, "Error : Face Too Far from Camera", cv::Point( 50, 150 ), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>( color[count] ), 2, CV_AA );
										break;
									case FaceModelBuilderCaptureStatus::FaceModelBuilderCaptureStatus_FaceTooNear:
										std::cout << "Error : Face Too Near to Camera" << std::endl;
										cv::putText( bufferMat, "Error : Face Too Near to Camera", cv::Point( 50, 150 ), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>( color[count] ), 2, CV_AA );
										break;
									case FaceModelBuilderCaptureStatus_MovingTooFast:
										std::cout << "Error : Moving Too Fast" << std::endl;
										cv::putText( bufferMat, "Error : Moving Too Fast", cv::Point( 50, 150 ), cv::FONT_HERSHEY_SIMPLEX, 1.0f, static_cast<cv::Scalar>( color[count] ), 2, CV_AA );
										break;
									default:
										break;
								}
							}
						}

						// HD Face Points
						std::vector<CameraSpacePoint> facePoints( vertex );
						hResult = pFaceModel[count]->CalculateVerticesForAlignment( pFaceAlignment[count], vertex, &facePoints[0] );
						if( SUCCEEDED( hResult ) ){
							for( int point = 0; point < vertex; point++ ){
								ColorSpacePoint colorSpacePoint;
								hResult = pCoordinateMapper->MapCameraPointToColorSpace( facePoints[point], &colorSpacePoint );
								if( SUCCEEDED( hResult ) ){
									int x = static_cast<int>( colorSpacePoint.X );
									int y = static_cast<int>( colorSpacePoint.Y );
									if( ( x >= 0 ) && ( x < width ) && ( y >= 0 ) && ( y < height ) ){
										cv::circle( bufferMat, cv::Point( static_cast<int>( colorSpacePoint.X ), static_cast<int>( colorSpacePoint.Y ) ), 5, static_cast<cv::Scalar>( color[count] ), -1, CV_AA );
									}
								}
							}
						}
					}
				}
			}
			SafeRelease( pHDFaceFrame );
		}

		cv::resize( bufferMat, faceMat, cv::Size(), 0.5, 0.5 );
		cv::imshow( "HDFace", faceMat );

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
		SafeRelease( pHDFaceSource[count] );
		SafeRelease( pHDFaceReader[count] );
		SafeRelease( pFaceModelBuilder[count] );
		SafeRelease( pFaceAlignment[count] );
		SafeRelease( pFaceModel[count] );
	}
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	cv::destroyAllWindows();

	return 0;
}

