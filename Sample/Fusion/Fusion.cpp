// Fusion.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
// This source code is licensed under the MIT license. Please see the License in License.txt.
// "This is preliminary software and/or hardware and APIs are preliminary and subject to change."
//

#include "stdafx.h"
#include <Windows.h>
#include <Kinect.h>
#include <Kinect.h>
#include <NuiKinectFusionApi.h>
// Quote from Kinect for Windows SDK v2.0 Developer Preview - Samples/Native/KinectFusionExplorer-D2D, and Partial Modification
// KinectFusionHelper is: Copyright (c) Microsoft Corporation. All rights reserved.
#include "KinectFusionHelper.h"
#include <opencv2/opencv.hpp>


template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != NULL ){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

void ResetReconstruction( INuiFusionReconstruction* pReconstruction, Matrix4* pMat )
{
	std::cout << "Reset Reconstruction" << std::endl;
	SetIdentityMatrix( *pMat );
	pReconstruction->ResetReconstruction( pMat, nullptr );
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
	IDepthFrameSource* pDepthSource;
	hResult = pSensor->get_DepthFrameSource( &pDepthSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IDepthFrameReader* pDepthReader;
	hResult = pDepthSource->OpenReader( &pDepthReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	IFrameDescription* pDescription;
	hResult = pDepthSource->get_FrameDescription( &pDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int width = 0;
	int height = 0;
	pDescription->get_Width( &width ); // 512
	pDescription->get_Height( &height ); // 424
	unsigned int bufferSize = width * height * sizeof( unsigned short );

	cv::Mat bufferMat( height, width, CV_16UC1 );
	cv::Mat depthMat( height, width, CV_8UC1 );
	cv::namedWindow( "Depth" );

	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	// Create Reconstruction
	NUI_FUSION_RECONSTRUCTION_PARAMETERS reconstructionParameter;
	reconstructionParameter.voxelsPerMeter = 256;
	reconstructionParameter.voxelCountX = 512;
	reconstructionParameter.voxelCountY = 384;
	reconstructionParameter.voxelCountZ = 512;

	Matrix4 worldToCameraTransform;
	SetIdentityMatrix( worldToCameraTransform );
	INuiFusionReconstruction* pReconstruction;
	hResult = NuiFusionCreateReconstruction( &reconstructionParameter, NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP, -1, &worldToCameraTransform, &pReconstruction );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiFusionCreateReconstruction()" << std::endl;
		return -1;
	}

	// Create Image Frame
	// Depth
	NUI_FUSION_IMAGE_FRAME* pDepthFloatImageFrame;
	hResult = NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE_FLOAT, width, height, nullptr, &pDepthFloatImageFrame );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiFusionCreateImageFrame( FLOAT )" << std::endl;
		return -1;
	}

	// SmoothDepth
	NUI_FUSION_IMAGE_FRAME* pSmoothDepthFloatImageFrame;
	hResult = NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE_FLOAT, width, height, nullptr, &pSmoothDepthFloatImageFrame );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiFusionCreateImageFrame( FLOAT )" << std::endl;
		return -1;
	}

	// Point Cloud
	NUI_FUSION_IMAGE_FRAME* pPointCloudImageFrame;
	hResult = NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, width, height, nullptr, &pPointCloudImageFrame );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiFusionCreateImageFrame( POINT_CLOUD )" << std::endl;
		return -1;
	}

	// Surface
	NUI_FUSION_IMAGE_FRAME* pSurfaceImageFrame;
	hResult = NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE_COLOR, width, height, nullptr, &pSurfaceImageFrame );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiFusionCreateImageFrame( COLOR )" << std::endl;
		return -1;
	}

	// Normal
	NUI_FUSION_IMAGE_FRAME* pNormalImageFrame;
	hResult = NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE_COLOR, width, height, nullptr, &pNormalImageFrame );
	if( FAILED( hResult ) ){
		std::cerr << "Error : NuiFusionCreateImageFrame( COLOR )" << std::endl;
		return -1;
	}

	cv::namedWindow( "Surface" );
	cv::namedWindow( "Normal" );

	while( 1 ){
		// Frame
		IDepthFrame* pDepthFrame = nullptr;
		hResult = pDepthReader->AcquireLatestFrame( &pDepthFrame );

		if( SUCCEEDED( hResult ) ){
			hResult = pDepthFrame->AccessUnderlyingBuffer( &bufferSize, reinterpret_cast<UINT16**>( &bufferMat.data ) );
			if( SUCCEEDED( hResult ) ){
				bufferMat.convertTo( depthMat, CV_8U, -255.0f / 8000.0f, 255.0f );
				hResult = pReconstruction->DepthToDepthFloatFrame( reinterpret_cast<UINT16*>( bufferMat.data ), width * height * sizeof( UINT16 ), pDepthFloatImageFrame, NUI_FUSION_DEFAULT_MINIMUM_DEPTH/* 0.5[m] */, NUI_FUSION_DEFAULT_MAXIMUM_DEPTH/* 8.0[m] */, true );
				if( FAILED( hResult ) ){
					std::cerr << "Error :INuiFusionReconstruction::DepthToDepthFloatFrame()" << std::endl;
					return -1;
				}
			}
		}
		SafeRelease( pDepthFrame );

		// Smooting Depth Image Frame
		hResult = pReconstruction->SmoothDepthFloatFrame( pDepthFloatImageFrame, pSmoothDepthFloatImageFrame, 1, 0.04f );
		if( FAILED( hResult ) ){
			std::cerr << "Error :INuiFusionReconstruction::SmoothDepthFloatFrame" << std::endl;
			return -1;
		}

		// Reconstruction Process
		pReconstruction->GetCurrentWorldToCameraTransform( &worldToCameraTransform );
		hResult = pReconstruction->ProcessFrame( pSmoothDepthFloatImageFrame, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT, nullptr, &worldToCameraTransform );
		if( FAILED( hResult ) ){
			static int errorCount = 0;
			errorCount++;
			if( errorCount >= 100 ) {
				errorCount = 0;
				ResetReconstruction( pReconstruction, &worldToCameraTransform );
			}
		}

		// Calculate Point Cloud
		hResult = pReconstruction->CalculatePointCloud( pPointCloudImageFrame, &worldToCameraTransform );
		if( FAILED( hResult ) ){
			std::cerr << "Error : CalculatePointCloud" << std::endl;
			return -1;
		}

		// Shading Point Clouid
		Matrix4 worldToBGRTransform = { 0.0f };
		worldToBGRTransform.M11 = reconstructionParameter.voxelsPerMeter / reconstructionParameter.voxelCountX;
		worldToBGRTransform.M22 = reconstructionParameter.voxelsPerMeter / reconstructionParameter.voxelCountY;
		worldToBGRTransform.M33 = reconstructionParameter.voxelsPerMeter / reconstructionParameter.voxelCountZ;
		worldToBGRTransform.M41 = 0.5f;
		worldToBGRTransform.M42 = 0.5f;
		worldToBGRTransform.M43 = 0.0f;
		worldToBGRTransform.M44 = 1.0f;

		hResult = NuiFusionShadePointCloud( pPointCloudImageFrame, &worldToCameraTransform, &worldToBGRTransform, pSurfaceImageFrame, pNormalImageFrame );
		if( FAILED( hResult ) ){
			std::cerr << "Error : NuiFusionShadePointCloud" << std::endl;
			return -1;
		}

		cv::Mat surfaceMat( height, width, CV_8UC4, pSurfaceImageFrame->pFrameBuffer->pBits );
		cv::Mat normalMat( height, width, CV_8UC4, pNormalImageFrame->pFrameBuffer->pBits );

		cv::imshow( "Depth", depthMat );
		cv::imshow( "Surface", surfaceMat );
		cv::imshow( "Normal", normalMat );

		int key = cv::waitKey( 30 );
		if( key == VK_ESCAPE ){
			break;
		}
		else if( key == 'r' ){
			ResetReconstruction( pReconstruction, &worldToCameraTransform );
		}
	}

	SafeRelease( pDepthSource );
	SafeRelease( pDepthReader );
	SafeRelease( pDescription );
	SafeRelease( pCoordinateMapper );
	SafeRelease( pReconstruction );
	NuiFusionReleaseImageFrame( pDepthFloatImageFrame );
	NuiFusionReleaseImageFrame( pSmoothDepthFloatImageFrame );
	NuiFusionReleaseImageFrame( pPointCloudImageFrame );
	NuiFusionReleaseImageFrame( pSurfaceImageFrame );
	NuiFusionReleaseImageFrame( pNormalImageFrame );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	cv::destroyAllWindows();

	return 0;
}

