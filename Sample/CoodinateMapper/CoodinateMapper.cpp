// CoodinateMapper.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
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
	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource( &pColorSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return -1;
	}

	IDepthFrameSource* pDepthSource;
	hResult = pSensor->get_DepthFrameSource( &pDepthSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return -1;
	}

	// Reader
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader( &pColorReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	IDepthFrameReader* pDepthReader;
	hResult = pDepthSource->OpenReader( &pDepthReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	// Description
	IFrameDescription* pColorDescription;
	hResult = pColorSource->get_FrameDescription( &pColorDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int colorWidth = 0;
	int colorHeight = 0;
	pColorDescription->get_Width( &colorWidth ); // 1920
	pColorDescription->get_Height( &colorHeight ); // 1080
	unsigned int colorBufferSize = colorWidth * colorHeight * 4 * sizeof( unsigned char );

	cv::Mat colorBufferMat( colorHeight, colorWidth, CV_8UC4 );
	cv::Mat colorMat( colorHeight / 2, colorWidth / 2, CV_8UC4 );
	cv::namedWindow( "Color" );

	IFrameDescription* pDepthDescription;
	hResult = pDepthSource->get_FrameDescription( &pDepthDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}

	int depthWidth = 0;
	int depthHeight = 0;
	pDepthDescription->get_Width( &depthWidth ); // 512
	pDepthDescription->get_Height( &depthHeight ); // 424
	unsigned int depthBufferSize = depthWidth * depthHeight * sizeof( unsigned short );

	cv::Mat depthBufferMat( depthHeight, depthWidth, CV_16UC1 );
	cv::Mat depthMat( depthHeight, depthWidth, CV_8UC1 );
	cv::namedWindow( "Depth" );

	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		return -1;
	}

	cv::Mat coordinateMapperMat( depthHeight, depthWidth, CV_8UC4 );
	cv::namedWindow( "CoordinateMapper" );

	unsigned short minDepth, maxDepth;
	pDepthSource->get_DepthMinReliableDistance( &minDepth );
	pDepthSource->get_DepthMaxReliableDistance( &maxDepth );

	while( 1 ){
		// Color Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame( &pColorFrame );
		if( SUCCEEDED( hResult ) ){
			hResult = pColorFrame->CopyConvertedFrameDataToArray( colorBufferSize, reinterpret_cast<BYTE*>( colorBufferMat.data ), ColorImageFormat::ColorImageFormat_Bgra );
			if( SUCCEEDED( hResult ) ){
				cv::resize( colorBufferMat, colorMat, cv::Size(), 0.5, 0.5 );
			}
		}
		//SafeRelease( pColorFrame );

		// Depth Frame
		IDepthFrame* pDepthFrame = nullptr;
		hResult = pDepthReader->AcquireLatestFrame( &pDepthFrame );
		if( SUCCEEDED( hResult ) ){
			hResult = pDepthFrame->AccessUnderlyingBuffer( &depthBufferSize, reinterpret_cast<UINT16**>( &depthBufferMat.data ) );
			if( SUCCEEDED( hResult ) ){
				depthBufferMat.convertTo( depthMat, CV_8U, -255.0f / 8000.0f, 255.0f );
			}
		}
		//SafeRelease( pDepthFrame );

		// Mapping (Depth to Color)
		if( SUCCEEDED( hResult ) ){
			std::vector<ColorSpacePoint> colorSpacePoints( depthWidth * depthHeight );
			hResult = pCoordinateMapper->MapDepthFrameToColorSpace( depthWidth * depthHeight, reinterpret_cast<UINT16*>( depthBufferMat.data ), depthWidth * depthHeight, &colorSpacePoints[0] );
			if( SUCCEEDED( hResult ) ){
				coordinateMapperMat = cv::Scalar( 0, 0, 0, 0 );
				for( int y = 0; y < depthHeight; y++ ){
					for( int x = 0; x < depthWidth; x++ ){
						unsigned int index = y * depthWidth + x;
						ColorSpacePoint point = colorSpacePoints[index];
						int colorX = static_cast<int>( std::floor( point.X + 0.5 ) );
						int colorY = static_cast<int>( std::floor( point.Y + 0.5 ) );
						unsigned short depth = depthBufferMat.at<unsigned short>( y, x );
						if( ( colorX >= 0 ) && ( colorX < colorWidth ) && ( colorY >= 0 ) && ( colorY < colorHeight )/* && ( depth >= minDepth ) && ( depth <= maxDepth )*/ ){
							coordinateMapperMat.at<cv::Vec4b>( y, x ) = colorBufferMat.at<cv::Vec4b>( colorY, colorX );
						}
					}
				}
			}
		}

		SafeRelease( pColorFrame );
		SafeRelease( pDepthFrame );

		cv::imshow( "Color", colorMat );
		cv::imshow( "Depth", depthMat );
		cv::imshow( "CoordinateMapper", coordinateMapperMat );

		if( cv::waitKey( 30 ) == VK_ESCAPE ){
			break;
		}
	}

	SafeRelease( pColorSource );
	SafeRelease( pDepthSource );
	SafeRelease( pColorReader );
	SafeRelease( pDepthReader );
	SafeRelease( pColorDescription );
	SafeRelease( pDepthDescription );
	SafeRelease( pCoordinateMapper );
	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( pSensor );
	cv::destroyAllWindows();

	return 0;
}

