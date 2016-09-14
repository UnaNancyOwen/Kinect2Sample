//--------------------------------------------------------------------------------------
// KinectJointFilter.h
//
// This file contains Holt Double Exponential Smoothing filter for filtering Joints
//
// Copyright (C) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------

#pragma once

#include <Windows.h>
#include <Kinect.h>
#include <DirectXMath.h>
#include <queue>

namespace Sample
{
	typedef struct _TRANSFORM_SMOOTH_PARAMETERS
	{
		FLOAT   fSmoothing;             // [0..1], lower values closer to raw data
		FLOAT   fCorrection;            // [0..1], lower values slower to correct towards the raw data
		FLOAT   fPrediction;            // [0..n], the number of frames to predict into the future
		FLOAT   fJitterRadius;          // The radius in meters for jitter reduction
		FLOAT   fMaxDeviationRadius;    // The maximum radius in meters that filtered positions are allowed to deviate from raw data
	} TRANSFORM_SMOOTH_PARAMETERS;

	// Holt Double Exponential Smoothing filter
	class FilterDoubleExponentialData
	{
		public:
		DirectX::XMVECTOR m_vRawPosition;
		DirectX::XMVECTOR m_vFilteredPosition;
		DirectX::XMVECTOR m_vTrend;
		DWORD    m_dwFrameCount;
	};

	class FilterDoubleExponential
	{
		public:
		FilterDoubleExponential() { Init(); }
		~FilterDoubleExponential() { Shutdown(); }

		void Init( FLOAT fSmoothing = 0.25f, FLOAT fCorrection = 0.25f, FLOAT fPrediction = 0.25f, FLOAT fJitterRadius = 0.03f, FLOAT fMaxDeviationRadius = 0.05f )
		{
			Reset( fSmoothing, fCorrection, fPrediction, fJitterRadius, fMaxDeviationRadius );
		}

		void Shutdown()
		{
		}

		void Reset( FLOAT fSmoothing = 0.25f, FLOAT fCorrection = 0.25f, FLOAT fPrediction = 0.25f, FLOAT fJitterRadius = 0.03f, FLOAT fMaxDeviationRadius = 0.05f )
		{
			assert( m_pFilteredJoints );
			assert( m_pHistory );

			m_fMaxDeviationRadius = fMaxDeviationRadius; // Size of the max prediction radius Can snap back to noisy data when too high
			m_fSmoothing = fSmoothing;                   // How much smothing will occur.  Will lag when too high
			m_fCorrection = fCorrection;                 // How much to correct back from prediction.  Can make things springy
			m_fPrediction = fPrediction;                 // Amount of prediction into the future to use. Can over shoot when too high
			m_fJitterRadius = fJitterRadius;             // Size of the radius where jitter is removed. Can do too much smoothing when too high

			memset( m_pFilteredJoints, 0, sizeof( DirectX::XMVECTOR ) * JointType_Count );
			memset( m_pHistory, 0, sizeof( FilterDoubleExponentialData ) * JointType_Count );
		}

		void Update( IBody* const pBody );
		void Update( Joint joints[] );

		inline const DirectX::XMVECTOR* GetFilteredJoints() const { return &m_pFilteredJoints[0]; }

		private:
		DirectX::XMVECTOR m_pFilteredJoints[JointType_Count];
		FilterDoubleExponentialData m_pHistory[JointType_Count];
		FLOAT m_fSmoothing;
		FLOAT m_fCorrection;
		FLOAT m_fPrediction;
		FLOAT m_fJitterRadius;
		FLOAT m_fMaxDeviationRadius;

		void Update( Joint joints[], UINT JointID, TRANSFORM_SMOOTH_PARAMETERS smoothingParams );
	};
}