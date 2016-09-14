//------------------------------------------------------------------------------
// <copyright file="KinectAudioStream.cpp" company="Microsoft">
//   Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
// <summary>
//   Implementation for KinectAudioStream methods.
//   KinectAudioStream wraps the Kinect audio stream and does proper format
//   conversion during read.
// </summary>
//------------------------------------------------------------------------------

//#include "stdafx.h"
#include "KinectAudioStream.h"

#include <stdio.h>

/// <summary>
/// KinectAudioStream constructor.
/// </summary>
KinectAudioStream::KinectAudioStream(IStream *p32BitAudio) :
    m_cRef(1),
    m_p32BitAudio(p32BitAudio),
    m_SpeechActive(false)

{
}

/// <summary>
/// SetSpeechState method
/// </summary>
void KinectAudioStream::SetSpeechState(bool state)
{
    m_SpeechActive = state;
}

/////////////////////////////////////////////
// IStream methods
__pragma(warning(push))
__pragma(warning(disable:6101)) // Suppress warning about returning uninitialized memory *pBuffer. It is written correctly.
__pragma(warning(disable:6386)) // Suppress warning about buffer overrun while writing to 'pByteBuffer'. There are no overruns.
STDMETHODIMP KinectAudioStream::Read(
    _Out_writes_bytes_to_(cbBuffer, *pcbRead) void *pBuffer, 
    _In_ ULONG cbBuffer, 
    _Out_opt_ ULONG *pcbRead)
{
    if (pBuffer == NULL || pcbRead == NULL || cbBuffer == 0)
    {
        return E_INVALIDARG;
    }

    HRESULT hr = S_OK;

    // 32bit -> 16bit conversion support
    INT16* p16Buffer = (INT16*)pBuffer;
    int factor = sizeof(float)/sizeof(INT16);

    // 32 bit read support
    float* p32Buffer = new float[cbBuffer/factor];
    byte* pByteBuffer = (byte*)p32Buffer;
    ULONG bytesRead = 0;
    ULONG bytesRemaining = cbBuffer * factor;

    // Speech reads at high frequency - this slows down the process
    int sleepDuration = 50;

    // Speech Service isn't tolerant of partial reads
    while (bytesRemaining > 0)
    {
        // Stop returning Audio data if Speech isn't active
        if (!m_SpeechActive)
        {
            *pcbRead = 0;
            hr = S_FALSE;
            goto exit;
        }

        // bytesRead will always be a multiple of 4 ( = sizeof(float))
        hr = m_p32BitAudio->Read(pByteBuffer, bytesRemaining, &bytesRead);
        pByteBuffer += bytesRead;
        bytesRemaining -= bytesRead;

        // All Audio buffers drained - wait for buffers to fill
        if (bytesRemaining != 0)
        {
            Sleep(sleepDuration);
        }
    }

    // Convert float value [-1,1] to int16 [SHRT_MIN, SHRT_MAX] and copy to output butter
    for (UINT i = 0; i < cbBuffer/factor; i++)
    {
        float sample = p32Buffer[i];

        // Make sure it is in the range [-1, +1]
        if (sample > 1.0f)
        {
            sample = 1.0f;
        }
        else if (sample < -1.0f)
        {
            sample = -1.0f;
        }

        // Scale float to the range (SHRT_MIN, SHRT_MAX] and then
        // convert to 16-bit signed with proper rounding
        float sampleScaled = sample * (float)SHRT_MAX;
        p16Buffer[i] = (sampleScaled > 0.f) ? (INT16)(sampleScaled + 0.5f) : (INT16)(sampleScaled - 0.5f);
    }

    *pcbRead = cbBuffer;

exit:
    delete[] p32Buffer;
    return hr;
}
__pragma(warning(pop))

STDMETHODIMP KinectAudioStream::Write(_In_reads_bytes_(cb) const void *pv, _In_ ULONG cb, _Out_opt_ ULONG *pcbWritten)
{
    return E_NOTIMPL;
}

STDMETHODIMP KinectAudioStream::Seek(LARGE_INTEGER /* dlibMove */, DWORD /* dwOrigin */, _Out_opt_ ULARGE_INTEGER *plibNewPosition)
{
    // Speech seeks and expects a seek implementation - but the NUIAudio stream doesn't support seeking
    if (plibNewPosition != NULL)
    {
        plibNewPosition->QuadPart = 0;
    }

    return S_OK;
}

STDMETHODIMP KinectAudioStream::SetSize(ULARGE_INTEGER)
{
    return E_NOTIMPL;
}

STDMETHODIMP KinectAudioStream::CopyTo(_In_ IStream *, ULARGE_INTEGER, _Out_opt_ ULARGE_INTEGER *, _Out_opt_ ULARGE_INTEGER *)
{
    return E_NOTIMPL;
}

STDMETHODIMP KinectAudioStream::Commit(DWORD)
{
    return E_NOTIMPL;
}

STDMETHODIMP KinectAudioStream::Revert()
{
    return E_NOTIMPL;
}

STDMETHODIMP KinectAudioStream::LockRegion(ULARGE_INTEGER, ULARGE_INTEGER, DWORD)
{
    return E_NOTIMPL;
}

STDMETHODIMP KinectAudioStream::UnlockRegion(ULARGE_INTEGER, ULARGE_INTEGER, DWORD)
{
    return E_NOTIMPL;
}

STDMETHODIMP KinectAudioStream::Stat(__RPC__out STATSTG *, DWORD)
{
    return E_NOTIMPL;
}

STDMETHODIMP KinectAudioStream::Clone(__RPC__deref_out_opt IStream **)
{
    return E_NOTIMPL;
}
