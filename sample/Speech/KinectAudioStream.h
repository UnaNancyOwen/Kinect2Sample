//------------------------------------------------------------------------------
// <copyright file="KinectAudioStream.h" company="Microsoft">
//   Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
// <summary>
//   Includes common headers and defines following classes:
//     - KinectAudioStream: IStream implementation to convert 32Bit Kinect Stream to 16Bit for Speech.
// </summary>
//------------------------------------------------------------------------------
#pragma once

// For WAVEFORMATEX
#include <Windows.h>
#include <mmreg.h>
#include <Shlobj.h>

/// <summary>
/// Asynchronous IStream implementation that captures audio data from Kinect audio sensor in a background thread
/// and lets clients read captured audio from any thread.
/// </summary>
class KinectAudioStream : public IStream
{
public:
    /////////////////////////////////////////////
    // KinectAudioStream methods

    /// <summary>
    /// KinectAudioStream constructor.
    /// </summary>
    KinectAudioStream(IStream *p32BitAudioStream);

    /// <summary>
    /// SetSpeechState method
    /// </summary>
    void SetSpeechState(bool state);

    /////////////////////////////////////////////
    // IUnknown methods
    STDMETHODIMP_(ULONG) AddRef() { return InterlockedIncrement(&m_cRef); }
    STDMETHODIMP_(ULONG) Release()
    {
        UINT ref = InterlockedDecrement(&m_cRef);
        if (ref == 0)
        {
            delete this;
        }
        return ref;
    }
    STDMETHODIMP QueryInterface(REFIID riid, void **ppv)
    {
        if (riid == IID_IUnknown)
        {
            AddRef();
            *ppv = (IUnknown*)this;
            return S_OK;
        }
        else if (riid == IID_IStream)
        {
            AddRef();
            *ppv = (IStream*)this;
            return S_OK;
        }
        else
        {
            return E_NOINTERFACE;
        }
    }

    /////////////////////////////////////////////
    // IStream methods
    STDMETHODIMP Read(_Out_writes_bytes_to_(cbBuffer, *pcbRead) void *pBuffer , _In_ ULONG cbBuffer, _Out_opt_ ULONG *pcbRead);
    STDMETHODIMP Write(_In_reads_bytes_(cb) const void *pv, _In_ ULONG cb, _Out_opt_ ULONG *pcbWritten);
    STDMETHODIMP Seek(LARGE_INTEGER, DWORD, _Out_opt_ ULARGE_INTEGER *);
    STDMETHODIMP SetSize(ULARGE_INTEGER);
    STDMETHODIMP CopyTo(_In_ IStream *, ULARGE_INTEGER, _Out_opt_ ULARGE_INTEGER *, _Out_opt_ ULARGE_INTEGER *);
    STDMETHODIMP Commit(DWORD);
    STDMETHODIMP Revert();
    STDMETHODIMP LockRegion(ULARGE_INTEGER, ULARGE_INTEGER, DWORD);
    STDMETHODIMP UnlockRegion(ULARGE_INTEGER, ULARGE_INTEGER, DWORD);
    STDMETHODIMP Stat(__RPC__out STATSTG *, DWORD);
    STDMETHODIMP Clone(__RPC__deref_out_opt IStream **);

private:
    
    // Number of references to this object
    UINT                    m_cRef; 
    IStream*                m_p32BitAudio;
    bool                    m_SpeechActive;
};