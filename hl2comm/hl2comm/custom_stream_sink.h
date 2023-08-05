
#pragma once

#include <mfidl.h>
#include "custom_sink_hook.h"

// OK
class CustomStreamSink : public IMFStreamSink
{
    friend class CustomMediaSink;

private:
    ULONG                m_nRefCount;
    CRITICAL_SECTION     m_critSec;
    bool                 m_isShutdown;
    DWORD                m_dwStreamSinkIdentifier;
    IMFMediaSink*        m_pSink;
    IMFMediaEventQueue*  m_pEventQueue;
    IMFMediaTypeHandler* m_pHandler;
    CustomStreamHook*    m_pHook;

    static HRESULT CreateInstance(CustomStreamSink** ppStream, IMFMediaSink* pSink, DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType, CustomSinkHook* pHook);

    CustomStreamSink(IMFMediaSink* pSink, DWORD dwStreamSinkIdentifier);
    ~CustomStreamSink();

    HRESULT Initialize(DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType, CustomSinkHook* pHook);

public:
    // IUnknown Methods
    ULONG   AddRef();
    ULONG   Release();
    HRESULT QueryInterface(REFIID iid, void** ppv);

    // IMFMediaEventGenerator Methods
    HRESULT BeginGetEvent(IMFAsyncCallback* pCallback, IUnknown* punkState);
    HRESULT EndGetEvent(IMFAsyncResult* pResult, IMFMediaEvent** ppEvent);
    HRESULT GetEvent(DWORD dwFlags, IMFMediaEvent** ppEvent);
    HRESULT QueueEvent(MediaEventType met, REFGUID guidExtendedType, HRESULT hrStatus, const PROPVARIANT* pvValue);

    // IMFStreamSink Methods
    HRESULT Flush();
    HRESULT GetIdentifier(DWORD* pdwIdentifier);
    HRESULT GetMediaSink(IMFMediaSink** ppMediaSink);
    HRESULT GetMediaTypeHandler(IMFMediaTypeHandler** ppHandler);
    HRESULT PlaceMarker(MFSTREAMSINK_MARKER_TYPE eMarkerType, PROPVARIANT const* pvarMarkerValue, PROPVARIANT const* pvarContextValue);
    HRESULT ProcessSample(IMFSample* pSample);

    // CustomMediaSink Events
    void Shutdown();
    void Detach();
    void Start(MFTIME hnsSystemTime, LONGLONG llClockStartOffset);
    void Stop(MFTIME hnsSystemTime);
    void Pause(MFTIME hnsSystemTime);
    void Restart(MFTIME hnsSystemTime);
    void SetRate(MFTIME hnsSystemTime, float flRate);
};
