
#pragma once

#include <mfidl.h>

class CustomStreamSink : public IMFStreamSink
{
private:
    ULONG                m_nRefCount;
    DWORD                m_dwStreamSinkIdentifier;
    IMFMediaSink*        m_pSink; // Release
    IMFMediaEventQueue*  m_pEventQueue; // Release
    IMFMediaTypeHandler* m_pHandler; // Release

    CustomStreamSink(IMFMediaSink* pSink, DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType);
    ~CustomStreamSink();

public:
    static HRESULT CreateInstance(CustomStreamSink** ppStream, IMFMediaSink* pSink, DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType);

    // IUnknown Methods
    ULONG   AddRef();
    ULONG   Release();
    HRESULT QueryInterface(REFIID iid, void** ppv);

    // IMFMediaEventGenerator Methods
    HRESULT BeginGetEvent(IMFAsyncCallback* pCallback, IUnknown* punkState);
    HRESULT EndGetEvent(IMFAsyncResult* pResult, IMFMediaEvent** ppEvent);
    HRESULT GetEvent(DWORD dwFlags, IMFMediaEvent** ppEvent);
    HRESULT QueueEvent(MediaEventType met, REFGUID guidExtendedType, HRESULT hrStatus, PROPVARIANT const* pvValue);

    // IMFStreamSink Methods
    HRESULT Flush();
    HRESULT GetIdentifier(DWORD* pdwIdentifier);
    HRESULT GetMediaSink(IMFMediaSink** ppMediaSink);
    HRESULT GetMediaTypeHandler(IMFMediaTypeHandler** ppHandler);
    HRESULT PlaceMarker(MFSTREAMSINK_MARKER_TYPE eMarkerType, PROPVARIANT const* pvarMarkerValue, PROPVARIANT const* pvarContextValue);
    HRESULT ProcessSample(IMFSample* pSample);

    // CustomMediaSink Events
    void Shutdown();
    void Start(MFTIME hnsSystemTime, LONGLONG llClockStartOffset);
    void Stop(MFTIME hnsSystemTime);
    void Pause(MFTIME hnsSystemTime);
    void Restart(MFTIME hnsSystemTime);
    void SetRate(MFTIME hnsSystemTime, float flRate);
};
