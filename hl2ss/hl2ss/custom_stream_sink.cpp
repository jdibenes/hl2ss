
#include <mfapi.h>
#include <mferror.h>
#include "custom_media_sink.h"
#include "custom_media_type_handler.h"

//-----------------------------------------------------------------------------
// CustomStreamSink Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomStreamSink::CreateInstance(CustomStreamSink** ppStream, IMFMediaSink* pSink, DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType)
{
    *ppStream = new CustomStreamSink(pSink, dwStreamSinkIdentifier, pMediaType);
    return S_OK;
}

// OK
CustomStreamSink::CustomStreamSink(IMFMediaSink *pSink, DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType)
{
    m_nRefCount = 1;
    m_dwStreamSinkIdentifier = dwStreamSinkIdentifier;
    (m_pSink = pSink)->AddRef();
    MFCreateEventQueue(&m_pEventQueue);
    CustomMediaTypeHandler::CreateInstance(&m_pHandler, pMediaType);
}

// OK
CustomStreamSink::~CustomStreamSink()
{
    m_pHandler->Release();
    m_pEventQueue->Release();    
    m_pSink->Release();
}

//-----------------------------------------------------------------------------
// IUnknown Methods
//-----------------------------------------------------------------------------

// OK
ULONG CustomStreamSink::AddRef()
{
    return InterlockedIncrement(&m_nRefCount);
}

// OK
ULONG CustomStreamSink::Release()
{
    ULONG uCount = InterlockedDecrement(&m_nRefCount);
    if (uCount == 0) { delete this; }
    return uCount;
}

// OK
HRESULT CustomStreamSink::QueryInterface(REFIID iid, void** ppv)
{
    if (!ppv) { return E_POINTER; }

    *ppv = NULL;

    if      (iid == IID_IUnknown)               { *ppv = static_cast<IUnknown*>(this); }
    else if (iid == IID_IMFMediaEventGenerator) { *ppv = static_cast<IMFMediaEventGenerator*>(this); }
    else if (iid == IID_IMFStreamSink)          { *ppv = static_cast<IMFStreamSink*>(this); }
    else                                        { return E_NOINTERFACE; }

    AddRef();
    return S_OK;
}

//-----------------------------------------------------------------------------
// IMFMediaEventGenerator Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomStreamSink::BeginGetEvent(IMFAsyncCallback* pCallback, IUnknown* punkState)
{
    return m_pEventQueue->BeginGetEvent(pCallback, punkState);
}

// OK
HRESULT CustomStreamSink::EndGetEvent(IMFAsyncResult* pResult, IMFMediaEvent** ppEvent)
{ 
    return m_pEventQueue->EndGetEvent(pResult, ppEvent);
}

// OK
HRESULT CustomStreamSink::GetEvent(DWORD dwFlags, IMFMediaEvent** ppEvent)
{
    return m_pEventQueue->GetEvent(dwFlags, ppEvent);
}

// OK
HRESULT CustomStreamSink::QueueEvent(MediaEventType met, REFGUID guidExtendedType, HRESULT hrStatus, PROPVARIANT const* pvValue)
{
    return m_pEventQueue->QueueEventParamVar(met, guidExtendedType, hrStatus, pvValue);
}

//-----------------------------------------------------------------------------
// IMFStreamSink Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomStreamSink::Flush()
{
    return S_OK;
}

// OK
HRESULT CustomStreamSink::GetIdentifier(DWORD* pdwIdentifier)
{
    *pdwIdentifier = m_dwStreamSinkIdentifier;
    return S_OK;
}

// OK
HRESULT CustomStreamSink::GetMediaSink(IMFMediaSink** ppMediaSink)
{
    (*ppMediaSink = m_pSink)->AddRef();
    return S_OK;
}

// OK
HRESULT CustomStreamSink::GetMediaTypeHandler(IMFMediaTypeHandler** ppHandler)
{
    (*ppHandler = m_pHandler)->AddRef();
    return S_OK;
}

// OK
HRESULT CustomStreamSink::PlaceMarker(MFSTREAMSINK_MARKER_TYPE eMarkerType, PROPVARIANT const* pvarMarkerValue, PROPVARIANT const* pvarContextValue)
{
    (void)eMarkerType;
    (void)pvarMarkerValue;
    return QueueEvent(MEStreamSinkMarker, GUID_NULL, S_OK, pvarContextValue);
}

// OK
HRESULT CustomStreamSink::ProcessSample(IMFSample* pSample)
{
    static_cast<CustomMediaSink*>(m_pSink)->InvokeHook(pSample);
    return QueueEvent(MEStreamSinkRequestSample, GUID_NULL, S_OK, NULL);
}

//-----------------------------------------------------------------------------
// CustomMediaSink Events
//-----------------------------------------------------------------------------

// OK
void CustomStreamSink::Shutdown()
{
    m_pEventQueue->Shutdown();
}

// OK
void CustomStreamSink::Start(MFTIME hnsSystemTime, LONGLONG llClockStartOffset)
{
    (void)hnsSystemTime;
    (void)llClockStartOffset;
    QueueEvent(MEStreamSinkStarted,       GUID_NULL, S_OK, NULL);
    QueueEvent(MEStreamSinkRequestSample, GUID_NULL, S_OK, NULL);
}

// OK
void CustomStreamSink::Stop(MFTIME hnsSystemTime)
{
    (void)hnsSystemTime;
}

// OK
void CustomStreamSink::Pause(MFTIME hnsSystemTime)
{
    (void)hnsSystemTime;
}

// OK
void CustomStreamSink::Restart(MFTIME hnsSystemTime)
{
    (void)hnsSystemTime;
}

// OK
void CustomStreamSink::SetRate(MFTIME hnsSystemTime, float flRate)
{
    (void)hnsSystemTime;
    (void)flRate;
}
