
#include <new>
#include <mfapi.h>
#include <mferror.h>
#include "custom_stream_sink.h"
#include "custom_media_type_handler.h"
#include "lock.h"

//-----------------------------------------------------------------------------
// CustomStreamSink Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomStreamSink::CreateInstance(CustomStreamSink** ppStream, IMFMediaSink* pSink, DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType, CustomSinkHook* pHook)
{
    if (!ppStream || !pSink || !pHook) { return E_INVALIDARG; }
    CustomStreamSink* pStream = new (std::nothrow) CustomStreamSink(pSink, dwStreamSinkIdentifier);
    if (!pStream) { return E_OUTOFMEMORY; }
    HRESULT hr = pStream->Initialize(dwStreamSinkIdentifier, pMediaType, pHook);
    if (SUCCEEDED(hr)) { (*ppStream = pStream)->AddRef(); }
    //SafeRelease(&pStream);
    pStream->Release();
    return hr;
}

// OK
CustomStreamSink::CustomStreamSink(IMFMediaSink *pSink, DWORD dwStreamSinkIdentifier)
{
    m_nRefCount = 1;
    InitializeCriticalSection(&m_critSec);
    m_isShutdown = false;
    m_dwStreamSinkIdentifier = dwStreamSinkIdentifier;
    (m_pSink = pSink)->AddRef();
    m_pEventQueue = NULL;
    m_pHandler = NULL;
    m_pHook = NULL;
}

// OK
HRESULT CustomStreamSink::Initialize(DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType, CustomSinkHook* pHook)
{
    HRESULT hr = pHook->CreateHook(dwStreamSinkIdentifier, pMediaType, &m_pHook);
    if (FAILED(hr)) { return hr; }
    hr = MFCreateEventQueue(&m_pEventQueue);
    if (FAILED(hr)) { return hr; }
    hr = CustomMediaTypeHandler::CreateInstance(&m_pHandler);
    if (FAILED(hr)) { return hr; }
    if (pMediaType) { hr = m_pHandler->SetCurrentMediaType(pMediaType); }
    return hr;
}

// OK
CustomStreamSink::~CustomStreamSink()
{
    //assert(!m_RefCount)
    DeleteCriticalSection(&m_critSec);
    //SafeRelease(&m_pSink);
    if (m_pSink) { m_pSink->Release(); }
    //SafeRelease(&m_pEventQueue);
    if (m_pEventQueue) { m_pEventQueue->Release(); }
    //SafeRelease(&m_pHandler);
    if (m_pHandler) { m_pHandler->Release(); }
    //SafeRelease(&m_pHook);
    if (m_pHook) { m_pHook->Release(); }
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
    if (!ppv) { return E_INVALIDARG; }

    *ppv = NULL;

    if      (iid == IID_IUnknown)               { *ppv = static_cast<IUnknown*>(static_cast<IMFMediaEventGenerator*>(this)); }
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
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }    
    return m_pEventQueue->BeginGetEvent(pCallback, punkState);
}

// OK
HRESULT CustomStreamSink::EndGetEvent(IMFAsyncResult* pResult, IMFMediaEvent** ppEvent)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }    
    return m_pEventQueue->EndGetEvent(pResult, ppEvent);
}

// OK
HRESULT CustomStreamSink::GetEvent(DWORD dwFlags, IMFMediaEvent** ppEvent)
{
    IMFMediaEventQueue* pQueue;
    {
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    (pQueue = m_pEventQueue)->AddRef();
    }
    HRESULT hr = pQueue->GetEvent(dwFlags, ppEvent);
    pQueue->Release();
    return hr;
}

// OK
HRESULT CustomStreamSink::QueueEvent(MediaEventType met, REFGUID guidExtendedType, HRESULT hrStatus, const PROPVARIANT* pvValue)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    return m_pEventQueue->QueueEventParamVar(met, guidExtendedType, hrStatus, pvValue);
}

//-----------------------------------------------------------------------------
// IMFStreamSink Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomStreamSink::Flush()
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    if (!m_pSink) { return MF_E_STREAMSINK_REMOVED; }
    return S_OK;
}

// OK
HRESULT CustomStreamSink::GetIdentifier(DWORD* pdwIdentifier)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    if (!m_pSink) { return MF_E_STREAMSINK_REMOVED; }
    if (!pdwIdentifier) { return E_INVALIDARG; }
    *pdwIdentifier = m_dwStreamSinkIdentifier;
    return S_OK;
}

// OK
HRESULT CustomStreamSink::GetMediaSink(IMFMediaSink** ppMediaSink)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    if (!m_pSink) { return MF_E_STREAMSINK_REMOVED; }
    if (!ppMediaSink) { return E_INVALIDARG; }    
    (*ppMediaSink = m_pSink)->AddRef();
    return S_OK;
}

// OK
HRESULT CustomStreamSink::GetMediaTypeHandler(IMFMediaTypeHandler** ppHandler)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    if (!m_pSink) { return MF_E_STREAMSINK_REMOVED; }
    if (!ppHandler) { return E_INVALIDARG; }    
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
{
    CriticalSection cs(&m_critSec);
    m_pHook->ProcessSample(pSample);
}
    return QueueEvent(MEStreamSinkRequestSample, GUID_NULL, S_OK, NULL);
}

//-----------------------------------------------------------------------------
// CustomMediaSink Events
//-----------------------------------------------------------------------------

// OK
void CustomStreamSink::Shutdown()
{
    CriticalSection cs(&m_critSec);
    if (m_pEventQueue) { m_pEventQueue->Shutdown(); } // TODO: Error Handling
    m_isShutdown = true;
}

// OK
void CustomStreamSink::Detach()
{
    CriticalSection cs(&m_critSec);
    //SafeRelease(&m_pSink);
    if (!m_pSink) { return; }
    m_pSink->Release();
    m_pSink = NULL;
}

// OK
void CustomStreamSink::Start(MFTIME hnsSystemTime, LONGLONG llClockStartOffset)
{
    (void)hnsSystemTime;
    (void)llClockStartOffset;
    QueueEvent(MEStreamSinkStarted, GUID_NULL, S_OK, NULL);       // TODO: Error Handling
    QueueEvent(MEStreamSinkRequestSample, GUID_NULL, S_OK, NULL); // TODO: Error Handling
}

// OK
void CustomStreamSink::Stop(MFTIME hnsSystemTime)
{
    (void)hnsSystemTime;
    QueueEvent(MEStreamSinkStopped, GUID_NULL, S_OK, NULL); // TODO: Error Handling
}

// OK
void CustomStreamSink::Pause(MFTIME hnsSystemTime)
{
    (void)hnsSystemTime;
    QueueEvent(MEStreamSinkPaused, GUID_NULL, S_OK, NULL); // TODO: Error Handling
}

// OK
void CustomStreamSink::Restart(MFTIME hnsSystemTime)
{
    (void)hnsSystemTime;
    QueueEvent(MEStreamSinkStarted, GUID_NULL, S_OK, NULL);       // TODO: Error Handling
    QueueEvent(MEStreamSinkRequestSample, GUID_NULL, S_OK, NULL); // TODO: Error Handling
}

// OK
void CustomStreamSink::SetRate(MFTIME hnsSystemTime, float flRate)
{
    (void)hnsSystemTime;
    (void)flRate;
    // NOP
}
