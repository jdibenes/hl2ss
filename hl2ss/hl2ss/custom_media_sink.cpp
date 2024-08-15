
#include <new>
#include <mferror.h>
#include "custom_media_sink.h"
#include "custom_stream_sink.h"
#include "lock.h"

//-----------------------------------------------------------------------------
// CustomMediaSink Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomMediaSink::CreateInstance(CustomMediaSink** ppSink, DWORD dwCharacteristics, CustomSinkHook* pHook)
{
    if (!ppSink || !pHook) { return E_INVALIDARG; }
    CustomMediaSink* pSink = new (std::nothrow) CustomMediaSink(dwCharacteristics, pHook);
    if (!pSink) { return E_OUTOFMEMORY; }
    *ppSink = pSink;
    return S_OK;
}

// OK
CustomMediaSink::CustomMediaSink(DWORD dwCharacteristics, CustomSinkHook* pHook)
{
    m_dwCharacteristics = dwCharacteristics;
    m_nRefCount = 1;
    InitializeCriticalSection(&m_critSec);
    m_isShutdown = false;
    (m_pHook = pHook)->AddRef();
    m_pClock = NULL;
}

// OK
CustomMediaSink::~CustomMediaSink()
{
    //assert(m_isShutDown)
    //assert(!m_nRefCount)
    DeleteCriticalSection(&m_critSec);
    m_pHook->Release();
}

// OK
DWORD CustomMediaSink::GetStreamId(IMFStreamSink* stream)
{
    DWORD dwId;
    stream->GetIdentifier(&dwId); // No failure cases
    return dwId;
}

// OK
CustomStreamSink* CustomMediaSink::GetStreamSinkById(DWORD dwStreamSinkIdentifier)
{
    for (auto const& stream : m_streams) { if (GetStreamId(stream) == dwStreamSinkIdentifier) { return stream; } }
    return NULL;
}

// OK
CustomStreamSink* CustomMediaSink::GetStreamSinkByIndex(DWORD dwIndex)
{
    DWORD idx = 0;
    for (auto const& stream : m_streams) { if (idx++ == dwIndex) { return stream; } }
    return NULL;
}

// OK
void CustomMediaSink::CleanupStreams()
{
    for (auto const& stream : m_streams) { stream->Release(); }
    m_streams.clear();
}

//-----------------------------------------------------------------------------
// IUnknown Methods
//-----------------------------------------------------------------------------

// OK
ULONG CustomMediaSink::AddRef()
{
    return InterlockedIncrement(&m_nRefCount);
}

// OK
ULONG CustomMediaSink::Release()
{
    ULONG uCount = InterlockedDecrement(&m_nRefCount);
    if (uCount == 0) { delete this; }
    return uCount;
}

// OK
HRESULT CustomMediaSink::QueryInterface(REFIID iid, void** ppv)
{
    if (!ppv) { return E_POINTER; }

    *ppv = NULL;

    if      (iid == IID_IUnknown)          { *ppv = static_cast<IUnknown*>(static_cast<IMFMediaSink*>(this)); }
    else if (iid == IID_IMFMediaSink)      { *ppv = static_cast<IMFMediaSink*>(this); }
    else if (iid == IID_IMFClockStateSink) { *ppv = static_cast<IMFClockStateSink*>(this); }
    else                                   { return E_NOINTERFACE; }

    AddRef();
    return S_OK;
}

//-----------------------------------------------------------------------------
// IMFMediaSink Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomMediaSink::GetCharacteristics(DWORD* pdwCharacteristics)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    if (!pdwCharacteristics) { return E_INVALIDARG; }
    *pdwCharacteristics = m_dwCharacteristics;
    return S_OK;
}

// OK
HRESULT CustomMediaSink::AddStreamSink(DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType, IMFStreamSink** ppStreamSink)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    if (!ppStreamSink) { return E_INVALIDARG; }
    CustomStreamSink* pSink = GetStreamSinkById(dwStreamSinkIdentifier);
    if (pSink) { return MF_E_STREAMSINK_EXISTS; }
    HRESULT hr = CustomStreamSink::CreateInstance(&pSink, this, dwStreamSinkIdentifier, pMediaType, m_pHook);
    if (FAILED(hr)) { return hr; }
    m_streams.push_back(pSink);
    (*ppStreamSink = pSink)->AddRef();
    return S_OK;
}

// OK
HRESULT CustomMediaSink::RemoveStreamSink(DWORD dwStreamSinkIdentifier)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    CustomStreamSink* pSink = GetStreamSinkById(dwStreamSinkIdentifier);
    if (!pSink) { return MF_E_INVALIDSTREAMNUMBER; }
    pSink->Detach();
    pSink->Release();
    m_streams.remove(pSink);
    return S_OK;
}

// OK
HRESULT CustomMediaSink::GetStreamSinkCount(DWORD* pcStreamSinkCount)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    if (!pcStreamSinkCount) { return E_INVALIDARG; }
    *pcStreamSinkCount = static_cast<DWORD>(m_streams.size());
    return S_OK;
}

// OK
HRESULT CustomMediaSink::GetStreamSinkByIndex(DWORD dwIndex, IMFStreamSink** ppStreamSink)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    if (!ppStreamSink) { return E_INVALIDARG; }    
    if (dwIndex >= m_streams.size()) { return MF_E_INVALIDINDEX; }
    CustomStreamSink* pSink = GetStreamSinkByIndex(dwIndex);
    (*ppStreamSink = static_cast<IMFStreamSink*>(pSink))->AddRef();
    return S_OK;
}

// OK
HRESULT CustomMediaSink::GetStreamSinkById(DWORD dwStreamSinkIdentifier, IMFStreamSink** ppStreamSink)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    if (!ppStreamSink) { return E_INVALIDARG; }
    CustomStreamSink* pSink = GetStreamSinkById(dwStreamSinkIdentifier);
    if (!pSink) { return MF_E_INVALIDSTREAMNUMBER; }
    (*ppStreamSink = static_cast<IMFStreamSink*>(pSink))->AddRef();
    return S_OK;    
}

// OK
HRESULT CustomMediaSink::SetPresentationClock(IMFPresentationClock* pPresentationClock)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    HRESULT hr;
    if (m_pClock)            { hr = m_pClock->RemoveClockStateSink(this);        if (FAILED(hr)) { return hr; } /*SafeRelease(&m_pClock);*/ m_pClock->Release(); m_pClock = NULL; }
    if (pPresentationClock)  { hr = pPresentationClock->AddClockStateSink(this); if (FAILED(hr)) { return hr; } pPresentationClock->AddRef(); }
    m_pClock = pPresentationClock;
    return S_OK;
}

// OK
HRESULT CustomMediaSink::GetPresentationClock(IMFPresentationClock** ppPresentationClock)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    if (!ppPresentationClock) { return E_INVALIDARG; }
    if (!m_pClock) { return MF_E_NO_CLOCK; }
    (*ppPresentationClock = m_pClock)->AddRef();
    return S_OK;
}

// OK
HRESULT CustomMediaSink::Shutdown()
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    for (auto const& stream : m_streams) { stream->Shutdown(); } // TODO: Error Handling
    //SafeRelease(&m_pClock);
    CleanupStreams();    
    m_isShutdown = true;
    if (!m_pClock) { return S_OK; }
    m_pClock->Release();
    m_pClock = NULL;
    return S_OK;
}

//-----------------------------------------------------------------------------
// IMFClockStateSink Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomMediaSink::OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    for (auto const& stream : m_streams) { stream->Start(hnsSystemTime, llClockStartOffset); } // TODO: Error Handling
    return S_OK;
}

// OK
HRESULT CustomMediaSink::OnClockStop(MFTIME hnsSystemTime)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    for (auto const& stream : m_streams) { stream->Stop(hnsSystemTime); } // TODO: Error Handling
    return S_OK;
}

// OK
HRESULT CustomMediaSink::OnClockPause(MFTIME hnsSystemTime)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    for (auto const& stream : m_streams) { stream->Pause(hnsSystemTime); } // TODO: Error Handling
    return S_OK;
}

// OK
HRESULT CustomMediaSink::OnClockRestart(MFTIME hnsSystemTime)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    for (auto const& stream : m_streams) { stream->Restart(hnsSystemTime); } // TODO: Error Handling
    return S_OK;
}

// OK
HRESULT CustomMediaSink::OnClockSetRate(MFTIME hnsSystemTime, float flRate)
{
    CriticalSection cs(&m_critSec);
    if (m_isShutdown) { return MF_E_SHUTDOWN; }
    for (auto const& stream : m_streams) { stream->SetRate(hnsSystemTime, flRate); } // TODO: Error Handling
    return S_OK;
}
