
#include <mferror.h>
#include "custom_media_sink.h"

//-----------------------------------------------------------------------------
// CustomMediaSink Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomMediaSink::CreateInstance(CustomMediaSink** ppSink, DWORD dwCharacteristics, HOOK_SINK_PROC pHookCallback, void* pHookParam)
{
    *ppSink = new CustomMediaSink(dwCharacteristics, pHookCallback, pHookParam);
    return S_OK;
}

// OK
CustomMediaSink::CustomMediaSink(DWORD dwCharacteristics, HOOK_SINK_PROC pHookCallback, void* pHookParam)
{
    m_nRefCount = 1;
    m_dwCharacteristics = dwCharacteristics;
    m_pClock = NULL;
    m_streams = {};
    m_pHookCallback = pHookCallback;
    m_pHookParam = pHookParam;    
}

// OK
CustomMediaSink::~CustomMediaSink()
{
}

// OK
CustomStreamSink* CustomMediaSink::GetStreamSinkById(DWORD dwStreamSinkIdentifier)
{
    DWORD dwId;
    for (auto const& stream : m_streams) { if (stream->GetIdentifier(&dwId), (dwId == dwStreamSinkIdentifier)) { return stream; } }
    return NULL;
}

// OK
void CustomMediaSink::ReleaseClock()
{
    if (!m_pClock) { return; }
    m_pClock->RemoveClockStateSink(this);
    m_pClock->Release();
    m_pClock = NULL;
}

// OK
void CustomMediaSink::ReleaseStream(CustomStreamSink* pStream)
{
    pStream->Shutdown();
    pStream->Release();
}

// OK
void CustomMediaSink::InvokeHook(IMFSample* pSample)
{
    m_pHookCallback(pSample, m_pHookParam);
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
    *pdwCharacteristics = m_dwCharacteristics;
    return S_OK;
}

// OK
HRESULT CustomMediaSink::AddStreamSink(DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType, IMFStreamSink** ppStreamSink)
{
    CustomStreamSink* pStream = GetStreamSinkById(dwStreamSinkIdentifier);
    if (pStream) { return MF_E_STREAMSINK_EXISTS; }
    CustomStreamSink::CreateInstance(&pStream, this, dwStreamSinkIdentifier, pMediaType);
    (*ppStreamSink = pStream)->AddRef();
    m_streams.push_back(pStream);
    return S_OK;
}

// OK
HRESULT CustomMediaSink::RemoveStreamSink(DWORD dwStreamSinkIdentifier)
{
    CustomStreamSink* pStream = GetStreamSinkById(dwStreamSinkIdentifier);
    if (!pStream) { return MF_E_INVALIDSTREAMNUMBER; }
    ReleaseStream(pStream);
    m_streams.remove(pStream);
    return S_OK;
}

// OK
HRESULT CustomMediaSink::GetStreamSinkCount(DWORD* pcStreamSinkCount)
{
    *pcStreamSinkCount = static_cast<DWORD>(m_streams.size());
    return S_OK;
}

// OK
HRESULT CustomMediaSink::GetStreamSinkByIndex(DWORD dwIndex, IMFStreamSink** ppStreamSink)
{
    if (dwIndex >= m_streams.size()) { return MF_E_INVALIDINDEX; }
    auto base = m_streams.begin();
    std::advance(base, dwIndex);
    (*ppStreamSink = *base)->AddRef();
    return S_OK;
}

// OK
HRESULT CustomMediaSink::GetStreamSinkById(DWORD dwStreamSinkIdentifier, IMFStreamSink** ppStreamSink)
{
    CustomStreamSink* pStream = GetStreamSinkById(dwStreamSinkIdentifier);
    if (!pStream) { return MF_E_INVALIDSTREAMNUMBER; }
    (*ppStreamSink = pStream)->AddRef();
    return S_OK;    
}

// OK
HRESULT CustomMediaSink::SetPresentationClock(IMFPresentationClock* pPresentationClock)
{
    ReleaseClock();
    if (!pPresentationClock) { return S_OK; }
    pPresentationClock->AddClockStateSink(this);
    (m_pClock = pPresentationClock)->AddRef();
    return S_OK;
}

// OK
HRESULT CustomMediaSink::GetPresentationClock(IMFPresentationClock** ppPresentationClock)
{
    if (!m_pClock) { return MF_E_NO_CLOCK; }
    (*ppPresentationClock = m_pClock)->AddRef();
    return S_OK;
}

// OK
HRESULT CustomMediaSink::Shutdown()
{
    ReleaseClock();
    for (auto const& stream : m_streams) { ReleaseStream(stream); }
    m_streams.clear();
    return S_OK;
}

//-----------------------------------------------------------------------------
// IMFClockStateSink Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomMediaSink::OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset)
{
    for (auto const& stream : m_streams) { stream->Start(hnsSystemTime, llClockStartOffset); }
    return S_OK;
}

// OK
HRESULT CustomMediaSink::OnClockStop(MFTIME hnsSystemTime)
{
    for (auto const& stream : m_streams) { stream->Stop(hnsSystemTime); }
    return S_OK;
}

// OK
HRESULT CustomMediaSink::OnClockPause(MFTIME hnsSystemTime)
{
    for (auto const& stream : m_streams) { stream->Pause(hnsSystemTime); }
    return S_OK;
}

// OK
HRESULT CustomMediaSink::OnClockRestart(MFTIME hnsSystemTime)
{
    for (auto const& stream : m_streams) { stream->Restart(hnsSystemTime); }
    return S_OK;
}

// OK
HRESULT CustomMediaSink::OnClockSetRate(MFTIME hnsSystemTime, float flRate)
{
    for (auto const& stream : m_streams) { stream->SetRate(hnsSystemTime, flRate); }
    return S_OK;
}
