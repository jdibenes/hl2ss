
#pragma once

#include <list>
#include "custom_sink_hook.h"

// OK
class CustomMediaSink : public IMFMediaSink, public IMFClockStateSink
{
    friend class CustomStreamSink;

private:
    DWORD                        m_dwCharacteristics;
    ULONG                        m_nRefCount;
    CRITICAL_SECTION             m_critSec;
    bool                         m_isShutdown;
    CustomSinkHook*              m_pHook;
    IMFPresentationClock*        m_pClock;
    std::list<CustomStreamSink*> m_streams;    

    CustomMediaSink(DWORD dwCharacteristics, CustomSinkHook* pHook);
    ~CustomMediaSink();

    DWORD GetStreamId(IMFStreamSink *stream);

    CustomStreamSink* GetStreamSinkById(DWORD dwStreamSinkIdentifier);
    CustomStreamSink* GetStreamSinkByIndex(DWORD dwIndex);

    void CleanupStreams();

public:
    static HRESULT CreateInstance(CustomMediaSink** ppSink, DWORD dwCharacteristics, CustomSinkHook* pHook);

    // IUnknown Methods
    ULONG   AddRef();
    ULONG   Release();
    HRESULT QueryInterface(REFIID iid, void** ppv);

    // IMFMediaSink Methods
    HRESULT GetCharacteristics(DWORD* pdwCharacteristics);
    HRESULT AddStreamSink(DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType, IMFStreamSink** ppStreamSink);
    HRESULT RemoveStreamSink(DWORD dwStreamSinkIdentifier);
    HRESULT GetStreamSinkByIndex(DWORD dwIndex, IMFStreamSink** ppStreamSink);
    HRESULT GetStreamSinkById(DWORD dwStreamSinkIdentifier, IMFStreamSink** ppStreamSink);
    HRESULT GetStreamSinkCount(DWORD* pcStreamSinkCount);
    HRESULT SetPresentationClock(IMFPresentationClock* pPresentationClock);
    HRESULT GetPresentationClock(IMFPresentationClock** ppPresentationClock);
    HRESULT Shutdown();

    // IMFClockStateSink Methods
    HRESULT OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset);
    HRESULT OnClockStop(MFTIME hnsSystemTime);
    HRESULT OnClockPause(MFTIME hnsSystemTime);
    HRESULT OnClockRestart(MFTIME hnsSystemTime);
    HRESULT OnClockSetRate(MFTIME hnsSystemTime, float flRate);
};
