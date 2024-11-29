
#pragma once

#include <list>
#include "custom_stream_sink.h"

typedef void(*HOOK_SINK_PROC)(IMFSample*, void*);

class CustomMediaSink : public IMFMediaSink, public IMFClockStateSink
{
private:    
    ULONG                        m_nRefCount;
    DWORD                        m_dwCharacteristics;
    IMFPresentationClock*        m_pClock; // Release
    std::list<CustomStreamSink*> m_streams; // Release
    HOOK_SINK_PROC               m_pHookCallback;
    void*                        m_pHookParam;

    CustomMediaSink(DWORD dwCharacteristics, HOOK_SINK_PROC pHookCallback, void* pHookParam);
    ~CustomMediaSink();

    CustomStreamSink* GetStreamSinkById(DWORD dwStreamSinkIdentifier);
    void ReleaseClock();
    void ReleaseStream(CustomStreamSink* pStream);

public:
    static HRESULT CreateInstance(CustomMediaSink** ppSink, DWORD dwCharacteristics, HOOK_SINK_PROC pHookCallback, void* pHookParam);

    void InvokeHook(IMFSample* pSample);

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
