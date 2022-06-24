
#pragma once

#include "custom_sink_hook.h"

typedef void(*HOOK_SINK_PROC)(IMFSample*, void*);

class HookSinkCallback : public CustomSinkHook
{
    friend class HookStreamCallback;

private:
    ULONG m_nRefCount;
    HOOK_SINK_PROC m_fCallback;
    void* m_param;

    HookSinkCallback(HOOK_SINK_PROC fCallback, void* param);
    ~HookSinkCallback();

    void Dispatch(IMFSample* pSample);

public:
    static HRESULT CreateInstance(HookSinkCallback** ppHook, HOOK_SINK_PROC fCallback, void* param);

    // IUnknown Methods
    ULONG   AddRef();
    ULONG   Release();
    HRESULT QueryInterface(REFIID iid, void** ppv);

    // CustomSinkHook Methods
    HRESULT CreateHook(DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType, CustomStreamHook** ppHook);
};

class HookStreamCallback : public CustomStreamHook
{
    friend class HookSinkCallback;

private:
    ULONG m_nRefCount;
    HookSinkCallback* m_pParent;

    HookStreamCallback(HookSinkCallback *pParent);
    ~HookStreamCallback();

public:
    // IUnknown Methods
    ULONG   AddRef();
    ULONG   Release();
    HRESULT QueryInterface(REFIID iid, void** ppv);

    // CustomStreamHook Methods
    void ProcessSample(IMFSample* pSample);
};
