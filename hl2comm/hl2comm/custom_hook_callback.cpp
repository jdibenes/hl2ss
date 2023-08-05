
#include <new>
#include "custom_hook_callback.h"

//-----------------------------------------------------------------------------
// HookSinkCallback Methods
//-----------------------------------------------------------------------------

// OK
ULONG HookSinkCallback::AddRef()
{
    return InterlockedIncrement(&m_nRefCount);
}

// OK
ULONG HookSinkCallback::Release()
{
    ULONG uCount = InterlockedDecrement(&m_nRefCount);
    if (uCount == 0) { delete this; }
    return uCount;
}

// OK
HRESULT HookSinkCallback::QueryInterface(REFIID iid, void** ppv)
{
    if (!ppv) { return E_POINTER; }

    *ppv = NULL;

    if (iid == IID_IUnknown) { *ppv = static_cast<IUnknown*>(this); }
    else                     { return E_NOINTERFACE; }

    AddRef();
    return S_OK;
}

// OK
HookSinkCallback::HookSinkCallback(HOOK_SINK_PROC fCallback, void* param)
{
    //assert(fCallback)
    m_nRefCount = 1;
    m_fCallback = fCallback;
    m_param = param;
}

// OK
HookSinkCallback::~HookSinkCallback()
{
    //assert(!m_nRefCount)
}

// OK
HRESULT HookSinkCallback::CreateInstance(HookSinkCallback** ppSink, HOOK_SINK_PROC fCallback, void* param)
{
    if (!ppSink || !fCallback) { return E_INVALIDARG; }
    HookSinkCallback* pSink = new (std::nothrow) HookSinkCallback(fCallback, param);
    if (!pSink) { return E_OUTOFMEMORY; }
    *ppSink = pSink;
    return S_OK;
}

// OK
HRESULT HookSinkCallback::CreateHook(DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType, CustomStreamHook** ppHook)
{
    (void)dwStreamSinkIdentifier;
    (void)pMediaType;
    if (!ppHook) { return E_INVALIDARG; }
    CustomStreamHook* pHook = new (std::nothrow) HookStreamCallback(this);
    if (!pHook) { return E_OUTOFMEMORY; }
    *ppHook = pHook;
    return S_OK;
}

// OK
void HookSinkCallback::Dispatch(IMFSample* pSample)
{
    m_fCallback(pSample, m_param);
}

//-----------------------------------------------------------------------------
// HookStreamCallback
//-----------------------------------------------------------------------------

// OK
ULONG HookStreamCallback::AddRef()
{
    return InterlockedIncrement(&m_nRefCount);
}

// OK
ULONG HookStreamCallback::Release()
{
    ULONG uCount = InterlockedDecrement(&m_nRefCount);
    if (uCount == 0) { delete this; }
    return uCount;
}

// OK
HRESULT HookStreamCallback::QueryInterface(REFIID iid, void** ppv)
{
    if (!ppv) { return E_POINTER; }

    *ppv = NULL;

    if (iid == IID_IUnknown) { *ppv = static_cast<IUnknown*>(this); }
    else { return E_NOINTERFACE; }

    AddRef();
    return S_OK;
}

// OK
HookStreamCallback::HookStreamCallback(HookSinkCallback *pParent)
{
    //assert(pParent)
    m_nRefCount = 1;
    (m_pParent = pParent)->AddRef();
}

// OK
HookStreamCallback::~HookStreamCallback()
{
    //assert(!m_nRefCount)
    m_pParent->Release();
}

// OK
void HookStreamCallback::ProcessSample(IMFSample* pSample)
{
    m_pParent->Dispatch(pSample);
}
