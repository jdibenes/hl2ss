
#include <new>
#include <mferror.h>
#include "custom_media_type_handler.h"
#include "lock.h"

//-----------------------------------------------------------------------------
// CustomMediaTypeHandler Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomMediaTypeHandler::CreateInstance(IMFMediaTypeHandler **ppHandler)
{
    if (!ppHandler) { return E_INVALIDARG; }
    CustomMediaTypeHandler* pHandler = new (std::nothrow) CustomMediaTypeHandler();
    if (!pHandler) { return E_OUTOFMEMORY; }
    *ppHandler = pHandler;
    return S_OK;
}

// OK
CustomMediaTypeHandler::CustomMediaTypeHandler()
{
    m_nRefCount = 1;
    InitializeCriticalSection(&m_critSec);
    m_pType = NULL;
}

// OK
CustomMediaTypeHandler::~CustomMediaTypeHandler()
{
    //assert(!m_nRefCount);
    DeleteCriticalSection(&m_critSec);
    if (m_pType) { m_pType->Release(); }
}

//-----------------------------------------------------------------------------
// IUnknown Methods
//-----------------------------------------------------------------------------

// OK
ULONG CustomMediaTypeHandler::AddRef()
{
    return InterlockedIncrement(&m_nRefCount);
}

// OK
ULONG CustomMediaTypeHandler::Release()
{
    ULONG uCount = InterlockedDecrement(&m_nRefCount);
    if (uCount == 0) { delete this; }
    return uCount;
}

// OK
HRESULT CustomMediaTypeHandler::QueryInterface(REFIID iid, void** ppv)
{
    if (!ppv) { return E_INVALIDARG; }

    *ppv = NULL;

    if      (iid == IID_IUnknown)            { *ppv = static_cast<IUnknown*>(this); }
    else if (iid == IID_IMFMediaTypeHandler) { *ppv = static_cast<IMFMediaTypeHandler*>(this); }
    else                                     { return E_NOINTERFACE; }

    AddRef();
    return S_OK;
}

//-----------------------------------------------------------------------------
// IMFMediaTypeHandler Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomMediaTypeHandler::GetCurrentMediaType(IMFMediaType** ppMediaType)
{
    CriticalSection cs(&m_critSec);
    if (!ppMediaType) { return E_INVALIDARG; }
    if (!m_pType) { return MF_E_NOT_INITIALIZED; }
    (*ppMediaType = m_pType)->AddRef();
    return S_OK;
}

// OK
HRESULT CustomMediaTypeHandler::GetMajorType(GUID* pguidMajorType)
{
    CriticalSection cs(&m_critSec);
    if (!pguidMajorType) { return E_INVALIDARG; }
    if (!m_pType) { return MF_E_NOT_INITIALIZED; }
    return m_pType->GetMajorType(pguidMajorType);
}

// OK
HRESULT CustomMediaTypeHandler::GetMediaTypeByIndex(DWORD dwIndex, IMFMediaType** ppType)
{
    CriticalSection cs(&m_critSec);
    if (!ppType) { return E_INVALIDARG; }
    if (!m_pType || (dwIndex > 0)) { return MF_E_NO_MORE_TYPES; }
    (*ppType = m_pType)->AddRef();
    return S_OK;
}

// OK
HRESULT CustomMediaTypeHandler::GetMediaTypeCount(DWORD* pdwTypeCount)
{
    CriticalSection cs(&m_critSec);
    if (!pdwTypeCount) { return E_INVALIDARG; }    
    *pdwTypeCount = m_pType ? 1 : 0;
    return S_OK;
}

// OK
HRESULT CustomMediaTypeHandler::IsMediaTypeSupported(IMFMediaType* pMediaType, IMFMediaType** ppMediaType)
{
    if (!pMediaType) { return E_INVALIDARG; }
    if (ppMediaType) { *ppMediaType = NULL; }
    return S_OK; // just say yes
}

// OK
HRESULT CustomMediaTypeHandler::SetCurrentMediaType(IMFMediaType* pMediaType)
{
    CriticalSection cs(&m_critSec);
    if (!pMediaType) { return E_INVALIDARG; }
    if (m_pType) { m_pType->Release(); }
    (m_pType = pMediaType)->AddRef();
    return S_OK;
}
