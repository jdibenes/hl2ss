
#include <mferror.h>
#include "custom_media_type_handler.h"

//-----------------------------------------------------------------------------
// CustomMediaTypeHandler Methods
//-----------------------------------------------------------------------------

// OK
HRESULT CustomMediaTypeHandler::CreateInstance(IMFMediaTypeHandler **ppHandler, IMFMediaType* pMediaType)
{
    *ppHandler = new CustomMediaTypeHandler(pMediaType);
    return S_OK;
}

// OK
CustomMediaTypeHandler::CustomMediaTypeHandler(IMFMediaType* pMediaType)
{
    m_nRefCount = 1;
    (m_pType = pMediaType)->AddRef();
}

// OK
CustomMediaTypeHandler::~CustomMediaTypeHandler()
{
    m_pType->Release();
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
    if (!ppv) { return E_POINTER; }

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
    (*ppMediaType = m_pType)->AddRef();
    return S_OK;
}

// OK
HRESULT CustomMediaTypeHandler::GetMajorType(GUID* pguidMajorType)
{
    return m_pType->GetMajorType(pguidMajorType);
}

// OK
HRESULT CustomMediaTypeHandler::GetMediaTypeByIndex(DWORD dwIndex, IMFMediaType** ppMediaType)
{
    if (dwIndex > 0) { return MF_E_NO_MORE_TYPES; }
    (*ppMediaType = m_pType)->AddRef();
    return S_OK;
}

// OK
HRESULT CustomMediaTypeHandler::GetMediaTypeCount(DWORD* pdwTypeCount)
{
    *pdwTypeCount = 1;
    return S_OK;
}

// OK
HRESULT CustomMediaTypeHandler::IsMediaTypeSupported(IMFMediaType* pMediaType, IMFMediaType** ppMediaType)
{
    (void)pMediaType;
    if (ppMediaType) { *ppMediaType = NULL; }
    return S_OK;
}

// OK
HRESULT CustomMediaTypeHandler::SetCurrentMediaType(IMFMediaType* pMediaType)
{
    m_pType->Release();
    (m_pType = pMediaType)->AddRef();
    return S_OK;
}
