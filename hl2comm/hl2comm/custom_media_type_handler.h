
#pragma once

#include <mfidl.h>

// OK
class CustomMediaTypeHandler : public IMFMediaTypeHandler
{
private:
    ULONG            m_nRefCount;
    CRITICAL_SECTION m_critSec;
    IMFMediaType*    m_pType;

    CustomMediaTypeHandler();
    ~CustomMediaTypeHandler();

public:
    static HRESULT CreateInstance(IMFMediaTypeHandler** ppHandler);

    // IUnknown Methods
    ULONG   AddRef();
    ULONG   Release();
    HRESULT QueryInterface(REFIID iid, void** ppv);

    // IMFMediaTypeHandler Methods
    HRESULT GetCurrentMediaType(IMFMediaType** ppMediaType);
    HRESULT GetMajorType(GUID* pguidMajorType);
    HRESULT GetMediaTypeByIndex(DWORD dwIndex, IMFMediaType** ppType);
    HRESULT GetMediaTypeCount(DWORD* pdwTypeCount);
    HRESULT IsMediaTypeSupported(IMFMediaType* pMediaType, IMFMediaType** ppMediaType);
    HRESULT SetCurrentMediaType(IMFMediaType* pMediaType);
};
