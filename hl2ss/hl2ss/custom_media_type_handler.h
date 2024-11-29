
#pragma once

#include <mfidl.h>

class CustomMediaTypeHandler : public IMFMediaTypeHandler
{
private:
    ULONG         m_nRefCount;
    IMFMediaType* m_pType; // Release

    CustomMediaTypeHandler(IMFMediaType* pMediaType);
    ~CustomMediaTypeHandler();

public:
    static HRESULT CreateInstance(IMFMediaTypeHandler** ppHandler, IMFMediaType* pMediaType);

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
