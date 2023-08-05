
#pragma once

#include <mfidl.h>

// OK
class CustomStreamHook : public IUnknown
{
public:
    virtual void ProcessSample(IMFSample* pSample) = 0;
};

// OK
class CustomSinkHook : public IUnknown
{
public:
    virtual HRESULT CreateHook(DWORD dwStreamSinkIdentifier, IMFMediaType* pMediaType, CustomStreamHook** ppHook) = 0;
};
