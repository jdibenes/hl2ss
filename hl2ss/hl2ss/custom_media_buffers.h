
#pragma once

#include <MemoryBuffer.h>
#include <mfidl.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Graphics.Imaging.h>

// OK
class SoftwareBitmapBuffer : public IMFMediaBuffer
{
private:
    ULONG m_nRefCount;
    bool  m_isLocked;
    
    winrt::Windows::Graphics::Imaging::SoftwareBitmap                  m_bmp;
    winrt::Windows::Graphics::Imaging::BitmapBuffer                    m_buf;
    winrt::Windows::Foundation::IMemoryBufferReference                 m_ref;
    winrt::impl::com_ref<Windows::Foundation::IMemoryBufferByteAccess> m_bba;

    BYTE* m_pBase;
    DWORD m_maxLength;
    DWORD m_curLength;

    SoftwareBitmapBuffer(winrt::Windows::Media::Capture::Frames::MediaFrameReference const& ref);
    ~SoftwareBitmapBuffer();

public:
    static HRESULT CreateInstance(SoftwareBitmapBuffer** ppBuffer, winrt::Windows::Media::Capture::Frames::MediaFrameReference const& ref);

    // IUnknown Methods
    ULONG AddRef();
    ULONG Release();
    HRESULT QueryInterface(REFIID iid, void** ppv);

    // IMFMediaBuffer Methods
    HRESULT GetCurrentLength(DWORD* pcbCurrentLength);
    HRESULT GetMaxLength(DWORD* pcbMaxLength);
    HRESULT Lock(BYTE** ppbBuffer, DWORD* pcbMaxLength, DWORD* pcbCurrentLength);
    HRESULT SetCurrentLength(DWORD cbCurrentLength);
    HRESULT Unlock();
};

// OK
class WrappedBuffer : public IMFMediaBuffer
{
private:
    ULONG m_nRefCount;
    bool  m_isLocked;
    BYTE* m_pBase;
    DWORD m_maxLength;
    DWORD m_curLength;

    WrappedBuffer(BYTE* pBase, DWORD dwLength);
    ~WrappedBuffer();

public:
    static HRESULT CreateInstance(WrappedBuffer** ppBuffer, BYTE* pBase, DWORD dwLength);

    // IUnknown Methods
    ULONG AddRef();
    ULONG Release();
    HRESULT QueryInterface(REFIID iid, void** ppv);

    // IMFMediaBuffer Methods
    HRESULT GetCurrentLength(DWORD* pcbCurrentLength);
    HRESULT GetMaxLength(DWORD* pcbMaxLength);
    HRESULT Lock(BYTE** ppbBuffer, DWORD* pcbMaxLength, DWORD* pcbCurrentLength);
    HRESULT SetCurrentLength(DWORD cbCurrentLength);
    HRESULT Unlock();
};
