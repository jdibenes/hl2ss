
#include <new>
#include <mferror.h>
#include "custom_media_buffers.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Media.Capture.Frames.h>
#include <winrt/Windows.Graphics.Imaging.h>

using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Graphics::Imaging;

//-----------------------------------------------------------------------------
// SoftwareBitmapBuffer Methods
//-----------------------------------------------------------------------------

// OK
ULONG SoftwareBitmapBuffer::AddRef()
{
    return InterlockedIncrement(&m_nRefCount);
}

// OK
ULONG SoftwareBitmapBuffer::Release()
{
    ULONG uCount = InterlockedDecrement(&m_nRefCount);
    if (uCount == 0) { delete this; }
    return uCount;
}

// OK
HRESULT SoftwareBitmapBuffer::QueryInterface(REFIID iid, void** ppv)
{
    if (!ppv) { return E_POINTER; }

    *ppv = NULL;

    if (iid == IID_IUnknown) { *ppv = static_cast<IUnknown*>(this); }
    else if (iid == IID_IMFMediaBuffer) { *ppv = static_cast<IMFMediaBuffer*>(this); }
    else { return E_NOINTERFACE; }

    AddRef();
    return S_OK;
}

// OK
HRESULT SoftwareBitmapBuffer::GetCurrentLength(DWORD* pcbCurrentLength)
{
    if (!pcbCurrentLength) { return E_INVALIDARG; }
    *pcbCurrentLength = m_curLength;
    return S_OK;
}

// OK
HRESULT SoftwareBitmapBuffer::GetMaxLength(DWORD* pcbMaxLength)
{
    if (!pcbMaxLength) { return E_INVALIDARG; }
    *pcbMaxLength = m_maxLength;
    return S_OK;
}

// OK
HRESULT SoftwareBitmapBuffer::Lock(BYTE** ppbBuffer, DWORD* pcbMaxLength, DWORD* pcbCurrentLength)
{
    if (!ppbBuffer) { return E_INVALIDARG; }
    if (m_isLocked) { return MF_E_INVALIDREQUEST; }
    m_isLocked = true;
    *ppbBuffer = m_pBase;
    if (pcbMaxLength) { *pcbMaxLength = m_maxLength; }
    if (pcbCurrentLength) { *pcbCurrentLength = m_curLength; }
    return S_OK;
}

// OK
HRESULT SoftwareBitmapBuffer::SetCurrentLength(DWORD cbCurrentLength)
{
    m_curLength = cbCurrentLength;
    return S_OK;
}

// OK
HRESULT SoftwareBitmapBuffer::Unlock()
{
    if (!m_isLocked) { return MF_E_INVALIDREQUEST; }
    m_isLocked = false;
    return S_OK;
}

// OK
SoftwareBitmapBuffer::SoftwareBitmapBuffer(MediaFrameReference const &ref) : m_bmp(nullptr), m_buf(nullptr)
{
    UINT32 length;

    m_nRefCount = 1;
    m_isLocked  = false;
    m_bmp       = ref.VideoMediaFrame().SoftwareBitmap(); // TODO: Error Handling
    m_buf       = m_bmp.LockBuffer(BitmapBufferAccessMode::Read); // TODO: Error Handling
    m_ref       = m_buf.CreateReference(); // TODO: Error Handling
    m_bba       = m_ref.as<Windows::Foundation::IMemoryBufferByteAccess>(); // TODO: Error Handling

    m_bba->GetBuffer(&m_pBase, &length); // TODO: Error Handling

    m_maxLength = length;
    m_curLength = length;
}

// OK
SoftwareBitmapBuffer::~SoftwareBitmapBuffer()
{
    //assert(!m_nRefcount);
    //assert(!m_isLocked);
                   m_bba = nullptr;
    m_ref.Close(); m_ref = nullptr;
    m_buf.Close(); m_buf = nullptr;
    m_bmp.Close(); m_bmp = nullptr;
}

// OK
HRESULT SoftwareBitmapBuffer::CreateInstance(SoftwareBitmapBuffer** ppBuffer, MediaFrameReference const& ref)
{
    if (!ppBuffer) { return E_INVALIDARG; }
    *ppBuffer = new (std::nothrow) SoftwareBitmapBuffer(ref);
    return S_OK;
}

//-----------------------------------------------------------------------------
// WrappedBuffer Methods
//-----------------------------------------------------------------------------

// OK
ULONG WrappedBuffer::AddRef()
{
    return InterlockedIncrement(&m_nRefCount);
}

// OK
ULONG WrappedBuffer::Release()
{
    ULONG uCount = InterlockedDecrement(&m_nRefCount);
    if (uCount == 0) { delete this; }
    return uCount;
}

// OK
HRESULT WrappedBuffer::QueryInterface(REFIID iid, void** ppv)
{
    if (!ppv) { return E_POINTER; }

    *ppv = NULL;

    if (iid == IID_IUnknown) { *ppv = static_cast<IUnknown*>(this); }
    else if (iid == IID_IMFMediaBuffer) { *ppv = static_cast<IMFMediaBuffer*>(this); }
    else { return E_NOINTERFACE; }

    AddRef();
    return S_OK;
}

// OK
HRESULT WrappedBuffer::GetCurrentLength(DWORD* pcbCurrentLength)
{
    if (!pcbCurrentLength) { return E_INVALIDARG; }
    *pcbCurrentLength = m_curLength;
    return S_OK;
}

// OK
HRESULT WrappedBuffer::GetMaxLength(DWORD* pcbMaxLength)
{
    if (!pcbMaxLength) { return E_INVALIDARG; }
    *pcbMaxLength = m_maxLength;
    return S_OK;
}

// OK
HRESULT WrappedBuffer::Lock(BYTE** ppbBuffer, DWORD* pcbMaxLength, DWORD* pcbCurrentLength)
{
    if (!ppbBuffer) { return E_INVALIDARG; }
    if (m_isLocked) { return MF_E_INVALIDREQUEST; }
    m_isLocked = true;
    *ppbBuffer = m_pBase;
    if (pcbMaxLength) { *pcbMaxLength = m_maxLength; }
    if (pcbCurrentLength) { *pcbCurrentLength = m_curLength; }
    return S_OK;
}

// OK
HRESULT WrappedBuffer::SetCurrentLength(DWORD cbCurrentLength)
{
    if (cbCurrentLength > m_maxLength) { return E_INVALIDARG; }
    m_curLength = cbCurrentLength;
    return S_OK;
}

// OK
HRESULT WrappedBuffer::Unlock()
{
    if (!m_isLocked) { return MF_E_INVALIDREQUEST; }
    m_isLocked = false;
    return S_OK;
}

// OK
WrappedBuffer::WrappedBuffer(BYTE* pBase, DWORD dwLength)
{
    m_nRefCount = 1;
    m_isLocked = false;

    m_pBase = pBase;

    m_maxLength = dwLength;
    m_curLength = dwLength;
}

// OK
WrappedBuffer::~WrappedBuffer()
{
    //assert(!m_nRefcount);
    //assert(!m_isLocked);
}

// OK
HRESULT WrappedBuffer::CreateInstance(WrappedBuffer** ppBuffer, BYTE *pData, DWORD dwLength)
{
    if (!ppBuffer) { return E_INVALIDARG; }
    *ppBuffer = new (std::nothrow) WrappedBuffer(pData, dwLength);
    return S_OK;
}
