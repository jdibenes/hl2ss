
#include <mferror.h>
#include "custom_media_buffers.h"

using namespace winrt::Windows::Storage::Streams;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Graphics::Imaging;

//*****************************************************************************
// SoftwareBitmapBuffer
//*****************************************************************************

//-----------------------------------------------------------------------------
// IUnknown Methods
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

    if      (iid == IID_IUnknown)       { *ppv = static_cast<IUnknown*>(this); }
    else if (iid == IID_IMFMediaBuffer) { *ppv = static_cast<IMFMediaBuffer*>(this); }
    else                                { return E_NOINTERFACE; }

    AddRef();
    return S_OK;
}

//-----------------------------------------------------------------------------
// IMFMediaBuffer Methods
//-----------------------------------------------------------------------------

// OK
HRESULT SoftwareBitmapBuffer::GetCurrentLength(DWORD* pcbCurrentLength)
{
    *pcbCurrentLength = m_curLength;
    return S_OK;
}

// OK
HRESULT SoftwareBitmapBuffer::GetMaxLength(DWORD* pcbMaxLength)
{
    *pcbMaxLength = m_maxLength;
    return S_OK;
}

// OK
HRESULT SoftwareBitmapBuffer::Lock(BYTE** ppbBuffer, DWORD* pcbMaxLength, DWORD* pcbCurrentLength)
{
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
    return S_OK;
}

//-----------------------------------------------------------------------------
// SoftwareBitmapBuffer Methods
//-----------------------------------------------------------------------------

// OK
SoftwareBitmapBuffer::SoftwareBitmapBuffer(MediaFrameReference const& ref)
{
    m_nRefCount = 1;
    m_bmp       = ref.VideoMediaFrame().SoftwareBitmap();
    m_buf       = m_bmp.LockBuffer(BitmapBufferAccessMode::Read);
    m_ref       = m_buf.CreateReference();
    m_pBase     = m_ref.data();
    m_maxLength =
    m_curLength = m_ref.Capacity();
}

// OK
SoftwareBitmapBuffer::~SoftwareBitmapBuffer()
{
    m_ref.Close();
    m_ref = nullptr;
    m_buf.Close();
    m_buf = nullptr;
    m_bmp.Close();
    m_bmp = nullptr;
}

// OK
HRESULT SoftwareBitmapBuffer::CreateInstance(SoftwareBitmapBuffer** ppBuffer, MediaFrameReference const& ref)
{
    *ppBuffer = new SoftwareBitmapBuffer(ref);
    return S_OK;
}

//*****************************************************************************
// BufferBuffer
//*****************************************************************************

//-----------------------------------------------------------------------------
// IUnknown Methods
//-----------------------------------------------------------------------------

// OK
ULONG BufferBuffer::AddRef()
{
    return InterlockedIncrement(&m_nRefCount);
}

// OK
ULONG BufferBuffer::Release()
{
    ULONG uCount = InterlockedDecrement(&m_nRefCount);
    if (uCount == 0) { delete this; }
    return uCount;
}

// OK
HRESULT BufferBuffer::QueryInterface(REFIID iid, void** ppv)
{
    if (!ppv) { return E_POINTER; }

    *ppv = NULL;

    if      (iid == IID_IUnknown)       { *ppv = static_cast<IUnknown*>(this); }
    else if (iid == IID_IMFMediaBuffer) { *ppv = static_cast<IMFMediaBuffer*>(this); }
    else                                { return E_NOINTERFACE; }

    AddRef();
    return S_OK;
}

//-----------------------------------------------------------------------------
// IMFMediaBuffer Methods
//-----------------------------------------------------------------------------

// OK
HRESULT BufferBuffer::GetCurrentLength(DWORD* pcbCurrentLength)
{
    *pcbCurrentLength = m_curLength;
    return S_OK;
}

// OK
HRESULT BufferBuffer::GetMaxLength(DWORD* pcbMaxLength)
{
    *pcbMaxLength = m_maxLength;
    return S_OK;
}

// OK
HRESULT BufferBuffer::Lock(BYTE** ppbBuffer, DWORD* pcbMaxLength, DWORD* pcbCurrentLength)
{
    *ppbBuffer = m_pBase;
    if (pcbMaxLength) { *pcbMaxLength = m_maxLength; }
    if (pcbCurrentLength) { *pcbCurrentLength = m_curLength; }
    return S_OK;
}

// OK
HRESULT BufferBuffer::SetCurrentLength(DWORD cbCurrentLength)
{
    m_curLength = cbCurrentLength;
    return S_OK;
}

// OK
HRESULT BufferBuffer::Unlock()
{
    return S_OK;
}

//-----------------------------------------------------------------------------
// BufferBuffer Methods
//-----------------------------------------------------------------------------

// OK
BufferBuffer::BufferBuffer(Buffer const& ref)
{
    m_nRefCount = 1;
    m_buf       = ref;
    m_pBase     = ref.data();
    m_maxLength =
    m_curLength = ref.Length();
}

// OK
BufferBuffer::~BufferBuffer()
{
}

// OK
HRESULT BufferBuffer::CreateInstance(BufferBuffer** ppBuffer, Buffer const& ref)
{
    *ppBuffer = new BufferBuffer(ref);
    return S_OK;
}
