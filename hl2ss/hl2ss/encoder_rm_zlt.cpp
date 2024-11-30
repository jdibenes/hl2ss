
#include <MemoryBuffer.h>
#include "encoder_rm_zlt.h"
#include "neon.h"

#include <winrt/Windows.Foundation.Collections.h>

using namespace Windows::Foundation;
using namespace winrt::Windows::Graphics::Imaging;
using namespace winrt::Windows::Storage::Streams;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Encoder_RM_ZLT::Encoder_RM_ZLT(HOOK_RM_ZLT_PROC pHookCallback, void* pHookParam, H26xFormat const& format, ZABFormat const& zabFormat) : m_softwareBitmap(nullptr)
{
    m_pngProperties.Insert(L"FilterOption", BitmapTypedValue(winrt::box_value(zabFormat.filter), winrt::Windows::Foundation::PropertyType::UInt8));
    m_softwareBitmap = SoftwareBitmap(BitmapPixelFormat::Bgra8, format.width, format.height, BitmapAlphaMode::Straight);
    m_pHookCallback = pHookCallback;
    m_pHookParam = pHookParam;
}

// OK
void Encoder_RM_ZLT::WriteSample(BYTE const* pSigma, UINT16 const* pDepth, UINT16 const* pAbImage, LONGLONG timestamp, UINT8* metadata, UINT32 metadata_size)
{
    BYTE* pixelBufferData;
    UINT32 pixelBufferDataLength;
    uint32_t streamSize;

    {
    auto bitmapBuffer = m_softwareBitmap.LockBuffer(BitmapBufferAccessMode::Write);
    auto spMemoryBufferByteAccess = bitmapBuffer.CreateReference().as<IMemoryBufferByteAccess>();

    spMemoryBufferByteAccess->GetBuffer(&pixelBufferData, &pixelBufferDataLength);

    Neon_ZLTToBGRA8(pSigma, pDepth, pAbImage, (u32*)pixelBufferData);
    }

    auto stream = InMemoryRandomAccessStream();
    auto encoder = BitmapEncoder::CreateAsync(BitmapEncoder::PngEncoderId(), stream, m_pngProperties).get();

    encoder.SetSoftwareBitmap(m_softwareBitmap);
    encoder.FlushAsync().get();

    streamSize = (uint32_t)stream.Size();

    auto streamBuf = Buffer(streamSize);

    stream.ReadAsync(streamBuf, streamSize, InputStreamOptions::None).get();

    m_pHookCallback(streamBuf, timestamp, metadata, metadata_size, m_pHookParam);
}
