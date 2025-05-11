
#include "encoder_rm_vlc_mosaic.h"
#include "research_mode.h"
#include "timestamp.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

RM_VLC_Mosaic_VideoTransform const Encoder_RM_VLC_Mosaic::m_vt_lut[2] =
{
    {0,                 VideoSubtype::VideoSubtype_L8},
    {RM_VLC_PIXELS / 2, VideoSubtype::VideoSubtype_NV12}
};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
RM_VLC_Mosaic_VideoTransform Encoder_RM_VLC_Mosaic::GetTransform(H26xFormat const& format)
{
    return m_vt_lut[format.profile != H26xProfile::H26xProfile_None];
}

// OK
void Encoder_RM_VLC_Mosaic::SetH26xFormat(H26xFormat& format, int count)
{
    format.width     = RM_VLC_WIDTH;
    format.height    = RM_VLC_HEIGHT * static_cast<uint16_t>(count);
    format.framerate = RM_VLC_FPS;
}

// OK
Encoder_RM_VLC_Mosaic::Encoder_RM_VLC_Mosaic(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, H26xFormat const& format, std::vector<uint64_t> const& options) :
CustomEncoder(pHookCallback, pHookParam, NULL, sizeof(RM_VLC_Mosaic_Metadata), GetTransform(format).subtype, format, RM_VLC_WIDTH, options)
{
    m_lumasize   = RM_VLC_PIXELS;
    m_chromasize = GetTransform(format).chromasize;	
    m_framebytes = m_lumasize + m_chromasize;
    m_duration   = (format.divisor * HNS_BASE) / RM_VLC_FPS;
}

// OK
void Encoder_RM_VLC_Mosaic::WriteSample(BYTE const* const* pImage, int count, LONGLONG timestamp, RM_VLC_Mosaic_Metadata* metadata)
{
    IMFMediaBuffer* pBuffer; // Release
    BYTE* pDst;

    CreateBuffer(&pBuffer, count * m_framebytes);

    pBuffer->Lock(&pDst, NULL, NULL);

    for (int i = 0; i < count; ++i) { memcpy(pDst + (i * m_lumasize), pImage[i], m_lumasize); }
    memset(pDst + (count * m_lumasize), NV12_ZERO_CHROMA, count * m_chromasize);

    pBuffer->Unlock();

    WriteBuffer(pBuffer, timestamp, m_duration, metadata);

    pBuffer->Release();
}
