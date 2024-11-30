
#include <mfapi.h>
#include "encoder_rm_zht.h"
#include "research_mode.h"
#include "neon.h"
#include "timestamps.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void RM_ZHT_SetZAB(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* out)
{
    Neon_ZHTInvalidate(pDepth, (uint16_t*)out);
    memcpy(out + RM_ZHT_ZSIZE, pAb, RM_ZHT_ABSIZE);
}

// OK
static void RM_ZHT_SetXAB(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* out)
{
    (void)pDepth;
    memcpy(out, pAb, RM_ZHT_ABSIZE);
}

// OK
static void RM_ZHT_ZABToNV12(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* pNV12)
{
    Neon_ZABToNV12(pDepth, pAb, pNV12);
}

// OK
static void RM_ZHT_XABToNV12(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* pNV12)
{
    (void)pDepth;
    Neon_XABToNV12(pAb, pNV12);
}

// OK
static void RM_ZHT_BypassZ(zdepth::DepthCompressor& compressor, uint16_t const* pDepth, std::vector<uint8_t>*& pData, bool keyframe)
{
    (void)compressor;
    (void)pDepth;
    (void)keyframe;
    pData = NULL;
}

// OK
static void RM_ZHT_AppendZ(zdepth::DepthCompressor& compressor, uint16_t const* pDepth, std::vector<uint8_t>*& pData, bool keyframe)
{
    Neon_ZHTInvalidate(pDepth, const_cast<UINT16*>(pDepth));
    pData = new std::vector<uint8_t>(); // delete
    compressor.Compress(RM_ZHT_WIDTH, RM_ZHT_HEIGHT, pDepth, *pData, keyframe);
}

// OK
Encoder_RM_ZHT::Encoder_RM_ZHT(HOOK_SINK_PROC pHookCallback, void* pHookParam, H26xFormat const& format, ZABFormat const& zabformat, std::vector<uint64_t> const& options)
{
    uint32_t pixels = format.width * format.height;
    VideoSubtype subtype;
    
    switch (2 * (zabformat.profile != ZProfile::ZProfile_Same) + (format.profile != H26xProfile::H26xProfile_None))
    {
    case 0:  m_kernel_sink = RM_ZHT_SetZAB;    m_kernel_blob = RM_ZHT_BypassZ; m_framebytes =  pixels * 4;      subtype = VideoSubtype::VideoSubtype_ARGB; break;
    case 1:  m_kernel_sink = RM_ZHT_ZABToNV12; m_kernel_blob = RM_ZHT_BypassZ; m_framebytes = (pixels * 3) / 2; subtype = VideoSubtype::VideoSubtype_NV12; break;
    case 2:  m_kernel_sink = RM_ZHT_SetXAB;    m_kernel_blob = RM_ZHT_AppendZ; m_framebytes =  pixels * 2;      subtype = VideoSubtype::VideoSubtype_L16;  break;
    default: m_kernel_sink = RM_ZHT_XABToNV12; m_kernel_blob = RM_ZHT_AppendZ; m_framebytes = (pixels * 3) / 2; subtype = VideoSubtype::VideoSubtype_NV12; break;
    }

    m_duration = (format.divisor * HNS_BASE) / format.framerate;

    m_pSinkWriter = CustomSinkWriter::CreateForVideo(pHookCallback, pHookParam, subtype, format, format.width, options);
}

// OK
void Encoder_RM_ZHT::WriteSample(UINT16 const* pDepth, UINT16 const* pAbImage, LONGLONG timestamp, RM_ZHT_SLOT* metadata, UINT32 metadata_size)
{
    IMFMediaBuffer* pBuffer; // Release
    IMFSample* pSample; // Release
    BYTE* pDst;

    MFCreateMemoryBuffer(m_framebytes, &pBuffer);

    pBuffer->Lock(&pDst, NULL, NULL);
    m_kernel_sink(pDepth, pAbImage, pDst);
    pBuffer->Unlock();
    pBuffer->SetCurrentLength(m_framebytes);

    m_kernel_blob(m_compressor, pDepth, metadata->z, true);

    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(m_duration);
    pSample->SetSampleTime(timestamp);
    pSample->SetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)metadata, metadata_size);

    m_pSinkWriter->WriteSample(pSample);

    pSample->Release();
    pBuffer->Release();
}
