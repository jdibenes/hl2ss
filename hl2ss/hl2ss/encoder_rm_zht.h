
#pragma once

#include <zdepth.hpp>
#include "custom_encoder.h"

#include <winrt/Windows.Foundation.Numerics.h>

typedef void(*RM_ZXX_KERNEL)(zdepth::DepthCompressor&, uint16_t const*, std::vector<uint8_t>*&, bool);
typedef void(*RM_ZAB_KERNEL)(uint16_t const*, uint16_t const*, uint8_t*);

struct RM_ZHT_Metadata
{
    uint64_t sensor_ticks;
    std::vector<uint8_t>* z;
    uint64_t timestamp;
    winrt::Windows::Foundation::Numerics::float4x4 pose;
};

struct RM_ZHT_VideoTransform
{
    RM_ZXX_KERNEL zxx_kernel;
    RM_ZAB_KERNEL zab_kernel;
    uint32_t framesize;
    VideoSubtype subtype;
};

class Encoder_RM_ZHT : public CustomEncoder
{
private:
    zdepth::DepthCompressor m_compressor;
    RM_ZXX_KERNEL m_kernel_blob;
    RM_ZAB_KERNEL m_kernel_sink;    
    uint32_t m_framebytes;
    LONGLONG m_duration;

    static RM_ZHT_VideoTransform const m_vt_lut[4];

    static void InvalidateZ(uint16_t const* pDepth, uint16_t* pOut);
    static void XABToNV12(uint16_t const* pAb, uint8_t* pOut);
    static void ZABToNV12(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* pNV12);
    static void SetZAB(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* out);
    static void SetXAB(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* out);
    static void SetZABToNV12(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* pNV12);
    static void SetXABToNV12(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* pNV12);
    static void BypassZ(zdepth::DepthCompressor& compressor, uint16_t const* pDepth, std::vector<uint8_t>*& pData, bool keyframe);
    static void AppendZ(zdepth::DepthCompressor& compressor, uint16_t const* pDepth, std::vector<uint8_t>*& pData, bool keyframe);
    static void FreeMetadata(void* metadata, uint32_t metadata_size);

    static RM_ZHT_VideoTransform GetTransform(H26xFormat const& format, ZABFormat const& zabformat);

public:
    Encoder_RM_ZHT(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, H26xFormat const& format, ZABFormat const& zabformat, std::vector<uint64_t> const& options);

    void WriteSample(UINT16 const* pDepth, UINT16 const* pAbImage, LONGLONG timestamp, RM_ZHT_Metadata* metadata);

    static void SetH26xFormat(H26xFormat& format);
};
