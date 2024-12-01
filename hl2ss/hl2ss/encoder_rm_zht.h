
#pragma once

#include <zdepth.hpp>
#include "custom_sink_writers.h"

typedef void(*RM_ZAB_KERNEL)(uint16_t const*, uint16_t const*, uint8_t*);
typedef void(*RM_ZXX_KERNEL)(zdepth::DepthCompressor&, uint16_t const*, std::vector<uint8_t>*&, bool);

struct RM_ZHT_SLOT
{
    std::vector<uint8_t>* z;
};

class Encoder_RM_ZHT
{
private:
    std::unique_ptr<CustomSinkWriter> m_pSinkWriter;
    zdepth::DepthCompressor m_compressor;
    RM_ZAB_KERNEL m_kernel_sink;
    RM_ZXX_KERNEL m_kernel_blob;
    uint32_t m_framebytes;
    LONGLONG m_duration;

    static void InvalidateZ(uint16_t const* pDepth, uint16_t* pOut);
    static void XABToNV12(uint16_t const* pAb, uint8_t* pOut);
    static void ZABToNV12(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* pNV12);

    static void SetZAB(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* out);
    static void SetXAB(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* out);
    static void SetZABToNV12(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* pNV12);
    static void SetXABToNV12(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* pNV12);

    static void BypassZ(zdepth::DepthCompressor& compressor, uint16_t const* pDepth, std::vector<uint8_t>*& pData, bool keyframe);
    static void AppendZ(zdepth::DepthCompressor& compressor, uint16_t const* pDepth, std::vector<uint8_t>*& pData, bool keyframe);

public:
    Encoder_RM_ZHT(HOOK_SINK_PROC pHookCallback, void* pHookParam, H26xFormat& format, ZABFormat& zabformat, std::vector<uint64_t> const& options);

    void WriteSample(UINT16 const* pDepth, UINT16 const* pAbImage, LONGLONG timestamp, RM_ZHT_SLOT* metadata, UINT32 metadata_size);
};
