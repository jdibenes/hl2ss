
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

public:
    Encoder_RM_ZHT(HOOK_SINK_PROC pHookCallback, void* pHookParam, H26xFormat const& format, ZABFormat const& zabformat, std::vector<uint64_t> const& options);

    void WriteSample(UINT16 const* pDepth, UINT16 const* pAbImage, LONGLONG timestamp, RM_ZHT_SLOT* metadata, UINT32 metadata_size);
};
