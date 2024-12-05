
#pragma once

#include "custom_sink_writers.h"

uint8_t const NV12_ZERO_CHROMA = 0x80;

typedef void(*HOOK_ENCODER_PROC)(void*, DWORD, LONGLONG, void*, UINT32, void*);
typedef void(*HOOK_METADATA_PROC)(void*, uint32_t);

class CustomEncoder
{
private:
    std::unique_ptr<CustomSinkWriter> m_pSinkWriter;
    std::unique_ptr<uint8_t[]> m_metadata;
    uint32_t m_metadata_size;
    bool m_shift;
    HOOK_ENCODER_PROC m_pHookCallback;
    void* m_pHookParam;
    HOOK_METADATA_PROC m_pMetadataFree;

    CustomEncoder(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, HOOK_METADATA_PROC pMetadataFree, uint32_t metadata_size, bool shift);

    void ProcessSample(IMFSample* pSample);

    static void SinkThunk(IMFSample* pSample, void* param);

protected:
    CustomEncoder(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, HOOK_METADATA_PROC pMetadataFree, uint32_t metadata_size, AudioSubtype input_subtype, AACFormat  const& format);
    CustomEncoder(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, HOOK_METADATA_PROC pMetadataFree, uint32_t metadata_size, VideoSubtype input_subtype, H26xFormat const& format, uint32_t stride, std::vector<uint64_t> const& encoder_options);
    
    virtual ~CustomEncoder();

    void WriteBuffer(IMFMediaBuffer* pBuffer, LONGLONG timestamp, LONGLONG duration, UINT8* metadata);

    static void CreateBuffer(IMFMediaBuffer** ppBuffer, DWORD size);
};
