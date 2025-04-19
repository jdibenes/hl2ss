
#pragma once

#include "queue.h"
#include "custom_sink_writers.h"
#include "custom_sample_buffer.h"

uint8_t const NV12_ZERO_CHROMA = 0x80;

typedef void(*HOOK_ENCODER_PROC)(void*, DWORD, UINT32, LONGLONG, void*, UINT32, void*);
typedef void(*HOOK_METADATA_PROC)(void*, uint32_t);

class CustomEncoder
{
private:
    std::unique_ptr<CustomSinkWriter> m_pSinkWriter;
    std::unique_ptr<uint8_t[]> m_metadata;
    uint32_t m_metadata_size;
    HOOK_ENCODER_PROC m_pHookCallback;
    void* m_pHookParam;
    HOOK_METADATA_PROC m_pMetadataFree;
    SampleBuffer m_buffer_i;
    SampleBuffer m_buffer_o;

    void WriteSample(IMFSample* pSample);
    void ReceiveSample(IMFSample* pSample);
    void ProcessSample(IMFSample* pSample);
    
    static void Thunk_Write(IMFSample* pSample, void* self);
    static void Thunk_Sink(IMFSample* pSample, void* self);
    static void Thunk_Process(IMFSample* pSample, void* self);

protected:
    CustomEncoder(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, HOOK_METADATA_PROC pMetadataFree, uint32_t metadata_size);
    CustomEncoder(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, HOOK_METADATA_PROC pMetadataFree, uint32_t metadata_size, AudioSubtype input_subtype, AACFormat  const& format);
    CustomEncoder(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, HOOK_METADATA_PROC pMetadataFree, uint32_t metadata_size, VideoSubtype input_subtype, H26xFormat const& format, uint32_t stride, std::vector<uint64_t> const& encoder_options);
    
    virtual ~CustomEncoder();

    void WriteBuffer(IMFMediaBuffer* pBuffer, LONGLONG timestamp, LONGLONG duration, void* metadata);

    static void CreateBuffer(IMFMediaBuffer** ppBuffer, DWORD size);
};

void CustomEncoder_Startup();
void CustomEncoder_Cleanup();
