
#pragma once

#include "custom_sink_writers.h"

#include <winrt/Windows.Graphics.Imaging.h>
#include <winrt/Windows.Storage.Streams.h>

typedef void(*HOOK_RM_ZLT_PROC)(winrt::Windows::Storage::Streams::Buffer&, LONGLONG, UINT8*, UINT32, void*);

class Encoder_RM_ZLT
{
private:
    winrt::Windows::Graphics::Imaging::BitmapPropertySet m_pngProperties;
    winrt::Windows::Graphics::Imaging::SoftwareBitmap m_softwareBitmap;
    HOOK_RM_ZLT_PROC m_pHookCallback;
    void* m_pHookParam;

    static void ToBGRA8(uint8_t const* pSigma, uint16_t const* pDepth, uint16_t const* pAb, uint32_t* pBGRA8);

public:
    Encoder_RM_ZLT(HOOK_RM_ZLT_PROC pHookCallback, void* pHookParam, H26xFormat& format, ZABFormat& zabFormat);

    void WriteSample(BYTE const* pSigma, UINT16 const* pDepth, UINT16 const* pAbImage, LONGLONG timestamp, UINT8* metadata, UINT32 metadata_size);
};
