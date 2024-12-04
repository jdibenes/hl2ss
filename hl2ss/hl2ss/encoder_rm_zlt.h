
#pragma once

#include "custom_encoder.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Graphics.Imaging.h>
#include <winrt/Windows.Storage.Streams.h>

struct RM_ZLT_Metadata
{
    uint64_t sensor_ticks;
    winrt::Windows::Foundation::Numerics::float4x4 pose;
};

class Encoder_RM_ZLT
{
private:
    winrt::Windows::Graphics::Imaging::BitmapPropertySet m_pngProperties;
    winrt::Windows::Graphics::Imaging::SoftwareBitmap m_softwareBitmap;
    HOOK_ENCODER_PROC m_pHookCallback;
    void* m_pHookParam;

    static void ToBGRA8(uint8_t const* pSigma, uint16_t const* pDepth, uint16_t const* pAb, uint32_t* pBGRA8);

public:
    Encoder_RM_ZLT(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, H26xFormat const& format, ZABFormat const& zabFormat);

    void WriteSample(BYTE const* pSigma, UINT16 const* pDepth, UINT16 const* pAbImage, LONGLONG timestamp, RM_ZLT_Metadata* metadata);

    static void SetH26xFormat(H26xFormat& format);
};
