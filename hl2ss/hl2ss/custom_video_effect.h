
#pragma once

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.Effects.h>

struct MRCOptions
{
    bool enable;
    bool hologram_composition;
    bool recording_indicator;
    bool video_stabilization;
    bool blank_protected;
    bool show_mesh;
    uint8_t _reserved[2];
    float global_opacity;
    float output_width;
    float output_height;
    uint32_t video_stabilization_length;
    uint32_t hologram_perspective;
};

struct MRCVideoEffect : winrt::implements<MRCVideoEffect, winrt::Windows::Media::Effects::IVideoEffectDefinition>
{
    winrt::Windows::Foundation::Collections::PropertySet m_propertySet;

    MRCVideoEffect(MRCOptions const& options);

    winrt::hstring ActivatableClassId();
    winrt::Windows::Foundation::Collections::IPropertySet Properties();
};
