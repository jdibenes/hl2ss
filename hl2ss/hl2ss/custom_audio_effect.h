
#pragma once

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.Effects.h>

struct MRCAudioOptions
{
    uint32_t mixer_mode;
    float loopback_gain;
    float microphone_gain;
};

struct MRCAudioEffect : winrt::implements<MRCAudioEffect, winrt::Windows::Media::Effects::IAudioEffectDefinition>
{
    winrt::Windows::Foundation::Collections::PropertySet m_propertySet;

    MRCAudioEffect(MRCAudioOptions const& options);

    winrt::hstring ActivatableClassId();
    winrt::Windows::Foundation::Collections::IPropertySet Properties();
};
