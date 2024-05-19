
#include "custom_audio_effect.h"

using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Media::Effects;

MRCAudioEffect::MRCAudioEffect(MRCAudioOptions const& options)
{
    m_propertySet.Insert(L"MixerMode", winrt::box_value<uint32_t>((options.mixer_mode & 3) % 3));
    m_propertySet.Insert(L"LoopbackGain", winrt::box_value<float>(options.loopback_gain));
    m_propertySet.Insert(L"MicrophoneGain", winrt::box_value<float>(options.microphone_gain));
}

IPropertySet MRCAudioEffect::Properties()
{
    return m_propertySet;
}

winrt::hstring MRCAudioEffect::ActivatableClassId()
{
    return L"Windows.Media.MixedRealityCapture.MixedRealityCaptureAudioEffect";
}
