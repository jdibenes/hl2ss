
#include "custom_video_effect.h"

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Effects;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
MRCVideoEffect::MRCVideoEffect(MRCVideoOptions const& options)
{
    m_propertySet.Insert(L"StreamType", winrt::box_value<uint32_t>(static_cast<uint32_t>(MediaStreamType::VideoRecord)));
    m_propertySet.Insert(L"HologramCompositionEnabled", winrt::box_value<bool>(options.hologram_composition));
    m_propertySet.Insert(L"RecordingIndicatorEnabled", winrt::box_value<bool>(options.recording_indicator));
    m_propertySet.Insert(L"VideoStabilizationEnabled", winrt::box_value<bool>(options.video_stabilization));
    m_propertySet.Insert(L"VideoStabilizationBufferLength", winrt::box_value<uint32_t>(options.video_stabilization_length));
    m_propertySet.Insert(L"GlobalOpacityCoefficient", winrt::box_value<float>(options.global_opacity));
    m_propertySet.Insert(L"BlankOnProtectedContent", winrt::box_value<bool>(options.blank_protected));
    m_propertySet.Insert(L"ShowHiddenMesh", winrt::box_value<bool>(options.show_mesh));
    m_propertySet.Insert(L"OutputSize", winrt::box_value(Size(options.output_width, options.output_height)));
    m_propertySet.Insert(L"OutputSubtype", winrt::box_value(winrt::hstring(L"Nv12")));
    m_propertySet.Insert(L"PreferredHologramPerspective", winrt::box_value<uint32_t>(options.hologram_perspective));
}

// OK
IPropertySet MRCVideoEffect::Properties()
{
    return m_propertySet;
}

// OK
winrt::hstring MRCVideoEffect::ActivatableClassId()
{
    return L"Windows.Media.MixedRealityCapture.MixedRealityCaptureVideoEffect";
}
