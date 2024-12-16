
#pragma once

#include <winrt/Windows.Foundation.Numerics.h>
#include <Microsoft.MixedReality.SceneUnderstanding.h>

enum SceneUnderstanding_KindFlag
{
    Background = 1,
    Wall = 2,
    Floor = 4,
    Ceiling = 8,
    Platform = 16,
    Unknown = 32,
    World = 64,
    CompletelyInferred = 128
};

struct SceneUnderstanding_Result
{
    Microsoft::MixedReality::SceneUnderstanding::Status status;
    Microsoft::MixedReality::SceneUnderstanding::Matrix4x4 extrinsics;
    winrt::Windows::Foundation::Numerics::float4x4 pose;
    uint32_t items;
    std::vector<std::shared_ptr<Microsoft::MixedReality::SceneUnderstanding::SceneObject>> selected;
};

void SceneUnderstanding_Startup();
void SceneUnderstanding_Cleanup();
bool SceneUnderstanding_WaitForConsent();
void SceneUnderstanding_Query(Microsoft::MixedReality::SceneUnderstanding::SceneQuerySettings sqs, float query_radius, bool use_previous, GUID const* guid_match, size_t guid_count, uint8_t kind_flags);
SceneUnderstanding_Result const* SceneUnderstanding_WaitForResult();
