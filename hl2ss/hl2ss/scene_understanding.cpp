
#include <Windows.h>
#include <unordered_set>
#include "timestamps.h"
#include "locator.h"
#include "scene_understanding.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>
#include <Microsoft.MixedReality.SceneUnderstanding.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;
using namespace Microsoft::MixedReality::SceneUnderstanding;

struct Query
{
    GUID const* guid_match;
    size_t guid_count;
    uint8_t kinds;
};

template<>
struct std::hash<GUID>
{
std::size_t operator()(GUID const& guid) const noexcept
{
    uint8_t hash[8];

    hash[0] = (uint8_t)(guid.Data1);
    hash[1] = (uint8_t)(guid.Data1 >> 8);
    hash[2] = guid.Data4[0];
    hash[3] = (uint8_t)(((guid.Data1 >> 16) & 0x03) | (guid.Data4[1] << 2));

    hash[4] = (uint8_t)(guid.Data1 >> 18);
    hash[5] = (uint8_t)(((guid.Data1 >> 26) & 0x3F) | (guid.Data3 << 6));
    hash[6] = (uint8_t)(guid.Data2);
    hash[7] = (uint8_t)(guid.Data2 >> 8);

    return *((size_t*)hash);
}
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::shared_ptr<PerceptionSceneFactory> g_scene_observer = nullptr;
static HANDLE g_event_consent = NULL; // CloseHandle
static Status g_s = Status::Failed;
static PerceptionSceneFactoryAccessStatus g_psfas = PerceptionSceneFactoryAccessStatus::UserPromptRequired;
static std::shared_ptr<Scene> g_scene = nullptr;
static Query g_query;
static SceneUnderstanding_Result g_result;
static HANDLE g_event_done = NULL; // CloseHandle

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void SceneUnderstanding_Callback_RequestAccess(Status s, PerceptionSceneFactoryAccessStatus psfas)
{
    g_s = s;
    g_psfas = psfas;
    SetEvent(g_event_consent);
}

// OK
void SceneUnderstanding_Initialize()
{
    g_event_consent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_event_done = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_scene_observer = PerceptionSceneFactory::CreatePerceptionSceneFactory();
    g_scene_observer->RequestAccessAsync(SceneUnderstanding_Callback_RequestAccess);
}

// OK
bool SceneUnderstanding_WaitForConsent()
{
    WaitForSingleObject(g_event_consent, INFINITE);
    return (g_s == Status::OK) && (g_psfas == PerceptionSceneFactoryAccessStatus::Allowed);
}

// OK
static uint32_t SceneUnderstanding_KindToFlag(SceneObjectKind kind)
{
    switch (kind)
    {
    case SceneObjectKind::Background:         return SceneUnderstanding_KindFlag::Background;
    case SceneObjectKind::Wall:               return SceneUnderstanding_KindFlag::Wall;
    case SceneObjectKind::Floor:              return SceneUnderstanding_KindFlag::Floor;
    case SceneObjectKind::Ceiling:            return SceneUnderstanding_KindFlag::Ceiling;
    case SceneObjectKind::Platform:           return SceneUnderstanding_KindFlag::Platform;
    case SceneObjectKind::Unknown:            return SceneUnderstanding_KindFlag::Unknown;
    case SceneObjectKind::World:              return SceneUnderstanding_KindFlag::World;
    case SceneObjectKind::CompletelyInferred: return SceneUnderstanding_KindFlag::CompletelyInferred;
    default:                                  return 0;
    }
}

// OK
static void SceneUnderstanding_AdvanceScene(std::shared_ptr<Scene> scene)
{
    PerceptionTimestamp ts = nullptr;
    SpatialCoordinateSystem scs = nullptr;
    std::shared_ptr<SceneComponent> component;
    std::unordered_set<GUID> check;
    GUID guid;

    if (g_scene) { g_scene->Dispose(); }
    g_scene = scene;

    ts = QPCTimestampToPerceptionTimestamp(GetCurrentQPCTimestamp());
    guid = scene->GetOriginSpatialGraphNodeId();
    scs = SpatialGraphInteropPreview::CreateCoordinateSystemForNode(*((winrt::guid*)&guid));
    
    g_result.extrinsics = scene->GetOrigin().CoordinateSystemToNodeTransform;
    g_result.pose = Locator_GetTransformTo(scs, Locator_GetWorldCoordinateSystem(ts));

    for (int i = 0; i < g_query.guid_count; ++i)
    {
    component = scene->FindComponent(g_query.guid_match[i]);
    if (!component || (check.count(component->GetId()) > 0)) { continue; }
    check.insert(component->GetId());
    g_result.selected.push_back(std::dynamic_pointer_cast<SceneObject>(component));
    }

    for (auto const& object : scene->GetSceneObjects()) { if ((SceneUnderstanding_KindToFlag(object->GetKind()) & g_query.kinds) && (check.count(object->GetId()) <= 0)) { g_result.selected.push_back(object); } }
}

// OK
static void SceneUnderstanding_Callback_ComputeAsync(Status s, std::shared_ptr<Scene> scene)
{
    g_result.status = s;
    g_result.selected.clear();
    if (s == Status::OK) { SceneUnderstanding_AdvanceScene(scene); }
    g_result.items = (uint32_t)g_result.selected.size();
    SetEvent(g_event_done);
}

// OK
void SceneUnderstanding_Query(SceneQuerySettings sqs, float query_radius, bool use_previous, GUID const* guid_match, size_t guid_count, uint8_t kind_flags) // IPC
{
    ResetEvent(g_event_done);
    g_query.guid_match = guid_match;
    g_query.guid_count = guid_count;
    g_query.kinds = kind_flags;
    if (use_previous && g_scene) { g_scene_observer->ComputeAsync(sqs, query_radius, g_scene, SceneUnderstanding_Callback_ComputeAsync); } else { g_scene_observer->ComputeAsync(sqs, query_radius, SceneUnderstanding_Callback_ComputeAsync); }
}

// OK
SceneUnderstanding_Result const* SceneUnderstanding_WaitForResult()
{
    WaitForSingleObject(g_event_done, INFINITE);
    return &g_result;
}
