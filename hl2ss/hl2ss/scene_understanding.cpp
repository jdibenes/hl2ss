
#include <unordered_set>
#include "locator.h"
#include "timestamp.h"
#include "scene_understanding.h"

#include <winrt/Windows.Perception.Spatial.Preview.h>
#include <Microsoft.MixedReality.SceneUnderstanding.h>

using namespace winrt::Windows::Perception::Spatial::Preview;
using namespace Microsoft::MixedReality::SceneUnderstanding;

struct SU_Query
{
    GUID const* guid_match;
    size_t guid_count;
    uint8_t kind_flags;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event_consent = NULL; // CloseHandle
static HANDLE g_event_done = NULL; // CloseHandle
static std::shared_ptr<PerceptionSceneFactory> g_scene_observer = nullptr;
static Status g_s = Status::Failed;
static PerceptionSceneFactoryAccessStatus g_psfas = PerceptionSceneFactoryAccessStatus::UserPromptRequired;
static std::shared_ptr<Scene> g_scene = nullptr;
static SU_Query g_query;
static SceneUnderstanding_Result g_result;

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
static void SceneUnderstanding_ShiftScene(std::shared_ptr<Scene> scene)
{
    if (g_scene) { g_scene->Dispose(); }
    g_scene = scene;
}

// OK
void SceneUnderstanding_Startup()
{
    g_event_consent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_event_done = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_scene_observer = PerceptionSceneFactory::CreatePerceptionSceneFactory();
    g_scene_observer->RequestAccessAsync(SceneUnderstanding_Callback_RequestAccess);
}

// OK
void SceneUnderstanding_Cleanup()
{
    g_result.selected.clear();
    SceneUnderstanding_ShiftScene(nullptr);
    g_scene_observer = nullptr;
    CloseHandle(g_event_done);
    CloseHandle(g_event_consent);
    g_event_done = NULL;
    g_event_consent = NULL;
    g_s = Status::Failed;
    g_psfas = PerceptionSceneFactoryAccessStatus::UserPromptRequired;
}

// OK
bool SceneUnderstanding_WaitForConsent()
{
    WaitForSingleObject(g_event_consent, INFINITE);
    return (g_s == Status::OK) && (g_psfas == PerceptionSceneFactoryAccessStatus::Allowed);
}

// OK
static void SceneUnderstanding_AdvanceScene(std::shared_ptr<Scene> scene)
{
    std::shared_ptr<SceneComponent> component;
    std::unordered_set<SceneComponent*> check;

    SceneUnderstanding_ShiftScene(scene);

    g_result.extrinsics = scene->GetOrigin().CoordinateSystemToNodeTransform;
    g_result.pose       = Locator_GetTransformTo(SpatialGraphInteropPreview::CreateCoordinateSystemForNode(winrt::guid(scene->GetOriginSpatialGraphNodeId())), Locator_GetWorldCoordinateSystem());

    for (int i = 0; i < g_query.guid_count; ++i)
    {
    component = scene->FindComponent(g_query.guid_match[i]);
    if (!component)                       { continue; }
    if (check.count(component.get()) > 0) { continue; }
    check.insert(component.get());
    g_result.selected.push_back(std::dynamic_pointer_cast<SceneObject>(component));
    }

    for (auto const& object : scene->GetSceneObjects())
    {
    if ((SceneUnderstanding_KindToFlag(object->GetKind()) & g_query.kind_flags) == 0) { continue; }
    if (check.count(object.get()) > 0)                                                { continue; }
    check.insert(object.get());
    g_result.selected.push_back(object);
    }
}

// OK
static void SceneUnderstanding_Callback_ComputeAsync(Status s, std::shared_ptr<Scene> scene)
{
    g_result.status = s;
    g_result.selected.clear();
    if (s == Status::OK) { SceneUnderstanding_AdvanceScene(scene); }
    g_result.items = static_cast<uint32_t>(g_result.selected.size());

    SetEvent(g_event_done);
}

// OK
void SceneUnderstanding_Query(SceneQuerySettings sqs, float query_radius, bool use_previous, GUID const* guid_match, size_t guid_count, uint8_t kind_flags)
{
    ResetEvent(g_event_done);

    g_query.guid_match = guid_match;
    g_query.guid_count = guid_count;
    g_query.kind_flags = kind_flags;

    if (use_previous && g_scene) { g_scene_observer->ComputeAsync(sqs, query_radius, g_scene, SceneUnderstanding_Callback_ComputeAsync); } 
    else                         { g_scene_observer->ComputeAsync(sqs, query_radius,          SceneUnderstanding_Callback_ComputeAsync); }
}

// OK
SceneUnderstanding_Result const* SceneUnderstanding_WaitForResult()
{
    WaitForSingleObject(g_event_done, INFINITE);
    return &g_result;
}
