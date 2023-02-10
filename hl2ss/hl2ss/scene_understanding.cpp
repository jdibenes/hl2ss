
#include <Windows.h>
#include "log.h"

#include <Microsoft.MixedReality.SceneUnderstanding.h>

using namespace Microsoft::MixedReality::SceneUnderstanding;

static std::shared_ptr<PerceptionSceneFactory> g_scene_observer;
static HANDLE g_event_consent = NULL; // CloseHandle
static Status g_s = Status::Failed;
static PerceptionSceneFactoryAccessStatus g_psfas = PerceptionSceneFactoryAccessStatus::UserPromptRequired;
static std::list<int> g_list; // ???

static void SceneUnderstanding_Callback_RequestAccess(Status s, PerceptionSceneFactoryAccessStatus psfas)
{
    g_s = s;
    g_psfas  = psfas;
    SetEvent(g_event_consent);
}

void SceneUnderstanding_Initialize()
{
    g_event_consent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_scene_observer = PerceptionSceneFactory::CreatePerceptionSceneFactory();
    g_scene_observer->RequestAccessAsync(SceneUnderstanding_Callback_RequestAccess);
}

bool SceneUnderstanding_WaitForConsent()
{
    WaitForSingleObject(g_event_consent, INFINITE);
    return (g_s == Status::OK) && (g_psfas == PerceptionSceneFactoryAccessStatus::Allowed);
}

static void SceneUnderstanding_Callback_ComputeAsync(Status s, std::shared_ptr<Scene> scene)
{
    // ???
}

void SceneUnderstanding_Query() // IPC
{
    SceneQuerySettings sqs;

    sqs.EnableSceneObjectQuads = true;
    sqs.EnableSceneObjectMeshes = true;
    sqs.EnableOnlyObservedSceneObjects = false;
    sqs.EnableWorldMesh = true;
    sqs.RequestedMeshLevelOfDetail = SceneMeshLevelOfDetail::Fine;

    g_scene_observer->ComputeAsync(sqs, 10.0f, SceneUnderstanding_Callback_ComputeAsync);
    // g_scene_observer->ComputeWithBoundsAsync() // ???
    // also with previous scene
}
