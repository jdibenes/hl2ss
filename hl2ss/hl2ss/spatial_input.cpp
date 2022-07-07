
#include "locator.h"
#include "utilities.h"
#include "spatial_input.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.UI.Input.h>
#include <winrt/Windows.UI.Input.Spatial.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.People.h>
#include <winrt/Windows.Graphics.Holographic.h>
#include <winrt/Windows.UI.Core.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::UI::Input;
using namespace winrt::Windows::UI::Input::Spatial;
using namespace winrt::Windows::UI::Core;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::People;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Graphics::Holographic;

static HANDLE g_event_consent = NULL;
static HANDLE g_thread_consent = NULL;
static GazeInputAccessStatus g_status_consent = GazeInputAccessStatus::Unspecified;
static SpatialCoordinateSystem g_world = nullptr;
static SpatialInteractionManager g_sim = nullptr;
static std::vector<HandJointKind> g_joints;

// Hand Tracking is disabled for now
//static HolographicSpace g_space = nullptr;
//static HolographicFrame g_frame = nullptr;

static DWORD WINAPI SpatialInput_RequestEyeAccess(void* param)
{
    (void)param;

    g_status_consent = EyesPose::RequestAccessAsync().get();
    SetEvent(g_event_consent);

    return 0;
}

bool SpatialInput_WaitForEyeConsent()
{
    WaitForSingleObject(g_event_consent, INFINITE);
    return g_status_consent == GazeInputAccessStatus::Allowed;
}

void SpatialInput_SetWorldCoordinateSystem(SpatialCoordinateSystem const& world)
{
    g_world = world;
}

void SpatialInput_Initialize()
{
    g_event_consent = CreateEvent(NULL, TRUE, FALSE, NULL);
    g_thread_consent = CreateThread(NULL, 0, SpatialInput_RequestEyeAccess, NULL, 0, NULL);

    // Hand Tracking requires creating a Holographic Space which is kind of expensive
    //g_space = HolographicSpace::CreateForCoreWindow(window);
    //g_sim = SpatialInteractionManager::GetForCurrentView();

    g_joints.resize(HAND_JOINTS);
    for (int i = 0; i < HAND_JOINTS; ++i) { g_joints[i] = (HandJointKind)i; }
}

void SpatialInput_GetHeadPoseAndEyeRay(PerceptionTimestamp const& ts, Frame& head_pose, Ray& eye_ray)
{
    SpatialPointerPose pointer = SpatialPointerPose::TryGetAtTimestamp(g_world, ts);

    head_pose.valid = false;
    eye_ray.valid = false;

    if (!pointer) { return; }

    auto h = pointer.Head();

    head_pose.position = h.Position();
    head_pose.forward  = h.ForwardDirection();
    head_pose.up       = h.UpDirection();

    head_pose.valid = true;

    auto pose = pointer.Eyes();
    if (!pose) { return; }

    auto gaze = pose.Gaze();
    if (!gaze) { return; }

    auto spatialRay = gaze.Value();

    eye_ray.origin    = spatialRay.Origin;
    eye_ray.direction = spatialRay.Direction;

    eye_ray.valid = true;
}

// Hand Tracking is disabled for now
/*
void SpatialInput_GetHandPose(PerceptionTimestamp const& ts, bool& left, JointPose *left_poses, bool& right, JointPose *right_poses)
{
    //auto frame = g_space.CreateNextFrame();
    //auto prediction = frame.CurrentPrediction();

    auto source_states = g_sim.GetDetectedSourcesAtTimestamp(prediction.Timestamp());
    //auto sim = SpatialInteractionManager::GetForCurrentView();
    //auto source_states = sim.GetDetectedSourcesAtTimestamp(ts);
    bool* flag;
    JointPose* target;
    bool ok;

    std::vector<JointPose> jointposes(26);

    left = false;
    right = false;

    ShowMessage("Sources detected: %d", (int)source_states.Size());

    for (auto source_state : source_states)
    {
        if (source_state.Source().Kind() == SpatialInteractionSourceKind::Hand)
        {
            ShowMessage("HAND");
            switch (source_state.Source().Handedness())
            {
            case SpatialInteractionSourceHandedness::Left:  ShowMessage("LEFT");  break;
            case SpatialInteractionSourceHandedness::Right: ShowMessage("RIGHT"); break;
            default: ShowMessage("OTHER");
            }
        }
        else
        {
            ShowMessage("Other SOURCE");
        }

    //auto pose = source_state.TryGetHandPose();
    //if (!pose) { continue; }

    //jointposes.clear();
   // ok = pose.TryGetJoints(g_world, g_joints, jointposes);
    //if (!ok) { continue; }

    //switch (source_state.Source().Handedness())
    //{
    //case SpatialInteractionSourceHandedness::Left:  flag = &left;  target = left_poses;  break;
    //case SpatialInteractionSourceHandedness::Right: flag = &right; target = right_poses; break;
    //default: continue;
    //}

    //for (int i = 0; i < 26; ++i) { target[i] = jointposes[i]; }
    //*flag = true;

    
    }
}
*/
