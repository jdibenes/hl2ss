
#include "extended_execution.h"
#include "locator.h"
#include "timestamp.h"
#include "spatial_input.h"
#include "lock.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.UI.Input.h>
#include <winrt/Windows.UI.Input.Spatial.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.People.h>

using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::UI::Input;
using namespace winrt::Windows::UI::Input::Spatial;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::People;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static SpatialInteractionManager g_sim = nullptr;
static std::vector<HandJointKind> g_joints;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
bool SpatialInput_WaitForConsent()
{
    Cleaner log_error_eyetracker([=]() { ExtendedExecution_EnterException(Exception::Exception_AccessDeniedEyeTracker); });
    if (EyesPose::RequestAccessAsync().get() != GazeInputAccessStatus::Allowed) { return false; }
    log_error_eyetracker.Set(false);
    return true;
}

// OK
void SpatialInput_Startup()
{
    g_joints.resize(HAND_JOINTS);
    for (int i = 0; i < HAND_JOINTS; ++i) { g_joints[i] = (HandJointKind)i; }

    g_sim = SpatialInteractionManager::GetForCurrentView();
}

// OK
void SpatialInput_Cleanup()
{
    g_sim = nullptr;
}

// OK
static int SpatialInput_GetHeadPoseAndEyeRay(SpatialCoordinateSystem const& world, UINT64 qpc, SpatialInput_Frame& head_pose, SpatialInput_Ray& eye_ray)
{
    auto pointer = SpatialPointerPose::TryGetAtTimestamp(world, Timestamp_QPCToPerception(qpc));
    int ret = 0;

    if (!pointer) { return ret; }

    auto h = pointer.Head();

    head_pose.position = h.Position();
    head_pose.forward  = h.ForwardDirection();
    head_pose.up       = h.UpDirection();

    ret |= 1;

    auto pose = pointer.Eyes();
    if (!pose) { return ret; }

    auto gaze = pose.Gaze();
    if (!gaze) { return ret; }

    auto spatialRay = gaze.Value();

    eye_ray.origin    = spatialRay.Origin;
    eye_ray.direction = spatialRay.Direction;

    ret |= 2;

    return ret;
}

// OK
static int SpatialInput_GetHandPose(SpatialCoordinateSystem const& world, UINT64 qpc, std::vector<JointPose>& left_poses, std::vector<JointPose>& right_poses)
{
    auto source_states = g_sim.GetDetectedSourcesAtTimestamp(Timestamp_QPCToPerception(qpc));
    int ret = 0;
    std::vector<JointPose>* target;
    int id;
    bool ok;

    for (auto const& source_state : source_states)
    {
    if (source_state.Source().Kind() != SpatialInteractionSourceKind::Hand) { continue; }

    switch (source_state.Source().Handedness())
    {
    case SpatialInteractionSourceHandedness::Left:  target = &left_poses;  id = 1; break;
    case SpatialInteractionSourceHandedness::Right: target = &right_poses; id = 2; break;
    default: continue;
    }

    auto pose = source_state.TryGetHandPose();
    if (!pose) { continue; }

    ok = pose.TryGetJoints(world, g_joints, *target);
    if (!ok) { continue; }

    ret |= id;
    }

    return ret;
}

// OK
void SpatialInput_ExecuteSensorLoop(HOOK_SI_PROC hook, void* param, HANDLE event_stop)
{
    HANDLE timer = CreateWaitableTimer(NULL, TRUE, NULL); // CloseHandle
    LARGE_INTEGER delay;
    SpatialInput_Frame head_pose;
    SpatialInput_Ray eye_ray;
    std::vector<JointPose> l_hand;
    std::vector<JointPose> r_hand;

    delay.QuadPart = -static_cast<int64_t>(HNS_BASE / 60);

    l_hand.resize(HAND_JOINTS);
    r_hand.resize(HAND_JOINTS);
    
    do
    {
    SetWaitableTimer(timer, &delay, 0, NULL, NULL, 0);

    UINT64 qpc = Timestamp_GetCurrentQPC();
    auto world = Locator_GetWorldCoordinateSystem();

    int status1 = SpatialInput_GetHeadPoseAndEyeRay(world, qpc, head_pose, eye_ray);
    int status2 = SpatialInput_GetHandPose(world, qpc, l_hand, r_hand);

    hook((status1 << 0) | (status2 << 2), &head_pose, &eye_ray, l_hand.data(), r_hand.data(), qpc, param);

    WaitForSingleObject(timer, INFINITE);
    }
    while (WaitForSingleObject(event_stop, 0) == WAIT_TIMEOUT);

    CloseHandle(timer);
}
