
#pragma once

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.People.h>

#define HAND_JOINTS 26

struct Frame
{
    bool valid;
    winrt::Windows::Foundation::Numerics::float3 position;
    winrt::Windows::Foundation::Numerics::float3 forward;
    winrt::Windows::Foundation::Numerics::float3 up;
};

struct Ray
{
    bool valid;
    winrt::Windows::Foundation::Numerics::float3 origin;
    winrt::Windows::Foundation::Numerics::float3 direction;
};

bool SpatialInput_WaitForEyeConsent();
void SpatialInput_SetWorldCoordinateSystem(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& world);
void SpatialInput_Initialize();
void SpatialInput_GetHeadPoseAndEyeRay(winrt::Windows::Perception::PerceptionTimestamp const& ts, Frame& head_pose, Ray& eye_ray);

// Hand Tracking is disabled for now
//void SpatialInput_GetHandPose(winrt::Windows::Perception::PerceptionTimestamp const& ts, bool& left, winrt::Windows::Perception::People::JointPose *left_poses, bool& right, winrt::Windows::Perception::People::JointPose *right_poses);
