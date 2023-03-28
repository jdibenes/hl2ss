
#pragma once

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.People.h>

int const HAND_JOINTS = 26;

struct SpatialInput_Frame
{
    winrt::Windows::Foundation::Numerics::float3 position;
    winrt::Windows::Foundation::Numerics::float3 forward;
    winrt::Windows::Foundation::Numerics::float3 up;
};

struct SpatialInput_Ray
{
    winrt::Windows::Foundation::Numerics::float3 origin;
    winrt::Windows::Foundation::Numerics::float3 direction;
};

bool SpatialInput_WaitForEyeConsent();
void SpatialInput_Initialize();
int SpatialInput_GetHeadPoseAndEyeRay(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& world, winrt::Windows::Perception::PerceptionTimestamp const& ts, SpatialInput_Frame& head_pose, SpatialInput_Ray& eye_ray);
int SpatialInput_GetHandPose(winrt::Windows::Perception::Spatial::SpatialCoordinateSystem const& world, winrt::Windows::Perception::PerceptionTimestamp const& ts, std::vector<winrt::Windows::Perception::People::JointPose>& left_poses, std::vector<winrt::Windows::Perception::People::JointPose>& right_poses);
