
//#define ENABLE_QRCODE_API 1

// TODO:
// locate RM
// Send RM intrinsics/extrinsics
// 

#include "Cannon/MixedReality.h"
#include "Cannon/TrackedHands.h"
#include "utilities.h"

#include <winrt/Windows.Perception.Spatial.h>

using namespace winrt::Windows::Perception::Spatial;




__declspec(align(16))
struct HeTHaTEyeFrame
{
    DirectX::XMMATRIX headTransform;
    std::array<DirectX::XMMATRIX, (size_t)HandJointIndex::Count> leftHandTransform;
    std::array<DirectX::XMMATRIX, (size_t)HandJointIndex::Count> rightHandTransform;
    DirectX::XMVECTOR eyeGazeOrigin;
    DirectX::XMVECTOR eyeGazeDirection;
    float eyeGazeDistance;
    bool leftHandPresent;
    bool rightHandPresent;
    bool eyeGazePresent;
    long long timestamp;
};

static MixedReality g_mixedreality;
static TrackedHands g_hands;





SpatialCoordinateSystem MR_GetWorldCoordinateSystem()
{
    return g_mixedreality.GetWorldCoordinateSystem();
}

bool MR_Initialize()
{
    return true;
    bool ok;

    //ok = ;
    //if (!ok) { return false; }

    //if (!MixedReality::IsAvailable())
    //    DrawCall::InitializeSwapChain((unsigned)window.Bounds().Width, (unsigned)window.Bounds().Height, window);

    ok = g_mixedreality.EnableMixedReality();
    if (!ok) { return false; }

    //g_mixedreality.EnableSurfaceMapping();
    //g_mixedreality.EnableQRCodeTracking();
    //g_mixedreality.EnableEyeTracking();

    return true;
    
}

void MR_Stream()
{
    sizeof(DirectX::XMMATRIX);
}



void MR_Update()
{
    return;

    g_mixedreality.Update();
    g_hands.UpdateFromMixedReality(g_mixedreality);

    XMVECTOR headPosition = g_mixedreality.GetHeadPosition();
    XMVECTOR headforward = g_mixedreality.GetHeadForwardDirection();
    XMVECTOR headup = g_mixedreality.GetHeadUpDirection();
    XMVECTOR headright = XMVector3Cross(headup, -headforward);

    HeTHaTEyeFrame frame;
    frame.headTransform = g_hands.GetHeadTransform();
    
    for (int j = 0; j < (int)HandJointIndex::Count; ++j)
    {
        frame.leftHandTransform[j] = g_hands.GetOrientedJoint(0, HandJointIndex(j));
        frame.rightHandTransform[j] = g_hands.GetOrientedJoint(1, HandJointIndex(j));
    }

    frame.leftHandPresent = g_hands.IsHandTracked(0);
    frame.rightHandPresent = g_hands.IsHandTracked(1);

    //ShowMessage("left  hand present %d", frame.leftHandPresent);
    //ShowMessage("right hand present %d", frame.rightHandPresent);


    frame.timestamp = g_mixedreality.GetPredictedDisplayTime();

    if (g_mixedreality.IsEyeTrackingEnabled() && g_mixedreality.IsEyeTrackingActive())
    {
        frame.eyeGazePresent = true;
        frame.eyeGazeOrigin = g_mixedreality.GetEyeGazeOrigin();
        frame.eyeGazeDirection = g_mixedreality.GetEyeGazeDirection();
        frame.eyeGazeDistance = 0.0f;

        if (g_mixedreality.IsSurfaceMappingActive())
        {
            float distance;
            XMVECTOR normal;
            if (g_mixedreality.GetSurfaceMappingInterface()->TestRayIntersection(frame.eyeGazeOrigin, frame.eyeGazeDirection, distance, normal))
            {
                frame.eyeGazeDistance = distance;
            }
        }
    }
    else
    {
        frame.eyeGazePresent = false;
    }

    //ShowMessage("eye gaze present % d", frame.eyeGazePresent);

    //g_mixedreality.GetCameraPoseCount();
    //g_mixedreality.

    //;
    //;

    

}