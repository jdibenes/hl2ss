
#include "spatial_input.h"
#include "locator.h"
#include "channel.h"
#include "ports.h"
#include "timestamps.h"

#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.People.h>

using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::People;

class Channel_SI : Channel
{
private:
    bool Startup();
    void Run();
    void Cleanup();

public:
    Channel_SI(char const* name, char const* port, uint32_t id);
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static std::unique_ptr<Channel_SI> g_channel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Channel_SI::Channel_SI(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
}

// OK
bool Channel_SI::Startup()
{
    return SpatialInput_WaitForEyeConsent();
}

// OK
void Channel_SI::Run()
{
    int     const hand_size   = HAND_JOINTS * sizeof(JointPose);
    int32_t const packet_size = sizeof(uint32_t) + sizeof(SpatialInput_Frame) + sizeof(SpatialInput_Ray) + (2 * hand_size);

    PerceptionTimestamp ts = nullptr;
    SpatialCoordinateSystem world = nullptr;
    UINT64 qpc;
    uint32_t valid;
    SpatialInput_Frame head_pose;
    SpatialInput_Ray eye_ray;
    std::vector<JointPose> left_poses;
    std::vector<JointPose> right_poses;    
    WSABUF wsaBuf[7];
    bool ok;
    
    left_poses.resize(HAND_JOINTS);
    right_poses.resize(HAND_JOINTS);

    pack_buffer(wsaBuf, 0, &qpc,               sizeof(qpc));
    pack_buffer(wsaBuf, 1, &packet_size,       sizeof(packet_size));
    pack_buffer(wsaBuf, 2, &valid,             sizeof(valid));
    pack_buffer(wsaBuf, 3, &head_pose,         sizeof(head_pose));
    pack_buffer(wsaBuf, 4, &eye_ray,           sizeof(eye_ray));
    pack_buffer(wsaBuf, 5, left_poses.data(),  hand_size);
    pack_buffer(wsaBuf, 6, right_poses.data(), hand_size);

    do
    {
    Sleep(1000 / 30);

    qpc   = GetCurrentQPCTimestamp();
    ts    = QPCTimestampToPerceptionTimestamp(qpc);
    world = Locator_GetWorldCoordinateSystem(ts);

    int status1 = SpatialInput_GetHeadPoseAndEyeRay(world, ts, head_pose, eye_ray);
    int status2 = SpatialInput_GetHandPose(world, ts, left_poses, right_poses);

    valid = status1 | (status2 << 2);

    ok = send_multiple(m_socket_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
    }
    while (ok);
}

// OK
void Channel_SI::Cleanup()
{
}

// OK
void SI_Initialize()
{
    g_channel = std::make_unique<Channel_SI>("SI", PORT_NAME_SI, PORT_NUMBER_SI - PORT_NUMBER_BASE);
}

// OK
void SI_Cleanup()
{
    g_channel.reset();
}
