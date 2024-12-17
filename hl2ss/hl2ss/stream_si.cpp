
#include "spatial_input.h"
#include "server_channel.h"

#include <winrt/Windows.Perception.People.h>

using namespace winrt::Windows::Perception::People;

class Channel_SI : Channel
{
private:
    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode0();

    void OnFrameArrived(uint32_t valid, SpatialInput_Frame* head_pose, SpatialInput_Ray* eye_ray, JointPose* left_hand, JointPose* right_hand, UINT64 timestamp);

    static void Thunk_Sensor(uint32_t valid, SpatialInput_Frame* head_pose, SpatialInput_Ray* eye_ray, JointPose* left_hand, JointPose* right_hand, UINT64 timestamp, void* self);

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
void Channel_SI::Thunk_Sensor(uint32_t valid, SpatialInput_Frame* head_pose, SpatialInput_Ray* eye_ray, JointPose* left_hand, JointPose* right_hand, UINT64 timestamp, void* self)
{
    static_cast<Channel_SI*>(self)->OnFrameArrived(valid, head_pose, eye_ray, left_hand, right_hand, timestamp);
}

// OK
void Channel_SI::OnFrameArrived(uint32_t valid, SpatialInput_Frame* head_pose, SpatialInput_Ray* eye_ray, JointPose* left_hand, JointPose* right_hand, UINT64 timestamp)
{
    int32_t const hand_size   = HAND_JOINTS * sizeof(JointPose);
    int32_t const packet_size = sizeof(uint32_t) + sizeof(SpatialInput_Frame) + sizeof(SpatialInput_Ray) + (2 * hand_size);

    WSABUF wsaBuf[7];

    pack_buffer(wsaBuf, 0, &timestamp,   sizeof(timestamp));
    pack_buffer(wsaBuf, 1, &packet_size, sizeof(packet_size));
    pack_buffer(wsaBuf, 2, &valid,       sizeof(uint32_t));
    pack_buffer(wsaBuf, 3, head_pose,    sizeof(SpatialInput_Frame));
    pack_buffer(wsaBuf, 4, eye_ray,      sizeof(SpatialInput_Ray));
    pack_buffer(wsaBuf, 5, left_hand,    hand_size);
    pack_buffer(wsaBuf, 6, right_hand,   hand_size);

    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_SI::Execute_Mode0()
{
    SpatialInput_ExecuteSensorLoop(Thunk_Sensor, this, m_event_client);
}

// OK
Channel_SI::Channel_SI(char const* name, char const* port, uint32_t id) :
Channel(name, port, id)
{
}

// OK
bool Channel_SI::Startup()
{
    SetNoDelay(true);
    return SpatialInput_WaitForConsent();
}

// OK
void Channel_SI::Run()
{
    Execute_Mode0();
}

// OK
void Channel_SI::Cleanup()
{
}

// OK
void SI_Startup()
{
    g_channel = std::make_unique<Channel_SI>("SI", PORT_NAME_SI, PORT_ID_SI);
}

// OK
void SI_Cleanup()
{
    g_channel.reset();
}
