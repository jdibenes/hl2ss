
#include "spatial_input.h"
#include "server_channel.h"
#include "encoder_si.h"

#include <winrt/Windows.Perception.People.h>

using namespace winrt::Windows::Perception::People;

class Channel_SI : Channel
{
private:
    std::unique_ptr<Encoder_SI> m_pEncoder;

    bool Startup();
    void Run();
    void Cleanup();

    void Execute_Mode0();

    void OnFrameArrived(uint32_t valid, SpatialInput_Frame* head_pose, SpatialInput_Ray* eye_ray, JointPose* left_hand, JointPose* right_hand, UINT64 timestamp);
    void OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size);

    static void Thunk_Sensor(uint32_t valid, SpatialInput_Frame* head_pose, SpatialInput_Ray* eye_ray, JointPose* left_hand, JointPose* right_hand, UINT64 timestamp, void* self);
    static void Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self);

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
void Channel_SI::Thunk_Encoder(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size, void* self)
{
    static_cast<Channel_SI*>(self)->OnEncodingComplete(encoded, encoded_size, clean_point, sample_time, metadata, metadata_size);
}

// OK
void Channel_SI::OnFrameArrived(uint32_t valid, SpatialInput_Frame* head_pose, SpatialInput_Ray* eye_ray, JointPose* left_hand, JointPose* right_hand, UINT64 timestamp)
{
    m_pEncoder->WriteSample(valid, head_pose, eye_ray, left_hand, right_hand, timestamp);
}

// OK
void Channel_SI::OnEncodingComplete(void* encoded, DWORD encoded_size, UINT32 clean_point, LONGLONG sample_time, void* metadata, UINT32 metadata_size)
{
    (void)clean_point;
    (void)sample_time;
    (void)metadata;
    (void)metadata_size;

    WSABUF wsaBuf[1];
    pack_buffer(wsaBuf, 0, encoded, encoded_size);
    send_multiple(m_socket_client, m_event_client, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));
}

// OK
void Channel_SI::Execute_Mode0()
{
    m_pEncoder = std::make_unique<Encoder_SI>(Thunk_Encoder, this);

    SpatialInput_ExecuteSensorLoop(Thunk_Sensor, this, m_event_client);

    m_pEncoder.reset();
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
