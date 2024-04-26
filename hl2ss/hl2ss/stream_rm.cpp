
#include "research_mode.h"
#include "server.h"
#include "stream_rm_vlc.h"
#include "stream_rm_imu.h"
#include "stream_rm_zht.h"
#include "stream_rm_zlt.h"
#include "log.h"
#include "ports.h"
#include "types.h"

#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>

using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;

typedef void(*RM_MODE0)(IResearchModeSensor*, SOCKET);
typedef void(*RM_MODE1)(IResearchModeSensor*, SOCKET, SpatialLocator const&);
typedef void(*RM_MODE2)(IResearchModeSensor*, SOCKET);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static char const* const g_research_sensor_port[] =
{
    PORT_NAME_RM_VLF,
    PORT_NAME_RM_VLL,
    PORT_NAME_RM_VRF,
    PORT_NAME_RM_VRR,
    PORT_NAME_RM_ZHT,
    PORT_NAME_RM_ZLT,
    PORT_NAME_RM_ACC,
    PORT_NAME_RM_GYR,
    PORT_NAME_RM_MAG
};

static HANDLE g_event_quit = NULL; // CloseHandle
static std::vector<HANDLE> g_threads; // CloseHandle

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
template<RM_MODE0 M0, RM_MODE1 M1, RM_MODE2 M2>
void RM_Stream(IResearchModeSensor* sensor, SOCKET clientsocket, SpatialLocator const& locator)
{
    uint8_t mode;
    bool ok;

    ok = recv_u8(clientsocket, mode);
    if (!ok) { return; }

    switch (mode)
    {
    case 0: M0(sensor, clientsocket);          break;
    case 1: M1(sensor, clientsocket, locator); break;
    case 2: M2(sensor, clientsocket);          break;
    }
}

// OK
static DWORD WINAPI RM_EntryPoint(void* param)
{
    SpatialLocator locator = nullptr;
    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket
    IResearchModeSensor* sensor;
    ResearchModeSensorType type;
    char const* port;
    bool ok;

    sensor = (IResearchModeSensor*)param;
    type = sensor->GetSensorType();

    ShowMessage(L"RM%d (%s): Waiting for consent", type, sensor->GetFriendlyName());

    ok = ResearchMode_WaitForConsent(sensor);
    if (!ok) { return false; }

    locator = ResearchMode_GetLocator();
    port = g_research_sensor_port[type];
    listensocket = CreateSocket(port);

    ShowMessage("RM%d: Listening at port %s", type, port);
    
    do
    {
    ShowMessage("RM%d: Waiting for client", type);

    clientsocket = accept(listensocket, NULL, NULL); // block
    if (clientsocket == INVALID_SOCKET) { break; }

    ShowMessage("RM%d: Client connected", type);

    switch (type)
    {
    case ResearchModeSensorType::LEFT_FRONT:       RM_Stream<RM_VLC_Mode0, RM_VLC_Mode1, RM_VLC_Mode2>(sensor, clientsocket, locator); break;
    case ResearchModeSensorType::LEFT_LEFT:        RM_Stream<RM_VLC_Mode0, RM_VLC_Mode1, RM_VLC_Mode2>(sensor, clientsocket, locator); break;
    case ResearchModeSensorType::RIGHT_FRONT:      RM_Stream<RM_VLC_Mode0, RM_VLC_Mode1, RM_VLC_Mode2>(sensor, clientsocket, locator); break;
    case ResearchModeSensorType::RIGHT_RIGHT:      RM_Stream<RM_VLC_Mode0, RM_VLC_Mode1, RM_VLC_Mode2>(sensor, clientsocket, locator); break;
    case ResearchModeSensorType::DEPTH_AHAT:       RM_Stream<RM_ZHT_Mode0, RM_ZHT_Mode1, RM_ZHT_Mode2>(sensor, clientsocket, locator); break;
    case ResearchModeSensorType::DEPTH_LONG_THROW: RM_Stream<RM_ZLT_Mode0, RM_ZLT_Mode1, RM_ZLT_Mode2>(sensor, clientsocket, locator); break;
    case ResearchModeSensorType::IMU_ACCEL:        RM_Stream<RM_ACC_Mode0, RM_ACC_Mode1, RM_ACC_Mode2>(sensor, clientsocket, locator); break;
    case ResearchModeSensorType::IMU_GYRO:         RM_Stream<RM_GYR_Mode0, RM_GYR_Mode1, RM_GYR_Mode2>(sensor, clientsocket, locator); break;
    case ResearchModeSensorType::IMU_MAG:          RM_Stream<RM_MAG_Mode0, RM_MAG_Mode1, RM_MAG_Mode2>(sensor, clientsocket, locator); break;
    }

    closesocket(clientsocket);

    ShowMessage("RM%d: Client disconnected", type);
    }
    while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);
  
    closesocket(listensocket);

    ShowMessage("RM%d: Closed", type);

    return 0;
}

// OK
void RM_Initialize()
{
    ResearchModeSensorType const* sensortypes = ResearchMode_GetSensorTypes();
    int const sensorcount = ResearchMode_GetSensorTypeCount();

    g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
    
    g_threads.resize(sensorcount);
    for (int i = 0; i < sensorcount; ++i) { g_threads[i] = CreateThread(NULL, 0, RM_EntryPoint, ResearchMode_GetSensor(sensortypes[i]), NULL, NULL); }
}

// OK
void RM_Quit()
{
    SetEvent(g_event_quit);
}

// OK
void RM_Cleanup()
{
    WaitForMultipleObjects((DWORD)g_threads.size(), g_threads.data(), TRUE, INFINITE);

    for (auto thread : g_threads) { CloseHandle(thread); }
    CloseHandle(g_event_quit);

    g_threads.clear();
    g_event_quit = NULL;
}
