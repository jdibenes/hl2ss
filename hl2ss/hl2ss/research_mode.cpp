
#include "extended_execution.h"
#include "locator.h"
#include "timestamp.h"
#include "research_mode.h"
#include "lock.h"

#include <winrt/Windows.Foundation.Numerics.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.Spatial.Preview.h>

using namespace winrt::Windows::Foundation::Numerics;
using namespace winrt::Windows::Perception::Spatial;
using namespace winrt::Windows::Perception::Spatial::Preview;

extern "C" { HMODULE LoadLibraryA(LPCSTR lpLibFileName); }

typedef HRESULT(__cdecl* PFN_CREATEPROVIDER)(IResearchModeSensorDevice**);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static ResearchModeSensorType const g_sensor_lut[] =
{
	LEFT_FRONT,
	LEFT_LEFT,
	RIGHT_FRONT,
	RIGHT_RIGHT,
	DEPTH_AHAT,
	DEPTH_LONG_THROW,
	IMU_ACCEL,
	IMU_GYRO,
	IMU_MAG
};

static uint32_t const g_sensor_count = sizeof(g_sensor_lut) / sizeof(ResearchModeSensorType);

static HMODULE g_hrResearchMode = NULL; // FreeLibrary
static IResearchModeSensorDevice* g_pSensorDevice = NULL; // Release
static IResearchModeSensorDeviceConsent* g_pSensorDeviceConsent = NULL; // Release
static HANDLE g_camera_consent_event = NULL; // CloseHandle
static HANDLE g_imu_consent_event = NULL; // CloseHandle
static ResearchModeSensorConsent g_camera_consent_value = ResearchModeSensorConsent::UserPromptRequired;
static ResearchModeSensorConsent g_imu_consent_value = ResearchModeSensorConsent::UserPromptRequired;
static IResearchModeSensor* g_sensors[g_sensor_count]; // Release
static SpatialLocator g_locator = nullptr;
static bool g_ready = false;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void ResearchMode_CameraAccessCallback(ResearchModeSensorConsent consent)
{
	g_camera_consent_value = consent;
	SetEvent(g_camera_consent_event);
}

// OK
static void ResearchMode_IMUAccessCallback(ResearchModeSensorConsent consent)
{
	g_imu_consent_value = consent;
	SetEvent(g_imu_consent_event);
}

// OK
static bool ResearchMode_WaitForCameraConsent()
{
	WaitForSingleObject(g_camera_consent_event, INFINITE);
	Cleaner log_error_camera([=]() { ExtendedExecution_EnterException(Exception::Exception_AccessDeniedCamera); });
	if (g_camera_consent_value != ResearchModeSensorConsent::Allowed) { return false; }
	log_error_camera.Set(false);
	return true;
}

// OK
static bool ResearchMode_WaitForIMUConsent()
{
	WaitForSingleObject(g_imu_consent_event, INFINITE);
	Cleaner log_error_movements([=]() { ExtendedExecution_EnterException(Exception::Exception_AccessDeniedMovements); });
	if (g_imu_consent_value != ResearchModeSensorConsent::Allowed) { return false; }
	log_error_movements.Set(false);
	return true;
}

// OK
void ResearchMode_Startup()
{
	IResearchModeSensorDevicePerception* pSensorDevicePerception; // Release
	GUID rigNodeId;
	HRESULT hr;

	Cleaner log_error_rm([=]() { ExtendedExecution_EnterException(Exception::Exception_DisabledResearchMode); });
	
	g_hrResearchMode = LoadLibraryA("ResearchModeAPI");

	PFN_CREATEPROVIDER pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>(GetProcAddress(g_hrResearchMode, "CreateResearchModeSensorDevice"));
	hr = pfnCreate(&g_pSensorDevice);
	if (FAILED(hr)) { goto _fail_not_enabled; }

	g_pSensorDevice->QueryInterface(IID_PPV_ARGS(&g_pSensorDeviceConsent));

	g_camera_consent_event = CreateEvent(NULL, TRUE, FALSE, NULL);
	g_imu_consent_event    = CreateEvent(NULL, TRUE, FALSE, NULL);

	g_pSensorDeviceConsent->RequestCamAccessAsync(ResearchMode_CameraAccessCallback);
	g_pSensorDeviceConsent->RequestIMUAccessAsync(ResearchMode_IMUAccessCallback);

	for (uint32_t sensor_index = 0; sensor_index < g_sensor_count; ++sensor_index) { g_pSensorDevice->GetSensor(g_sensor_lut[sensor_index], &(g_sensors[sensor_index])); }

	g_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception));

	pSensorDevicePerception->GetRigNodeId(&rigNodeId);
	pSensorDevicePerception->Release();

	g_locator = SpatialGraphInteropPreview::CreateLocatorForNode(rigNodeId);
	g_ready   = true;

	log_error_rm.Set(false);

	return;

_fail_not_enabled:
	FreeLibrary(g_hrResearchMode);
	g_hrResearchMode = NULL;
}

// OK
void ResearchMode_Cleanup()
{
	g_ready = false;
	g_locator = nullptr;
	for (uint32_t sensor_index = 0; sensor_index < g_sensor_count; ++sensor_index) { g_sensors[sensor_index]->Release(); }
	for (uint32_t sensor_index = 0; sensor_index < g_sensor_count; ++sensor_index) { g_sensors[sensor_index] = NULL; }
	g_pSensorDeviceConsent->Release();
	g_pSensorDeviceConsent = NULL;
	g_pSensorDevice->Release();
	g_pSensorDevice = NULL;
	FreeLibrary(g_hrResearchMode);
	g_hrResearchMode = NULL;
	CloseHandle(g_camera_consent_event);
	CloseHandle(g_imu_consent_event);
	g_camera_consent_event = NULL;
	g_imu_consent_event = NULL;
	g_camera_consent_value = ResearchModeSensorConsent::UserPromptRequired;
	g_imu_consent_value = ResearchModeSensorConsent::UserPromptRequired;
}

// OK
bool ResearchMode_Status()
{
	return g_ready;
}

// OK
IResearchModeSensor* ResearchMode_GetSensor(ResearchModeSensorType type)
{
	return g_sensors[type];
}

// OK
float4x4 ResearchMode_GetRigNodeWorldPose(UINT64 host_ticks)
{
	return Locator_Locate(Timestamp_QPCToPerception(host_ticks), g_locator, Locator_GetWorldCoordinateSystem());
}

// OK
bool ResearchMode_WaitForConsent(IResearchModeSensor* sensor)
{
	switch (sensor->GetSensorType())
	{
	case LEFT_FRONT:
	case LEFT_LEFT:
	case RIGHT_FRONT:
	case RIGHT_RIGHT:
	case DEPTH_AHAT:
	case DEPTH_LONG_THROW: return ResearchMode_WaitForCameraConsent();
	case IMU_ACCEL:
	case IMU_GYRO:
	case IMU_MAG:          return ResearchMode_WaitForIMUConsent();
	}

	return false;
}

// OK
bool ResearchMode_GetIntrinsics(IResearchModeSensor* sensor, std::vector<float>& uv2x, std::vector<float>& uv2y, std::vector<float>& mapx, std::vector<float>& mapy, float K[4])
{
    IResearchModeCameraSensor* pCameraSensor; // Release
    int width;
    int height;
	int elements;
	float* lutx;
	float* luty;
    float uv[2];
    float xy[2];
	float x;
	float y;    
	float min_x;
	float max_x;
	float min_y;
	float max_y;
	float span_x;
	float span_y;
	float span_u;
	float span_v;
	float fx;
	float fy;
	float cx;
	float cy;

    switch (sensor->GetSensorType())
    {
    case LEFT_FRONT:
    case LEFT_LEFT:
    case RIGHT_FRONT:
    case RIGHT_RIGHT:      width = RM_VLC_WIDTH; height = RM_VLC_HEIGHT; break;
    case DEPTH_AHAT:       width = RM_ZHT_WIDTH; height = RM_ZHT_HEIGHT; break;
    case DEPTH_LONG_THROW: width = RM_ZLT_WIDTH; height = RM_ZLT_HEIGHT; break;
    default:               return false;
    }

    sensor->QueryInterface(IID_PPV_ARGS(&pCameraSensor));

    elements = width * height;

    uv2x.resize(elements);
    uv2y.resize(elements);
	mapx.resize(elements);
	mapy.resize(elements);

	min_x =  std::numeric_limits<float>::infinity();
	max_x = -std::numeric_limits<float>::infinity();
	min_y =  std::numeric_limits<float>::infinity();
	max_y = -std::numeric_limits<float>::infinity();

	lutx = uv2x.data();
	luty = uv2y.data();

    for (int v = 0; v < height; ++v)
    {
    for (int u = 0; u < width;  ++u)
    {
    uv[0] = static_cast<float>(u) + 0.5f;
	uv[1] = static_cast<float>(v) + 0.5f;

    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

	x = xy[0];
	y = xy[1];

    *(lutx++) = x;
    *(luty++) = y;

	if (x < min_x) { min_x = x; } else if (x > max_x) { max_x = x; }
	if (y < min_y) { min_y = y; } else if (y > max_y) { max_y = y; }
    }
    }

	span_x = max_x - min_x;
	span_y = max_y - min_y;
	span_u = static_cast<float>(width - 1);
	span_v = static_cast<float>(height - 1);

	fx = span_x / span_u;
	fy = span_y / span_v;
	cx = min_x;
	cy = min_y;

	lutx = mapx.data();
	luty = mapy.data();

	for (int v = 0; v < height; ++v)
	{	
	for (int u = 0; u < width; ++u)
	{
	xy[0] = fx * u + cx;
	xy[1] = fy * v + cy;

	pCameraSensor->MapCameraSpaceToImagePoint(xy, uv);

	*(lutx++) = uv[0];
	*(luty++) = uv[1];
	}
	}

	K[0] = span_u / span_x;
	K[1] = span_v / span_y;
	K[2] = -min_x * K[0];
	K[3] = -min_y * K[1];

    pCameraSensor->Release();
    return true;
}

// OK
bool ResearchMode_GetExtrinsics(IResearchModeSensor* sensor, DirectX::XMFLOAT4X4& extrinsics)
{
	IResearchModeCameraSensor* pCameraSensor; // Release
	IResearchModeAccelSensor* pAccelSensor; // Release
	IResearchModeGyroSensor* pGyroSensor; // Release

	switch (sensor->GetSensorType())
	{
	case LEFT_FRONT:
	case LEFT_LEFT:
	case RIGHT_FRONT:
	case RIGHT_RIGHT:
	case DEPTH_AHAT:
	case DEPTH_LONG_THROW: sensor->QueryInterface(IID_PPV_ARGS(&pCameraSensor)); pCameraSensor->GetCameraExtrinsicsMatrix(&extrinsics); pCameraSensor->Release(); break;
	case IMU_ACCEL:        sensor->QueryInterface(IID_PPV_ARGS(&pAccelSensor));  pAccelSensor->GetExtrinsicsMatrix(&extrinsics);        pAccelSensor->Release();  break;
	case IMU_GYRO:         sensor->QueryInterface(IID_PPV_ARGS(&pGyroSensor));   pGyroSensor->GetExtrinsicsMatrix(&extrinsics);         pGyroSensor->Release();   break;
	default:               return false;
	}

	return true;
}

// OK
void ResearchMode_ExecuteSensorLoop(IResearchModeSensor* sensor, HOOK_RM_PROC hook, void* param, HANDLE event_stop)
{
	IResearchModeSensorFrame* pSensorFrame; // Release

	sensor->OpenStream();

	do
	{
	sensor->GetNextBuffer(&pSensorFrame); // block
	hook(pSensorFrame, param);
	pSensorFrame->Release();
	}
	while (WaitForSingleObject(event_stop, 0) == WAIT_TIMEOUT);

	sensor->CloseStream();
}

// OK
void ResearchMode_ProcessSample_VLC(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_VLC_PROC hook, void* param)
{
	IResearchModeSensorVLCFrame* pVLCFrame; // Release
	ResearchModeSensorTimestamp timestamp;
	BYTE const* pImage;
	size_t length;
	UINT64 exposure;
	UINT32 gain;

	pSensorFrame->GetTimeStamp(&timestamp);
	pSensorFrame->QueryInterface(IID_PPV_ARGS(&pVLCFrame));

	pVLCFrame->GetBuffer(&pImage, &length);
	pVLCFrame->GetExposure(&exposure);
	pVLCFrame->GetGain(&gain);

	hook(pImage, timestamp.HostTicks, timestamp.SensorTicks, exposure, gain, param);

	pVLCFrame->Release();
}

// OK
void ResearchMode_ProcessSample_ZHT(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_ZHT_PROC hook, void* param)
{
	IResearchModeSensorDepthFrame* pDepthFrame; // Release
	ResearchModeSensorTimestamp timestamp;
	UINT16 const* pDepth;
	UINT16 const* pAbImage;
	size_t nDepthCount;
	size_t nAbCount;

	pSensorFrame->GetTimeStamp(&timestamp);
	pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));

	pDepthFrame->GetBuffer(&pDepth, &nDepthCount);
	pDepthFrame->GetAbDepthBuffer(&pAbImage, &nAbCount);

	hook(pDepth, pAbImage, timestamp.HostTicks, timestamp.SensorTicks, param);

	pDepthFrame->Release();
}

// OK
void ResearchMode_ProcessSample_ZLT(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_ZLT_PROC hook, void* param)
{
	IResearchModeSensorDepthFrame* pDepthFrame; // Release
	ResearchModeSensorTimestamp timestamp;
	BYTE const* pSigma;
	UINT16 const* pDepth;
	UINT16 const* pAbImage;
	size_t nSigmaCount;
	size_t nDepthCount;
	size_t nAbCount;

	pSensorFrame->GetTimeStamp(&timestamp);
	pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));

	pDepthFrame->GetSigmaBuffer(&pSigma, &nSigmaCount);
	pDepthFrame->GetBuffer(&pDepth, &nDepthCount);
	pDepthFrame->GetAbDepthBuffer(&pAbImage, &nAbCount);

	hook(pSigma, pDepth, pAbImage, timestamp.HostTicks, timestamp.SensorTicks, param);

	pDepthFrame->Release();
}

// OK
void ResearchMode_ProcessSample_ACC(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_ACC_PROC hook, void* param)
{
	IResearchModeAccelFrame* pSensorIMUFrame; // Release
	ResearchModeSensorTimestamp timestamp;
	AccelDataStruct const* pIMUBuffer;
	size_t nIMUSamples;

	pSensorFrame->GetTimeStamp(&timestamp);
	pSensorFrame->QueryInterface(IID_PPV_ARGS(&pSensorIMUFrame));

	pSensorIMUFrame->GetCalibratedAccelarationSamples(&pIMUBuffer, &nIMUSamples);

	hook(pIMUBuffer, nIMUSamples, timestamp.HostTicks, timestamp.SensorTicks, param);

	pSensorIMUFrame->Release();
}

// OK
void ResearchMode_ProcessSample_GYR(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_GYR_PROC hook, void* param)
{
	IResearchModeGyroFrame* pSensorIMUFrame; // Release
	ResearchModeSensorTimestamp timestamp;
	GyroDataStruct const* pIMUBuffer;
	size_t nIMUSamples;

	pSensorFrame->GetTimeStamp(&timestamp);
	pSensorFrame->QueryInterface(IID_PPV_ARGS(&pSensorIMUFrame));

	pSensorIMUFrame->GetCalibratedGyroSamples(&pIMUBuffer, &nIMUSamples);

	hook(pIMUBuffer, nIMUSamples, timestamp.HostTicks, timestamp.SensorTicks, param);

	pSensorIMUFrame->Release();
}

// OK
void ResearchMode_ProcessSample_MAG(IResearchModeSensorFrame* pSensorFrame, HOOK_RM_MAG_PROC hook, void* param)
{
	IResearchModeMagFrame* pSensorIMUFrame; // Release
	ResearchModeSensorTimestamp timestamp;
	MagDataStruct const* pIMUBuffer;
	size_t nIMUSamples;

	pSensorFrame->GetTimeStamp(&timestamp);
	pSensorFrame->QueryInterface(IID_PPV_ARGS(&pSensorIMUFrame));

	pSensorIMUFrame->GetMagnetometerSamples(&pIMUBuffer, &nIMUSamples);

	hook(pIMUBuffer, nIMUSamples, timestamp.HostTicks, timestamp.SensorTicks, param);

	pSensorIMUFrame->Release();
}

// OK
void ResearchMode_SetEyeSelection(bool enable)
{
	if (!g_ready) { return; }
	if (enable) { g_pSensorDevice->EnableEyeSelection(); } else { g_pSensorDevice->DisableEyeSelection(); }
}
