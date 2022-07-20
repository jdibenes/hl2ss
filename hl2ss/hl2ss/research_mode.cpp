
#include "research_mode.h"

extern "C" { HMODULE LoadLibraryA(LPCSTR lpLibFileName); }

typedef HRESULT(__cdecl* PFN_CREATEPROVIDER)(IResearchModeSensorDevice**);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static ResearchModeSensorType const g_types[] =
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

static HMODULE g_hrResearchMode = NULL; // FreeLibrary
static IResearchModeSensorDevice* g_pSensorDevice = NULL; // Release
static IResearchModeSensorDeviceConsent* g_pSensorDeviceConsent = NULL; // Release
static HANDLE g_camera_consent_event = NULL; // CloseHandle
static HANDLE g_imu_consent_event = NULL; // CloseHandle
static ResearchModeSensorConsent g_camera_consent_value = ResearchModeSensorConsent::UserPromptRequired;
static ResearchModeSensorConsent g_imu_consent_value = ResearchModeSensorConsent::UserPromptRequired;
static std::vector<IResearchModeSensor*> g_sensors; // Release
static GUID g_rigNodeId;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void ResearchMode_CamAccessCallback(ResearchModeSensorConsent consent)
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
bool ResearchMode_WaitForCameraConsent()
{
	WaitForSingleObject(g_camera_consent_event, INFINITE);
	return g_camera_consent_value == ResearchModeSensorConsent::Allowed;
}

// OK
bool ResearchMode_WaitForIMUConsent()
{
	WaitForSingleObject(g_imu_consent_event, INFINITE);
	return g_imu_consent_value == ResearchModeSensorConsent::Allowed;
}

// OK
bool ResearchMode_WaitForConsent(IResearchModeSensor *sensor)
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
bool ResearchMode_Initialize()
{
	IResearchModeSensorDevicePerception* pSensorDevicePerception; // Release
	HRESULT hr;
	std::vector<ResearchModeSensorDescriptor> sensordescriptors;
	PFN_CREATEPROVIDER pfnCreate;
	size_t sensorcount;
	size_t sensorsloaded;
	
	g_hrResearchMode = LoadLibraryA("ResearchModeAPI");
	if (!g_hrResearchMode) { return false; }

	pfnCreate = reinterpret_cast<PFN_CREATEPROVIDER>(GetProcAddress(g_hrResearchMode, "CreateResearchModeSensorDevice"));
	if (!pfnCreate) { return false; }

	hr = pfnCreate(&g_pSensorDevice);
	if (FAILED(hr)) { return false; }

	hr = g_pSensorDevice->QueryInterface(IID_PPV_ARGS(&g_pSensorDeviceConsent));
	if (FAILED(hr)) { return false; }

	g_camera_consent_event = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (!g_camera_consent_event) { return false; }

	hr = g_pSensorDeviceConsent->RequestCamAccessAsync(ResearchMode_CamAccessCallback);
	if (FAILED(hr)) { return false; }

	g_imu_consent_event = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (!g_imu_consent_event) { return false; }

	hr = g_pSensorDeviceConsent->RequestIMUAccessAsync(ResearchMode_IMUAccessCallback);
	if (FAILED(hr)) { return false; }
	
	hr = g_pSensorDevice->GetSensorCount(&sensorcount);
	if (FAILED(hr)) { return false; }

	sensordescriptors.resize(sensorcount);	
	hr = g_pSensorDevice->GetSensorDescriptors(sensordescriptors.data(), sensordescriptors.size(), &sensorsloaded);
	if (FAILED(hr)) { return false; }
	if (sensorcount != sensorsloaded) { return false; }

	g_sensors.clear();
	g_sensors.resize(sensorcount);

	memset(g_sensors.data(), 0, g_sensors.size() * sizeof(IResearchModeSensor*));

	for (auto const& sensorDescriptor : sensordescriptors)
	{
	hr = g_pSensorDevice->GetSensor(sensorDescriptor.sensorType, &g_sensors[sensorDescriptor.sensorType]);
	if (FAILED(hr)) { return false; }
	}

	hr = g_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception));
	if (FAILED(hr)) { return false; }

	hr = pSensorDevicePerception->GetRigNodeId(&g_rigNodeId);
	if (FAILED(hr)) { return false; }

	pSensorDevicePerception->Release();

	return true;
}

// OK
void ResearchMode_Cleanup()
{
	for (auto sensor : g_sensors) { if (sensor) { sensor->Release(); } }	

	if (g_camera_consent_event) { CloseHandle(g_camera_consent_event); }
	if (g_imu_consent_event) { CloseHandle(g_imu_consent_event); }

	if (g_pSensorDeviceConsent) { g_pSensorDeviceConsent->Release(); }
	if (g_pSensorDevice) { g_pSensorDevice->Release(); }

	if (g_hrResearchMode) { FreeLibrary(g_hrResearchMode); }

	g_hrResearchMode = NULL;
	g_pSensorDevice = NULL;
	g_pSensorDeviceConsent = NULL;
	g_camera_consent_event = NULL;
	g_imu_consent_event = NULL;
	g_camera_consent_value = ResearchModeSensorConsent::UserPromptRequired;
	g_imu_consent_value = ResearchModeSensorConsent::UserPromptRequired;	
	g_sensors.clear();
	memset(&g_rigNodeId, 0, sizeof(g_rigNodeId));
}

// OK
IResearchModeSensor* ResearchMode_GetSensor(ResearchModeSensorType type)
{
	return g_sensors[type];
}

// OK
ResearchModeSensorType const* ResearchMode_GetSensorTypes()
{
	return g_types;
}

// OK
int ResearchMode_GetSensorTypeCount()
{
	return sizeof(g_types) / sizeof(ResearchModeSensorType);
}

// OK
bool ResearchMode_GetIntrinsics(IResearchModeSensor* sensor, std::vector<float>& uv2x, std::vector<float>& uv2y)
{
    IResearchModeCameraSensor* pCameraSensor; // Release
    int width;
    int height;
    float uv[2];
    float xy[2];
    float* lutx;
    float* luty;
    int elements;

    switch (sensor->GetSensorType())
    {
    case LEFT_FRONT:
    case LEFT_LEFT:
    case RIGHT_FRONT:
    case RIGHT_RIGHT:      width = RM_VLC_WIDTH;  height = RM_VLC_HEIGHT;  break;
    case DEPTH_AHAT:       width = RM_AHAT_WIDTH; height = RM_AHAT_HEIGHT; break;
    case DEPTH_LONG_THROW: width = RM_LT_WIDTH;   height = RM_LT_HEIGHT;   break;
    default:               return false;
    }

    sensor->QueryInterface(IID_PPV_ARGS(&pCameraSensor));

    elements = width * height;

    uv2x.resize(elements);
    uv2y.resize(elements);

    lutx = uv2x.data();
    luty = uv2y.data();

    for (int y = 0; y < height; ++y)
    {
    uv[1] = (float)y + 0.5f;
    for (int x = 0; x < width;  ++x)
    {
    uv[0] = (float)x + 0.5f;

    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

    *(lutx++) = xy[0];
    *(luty++) = xy[1];
    }
    }

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
GUID ResearchMode_GetRigNodeId()
{
	return g_rigNodeId;
}
