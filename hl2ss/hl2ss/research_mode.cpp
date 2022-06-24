
#include "./researchmode/ResearchModeApi.h"

extern "C" { HMODULE LoadLibraryA(LPCSTR lpLibFileName); }

typedef HRESULT(__cdecl* PFN_CREATEPROVIDER)(IResearchModeSensorDevice** ppSensorDevice);

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

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void CamAccessCallback(ResearchModeSensorConsent consent)
{
	g_camera_consent_value = consent;
	SetEvent(g_camera_consent_event);
}

// OK
static void IMUAccessCallback(ResearchModeSensorConsent consent)
{
	g_imu_consent_value = consent;
	SetEvent(g_imu_consent_event);
}

// OK
bool ResearchModeWaitForCameraConsent()
{
	WaitForSingleObject(g_camera_consent_event, INFINITE);
	return g_camera_consent_value == ResearchModeSensorConsent::Allowed;
}

// OK
bool ResearchModeWaitForIMUConsent()
{
	WaitForSingleObject(g_imu_consent_event, INFINITE);
	return g_imu_consent_value == ResearchModeSensorConsent::Allowed;
}

// OK
bool InitializeResearchMode()
{
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

	hr = g_pSensorDeviceConsent->RequestCamAccessAsync(CamAccessCallback);
	if (FAILED(hr)) { return false; }

	g_imu_consent_event = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (!g_imu_consent_event) { return false; }

	hr = g_pSensorDeviceConsent->RequestIMUAccessAsync(IMUAccessCallback);
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

	return true;
}

// OK
void CleanupResearchMode()
{
	for (auto sensor : g_sensors) { if (sensor) { sensor->Release(); } }
	g_sensors.clear();

	g_camera_consent_value = ResearchModeSensorConsent::UserPromptRequired;
	g_imu_consent_value = ResearchModeSensorConsent::UserPromptRequired;

	if (g_camera_consent_event) { CloseHandle(g_camera_consent_event); }
	if (g_imu_consent_event) { CloseHandle(g_imu_consent_event); }

	if (g_pSensorDeviceConsent) { g_pSensorDeviceConsent->Release(); }
	if (g_pSensorDevice) { g_pSensorDevice->Release(); }

	if (g_hrResearchMode) { FreeLibrary(g_hrResearchMode); }

	g_camera_consent_event = NULL;
	g_imu_consent_event = NULL;
	g_pSensorDeviceConsent = NULL;
	g_pSensorDevice = NULL;
	g_hrResearchMode = NULL;
}

// OK
void GetRigNodeId(GUID& outGuid)
{
	IResearchModeSensorDevicePerception* pSensorDevicePerception;
	g_pSensorDevice->QueryInterface(IID_PPV_ARGS(&pSensorDevicePerception));
	pSensorDevicePerception->GetRigNodeId(&outGuid);
	pSensorDevicePerception->Release();
}

// OK
IResearchModeSensor* GetResearchModeSensor(ResearchModeSensorType type)
{
	return g_sensors[type];
}

// OK
ResearchModeSensorType const* GetResearchModeSensorTypes()
{
	return g_types;
}

// OK
int GetResearchModeSensorTypeCount()
{
	return sizeof(g_types) / sizeof(ResearchModeSensorType);
}
