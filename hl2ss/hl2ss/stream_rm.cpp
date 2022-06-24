
// TODO: AHAT

#include <mfapi.h>
#include <MemoryBuffer.h>
#include "research_mode.h"
#include "ports.h"
#include "utilities.h"
#include "types.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Graphics.Imaging.h>
#include <winrt/Windows.Storage.h>
#include <winrt/Windows.Storage.Streams.h>

using namespace Windows::Foundation;
using namespace winrt::Windows::Graphics::Imaging;
using namespace winrt::Windows::Storage;
using namespace winrt::Windows::Storage::Streams;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static char const* const g_research_sensor_port[] =
{
    PORT_RMVLF,
    PORT_RMVLL,
    PORT_RMVRF,
    PORT_RMVRR,
    PORT_RMZHT,
    PORT_RMZLT,
    PORT_RMACC,
    PORT_RMGYR,
    PORT_RMMAG
};

static HANDLE g_quitevent = NULL; // CloseHandle
static std::vector<HANDLE> g_threads; // CloseHandle

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void RM_VLC_Stream(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    uint32_t const width = 640;
    uint32_t const height = 480;
    uint32_t const framerate = 30;
    uint32_t const lumasize = width * height;
    uint32_t const chromasize = (width * height) / 2;
    uint32_t const framebytes = lumasize + chromasize;
    LONGLONG const duration = HNS_BASE / framerate;
    uint8_t const zerochroma = 0x80;

    IResearchModeSensorFrame* pSensorFrame; // release
    IResearchModeSensorVLCFrame* pVLCFrame; // release
    IMFSinkWriter* pSinkWriter; // release
    IMFMediaBuffer* pBuffer; // release
    IMFSample* pSample; // release
    HANDLE clientevent; // CloseHandle
    HookCallbackSocketData user;
    ResearchModeSensorTimestamp timestamp;
    DWORD dwVideoIndex;
    BYTE const* pImage;
    size_t length;
    BYTE* pDst;
    H26xFormat format;
    bool ok;
    
    ok = ReceiveVideoFormatH26x(clientsocket, format);
    if (!ok) { return; }

    format.width     = width;
    format.height    = height;
    format.framerate = framerate;

    clientevent = CreateEvent(NULL, TRUE, FALSE, NULL);

    user.clientsocket = clientsocket;
    user.clientevent = clientevent;

    CreateSinkWriterNV12ToH26x(&pSinkWriter, &dwVideoIndex, format, SendSampleToSocket, &user);

    MFCreateMemoryBuffer(framebytes, &pBuffer);
    
    pBuffer->Lock(&pDst, NULL, NULL);
    memset(pDst + lumasize, zerochroma, chromasize);
    pBuffer->Unlock();
    pBuffer->SetCurrentLength(framebytes);

    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(duration);

    sensor->OpenStream();

    do
    {
    sensor->GetNextBuffer(&pSensorFrame); // block

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pVLCFrame));

    pVLCFrame->GetBuffer(&pImage, &length);

    pBuffer->Lock(&pDst, NULL, NULL);
    memcpy(pDst, pImage, lumasize);
    pBuffer->Unlock();

    pSample->SetSampleTime(timestamp.HostTicks);
    pSinkWriter->WriteSample(dwVideoIndex, pSample);

    pVLCFrame->Release();
    pSensorFrame->Release();
    }
    while (WaitForSingleObject(clientevent, 0) == WAIT_TIMEOUT);

    sensor->CloseStream();

    pSinkWriter->Flush(dwVideoIndex);

    pSinkWriter->Release();
    pSample->Release();
    pBuffer->Release();

    CloseHandle(clientevent);
}

// OK
static void RM_ZHT_Stream(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    (void)sensor;
    (void)clientsocket;

    // Not supported yet
}

// OK
static void RM_ZLT_Stream(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    int const width = 320;
    int const height = 288;
    int const pixels = width * height;
    int const n32ByteVectors = (width * height * 4) / 32;
    uint8_t const depthinvalid = 0x80;
   
    IResearchModeSensorFrame* pSensorFrame; // release
    IResearchModeSensorDepthFrame* pDepthFrame; // release
    ResearchModeSensorTimestamp timestamp;
    BitmapPropertySet pngProperties;
    BYTE const* pSigma;
    size_t nSigmaCount;
    UINT16* pDepth;
    size_t nDepthCount;
    UINT16 const* pAbImage;
    size_t nAbCount;
    BYTE* pixelBufferData;
    UINT32 pixelBufferDataLength;
    WSABUF wsaBuf[3];
    bool ok;

    pngProperties.Insert(L"FilterOption", BitmapTypedValue(winrt::box_value(PngFilterMode::Paeth), winrt::Windows::Foundation::PropertyType::UInt8));
    
    sensor->OpenStream();

    do
    {
    sensor->GetNextBuffer(&pSensorFrame); // block

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pDepthFrame));

    pDepthFrame->GetSigmaBuffer(&pSigma, &nSigmaCount);
    pDepthFrame->GetBuffer(const_cast<const UINT16**>(&pDepth), &nDepthCount);
    pDepthFrame->GetAbDepthBuffer(&pAbImage, &nAbCount);

    auto stream = InMemoryRandomAccessStream();
    auto opEncoderCreate = BitmapEncoder::CreateAsync(BitmapEncoder::PngEncoderId(), stream, pngProperties);

    for (int i = 0; i < pixels; ++i) { if (pSigma[i] & depthinvalid) { pDepth[i] = 0; } }

    auto softwareBitmap = SoftwareBitmap(BitmapPixelFormat::Bgra8, width, height, BitmapAlphaMode::Straight);
    {
    auto bitmapBuffer = softwareBitmap.LockBuffer(BitmapBufferAccessMode::Write);
    auto spMemoryBufferByteAccess = bitmapBuffer.CreateReference().as<IMemoryBufferByteAccess>();
    spMemoryBufferByteAccess->GetBuffer(&pixelBufferData, &pixelBufferDataLength);
    PackUINT16toUINT32((BYTE*)pDepth, (BYTE*)pAbImage, pixelBufferData, n32ByteVectors);
    }

    auto encoder = opEncoderCreate.get();
    encoder.SetSoftwareBitmap(softwareBitmap);
    encoder.FlushAsync().get();

    auto streamSize = (uint32_t)stream.Size();
    auto streamBuf = Buffer(streamSize);
    stream.ReadAsync(streamBuf, streamSize, InputStreamOptions::None).get();

    wsaBuf[0].buf = (char*)&timestamp.HostTicks; wsaBuf[0].len = sizeof(timestamp.HostTicks);
    wsaBuf[1].buf = (char*)&streamSize;          wsaBuf[1].len = sizeof(streamSize);
    wsaBuf[2].buf = (char*)streamBuf.data();     wsaBuf[2].len = streamSize;

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));

    pDepthFrame->Release();
    pSensorFrame->Release();
    }
    while (ok);

    sensor->CloseStream();
}

// OK
static void RM_ACC_Stream(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    int const chunksize = 20;

    IResearchModeSensorFrame* pSensorFrame; // release
    IResearchModeAccelFrame* pSensorAccelFrame; // release
    ResearchModeSensorTimestamp timestamp;
    AccelDataStruct const* pAccelBuffer;
    size_t nAccelSamples;
    std::vector<BYTE> sampleBuffer;
    int bufSize;
    BYTE* pDst;
    WSABUF wsaBuf[3];
    bool ok;

    sensor->OpenStream();
    
    do
    {
    sensor->GetNextBuffer(&pSensorFrame); // block

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pSensorAccelFrame));

    pSensorAccelFrame->GetCalibratedAccelarationSamples(&pAccelBuffer, &nAccelSamples);

    bufSize = (int)(nAccelSamples * chunksize);
    if (sampleBuffer.size() < bufSize) { sampleBuffer.resize(bufSize); }
    pDst = sampleBuffer.data();

    for (int i = 0; i < (int)nAccelSamples; ++i) { memcpy(pDst + (i * chunksize), &(pAccelBuffer[i].SocTicks), chunksize); }

    wsaBuf[0].buf = (char*)&timestamp.HostTicks; wsaBuf[0].len = sizeof(timestamp.HostTicks);
    wsaBuf[1].buf = (char*)&bufSize;             wsaBuf[1].len = sizeof(bufSize);
    wsaBuf[2].buf = (char*)pDst;                 wsaBuf[2].len = bufSize;

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));

    pSensorAccelFrame->Release();
    pSensorFrame->Release();
    }
    while (ok);

    sensor->CloseStream();
}

// OK
static void RM_GYR_Stream(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    int const chunksize = 20;

    IResearchModeSensorFrame* pSensorFrame; // release
    IResearchModeGyroFrame* pSensorGyroFrame; // release
    ResearchModeSensorTimestamp timestamp;
    GyroDataStruct const* pGyroBuffer;
    size_t nGyroSamples;
    std::vector<BYTE> sampleBuffer;
    int bufSize;
    BYTE* pDst;
    WSABUF wsaBuf[3];
    bool ok;

    sensor->OpenStream();

    do
    {
    sensor->GetNextBuffer(&pSensorFrame); // block

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pSensorGyroFrame));

    pSensorGyroFrame->GetCalibratedGyroSamples(&pGyroBuffer, &nGyroSamples);

    bufSize = (int)(nGyroSamples * chunksize);
    if (sampleBuffer.size() < bufSize) { sampleBuffer.resize(bufSize); }
    pDst = sampleBuffer.data();

    for (int i = 0; i < (int)nGyroSamples; ++i) { memcpy(pDst + (i * chunksize), &(pGyroBuffer[i].SocTicks), chunksize); }

    wsaBuf[0].buf = (char*)&timestamp.HostTicks; wsaBuf[0].len = sizeof(timestamp.HostTicks);
    wsaBuf[1].buf = (char*)&bufSize;             wsaBuf[1].len = sizeof(bufSize);
    wsaBuf[2].buf = (char*)pDst;                 wsaBuf[2].len = bufSize;

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));

    pSensorGyroFrame->Release();
    pSensorFrame->Release();
    }
    while (ok);

    sensor->CloseStream();
}

// OK
static void RM_MAG_Stream(IResearchModeSensor* sensor, SOCKET clientsocket)
{
    int const chunksize = 20;

    IResearchModeSensorFrame* pSensorFrame; // release
    IResearchModeMagFrame* pSensorMagFrame; // release
    ResearchModeSensorTimestamp timestamp;
    MagDataStruct const* pMagBuffer;
    size_t nMagSamples;
    std::vector<BYTE> sampleBuffer;
    int bufSize;
    BYTE* pDst;
    WSABUF wsaBuf[3];
    bool ok;

    sensor->OpenStream();

    do
    {
    sensor->GetNextBuffer(&pSensorFrame); // block

    pSensorFrame->GetTimeStamp(&timestamp);
    pSensorFrame->QueryInterface(IID_PPV_ARGS(&pSensorMagFrame));
    pSensorMagFrame->GetMagnetometerSamples(&pMagBuffer, &nMagSamples);

    bufSize = (int)(nMagSamples * chunksize);
    if (sampleBuffer.size() < bufSize) { sampleBuffer.resize(bufSize); }
    pDst = sampleBuffer.data();

    for (int i = 0; i < (int)nMagSamples; ++i) { memcpy(pDst + (i * chunksize), &(pMagBuffer[i].SocTicks), chunksize); }

    wsaBuf[0].buf = (char*)&timestamp.HostTicks; wsaBuf[0].len = sizeof(timestamp.HostTicks);
    wsaBuf[1].buf = (char*)&bufSize;             wsaBuf[1].len = sizeof(bufSize);
    wsaBuf[2].buf = (char*)pDst;                 wsaBuf[2].len = bufSize;

    ok = send_multiple(clientsocket, wsaBuf, sizeof(wsaBuf) / sizeof(WSABUF));

    pSensorMagFrame->Release();
    pSensorFrame->Release();
    }
    while (ok);

    sensor->CloseStream();
}

// OK
static DWORD WINAPI RM_EntryPoint(void* param)
{
    IResearchModeSensor* sensor;
    ResearchModeSensorType type;
    char const* port;
    SOCKET listensocket; // closesocket
    SOCKET clientsocket; // closesocket
    bool ok;

    sensor = (IResearchModeSensor*)param;
    type = sensor->GetSensorType();
    ShowMessage(L"RM%d (%s): Waiting for consent", type, sensor->GetFriendlyName());

    switch (type)
    {
    case LEFT_FRONT:
    case LEFT_LEFT:
    case RIGHT_FRONT:
    case RIGHT_RIGHT:      
    case DEPTH_AHAT:       
    case DEPTH_LONG_THROW: ok = ResearchModeWaitForCameraConsent();  break;
    case IMU_ACCEL:        
    case IMU_GYRO:         
    case IMU_MAG:          ok = ResearchModeWaitForIMUConsent();     break;
    default:               ok = false;
    }

    if (!ok) { return 0; }

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
    case LEFT_FRONT:
    case LEFT_LEFT:
    case RIGHT_FRONT:
    case RIGHT_RIGHT:      RM_VLC_Stream(sensor, clientsocket); break;
    case DEPTH_AHAT:       RM_ZHT_Stream(sensor, clientsocket); break;
    case DEPTH_LONG_THROW: RM_ZLT_Stream(sensor, clientsocket); break;
    case IMU_ACCEL:        RM_ACC_Stream(sensor, clientsocket); break;
    case IMU_GYRO:         RM_GYR_Stream(sensor, clientsocket); break;
    case IMU_MAG:          RM_MAG_Stream(sensor, clientsocket); break;
    }

    closesocket(clientsocket);
    ShowMessage("RM%d: Client disconnected", type);
    }
    while (WaitForSingleObject(g_quitevent, 0) == WAIT_TIMEOUT);
  
    closesocket(listensocket);
    ShowMessage("RM%d: Closed", type);

    return 0;
}

// OK
bool RM_GetCameraExtrinsics(ResearchModeSensorType type, DirectX::XMFLOAT4X4& extrinsics)
{
    IResearchModeSensor* sensor;
    IResearchModeCameraSensor* pCameraSensor; // release
    IResearchModeAccelSensor* pAccelSensor; // release
    IResearchModeGyroSensor* pGyroSensor; // release

    sensor = GetResearchModeSensor(type);

    switch (type)
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
bool RM_GetCameraIntrinsics(ResearchModeSensorType type, std::vector<float> &uv2x, std::vector<float> &uv2y)
{
    IResearchModeCameraSensor* pCameraSensor; // release
    int width;
    int height;
    float uv[2];
    float xy[2];
    float* lutx;
    float* luty;
    int elements;

    switch (type)
    {
    case LEFT_FRONT:
    case LEFT_LEFT:
    case RIGHT_FRONT:
    case RIGHT_RIGHT:      width = 640; height = 480; break;
    case DEPTH_AHAT:       width = 512; height = 512; break;
    case DEPTH_LONG_THROW: width = 320; height = 288; break;
    default:               return false;
    }

    GetResearchModeSensor(type)->QueryInterface(IID_PPV_ARGS(&pCameraSensor));

    elements = width * height;

    uv2x.resize(elements);
    uv2y.resize(elements);

    lutx = uv2x.data();
    luty = uv2y.data();

    for (int y = 0; y < height; ++y)
    {
    uv[1] = (float)y;
    for (int x = 0; x < width;  ++x)
    {
    uv[0] = (float)x;

    pCameraSensor->MapImagePointToCameraUnitPlane(uv, xy);

    *(lutx++) = xy[0];
    *(luty++) = xy[1];
    }
    }

    pCameraSensor->Release();
    return true;
}

// OK
void RM_Initialize()
{
    g_quitevent = CreateEvent(NULL, TRUE, FALSE, NULL);
    int sensorcount = GetResearchModeSensorTypeCount();
    ResearchModeSensorType const* sensortypes = GetResearchModeSensorTypes();
    g_threads.resize(sensorcount);
    for (int i = 0; i < sensorcount; ++i) { g_threads[i] = CreateThread(NULL, 0, RM_EntryPoint, GetResearchModeSensor(sensortypes[i]), NULL, NULL); }
}

// OK
void RM_Quit()
{
    SetEvent(g_quitevent);
}

// OK
void RM_Cleanup()
{
    WaitForMultipleObjects((DWORD)g_threads.size(), g_threads.data(), TRUE, INFINITE);

    for (auto thread : g_threads) { CloseHandle(thread); }
    CloseHandle(g_quitevent);

    g_threads.clear();
    g_quitevent = NULL;
}
