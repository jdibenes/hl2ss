
#include "Cannon/DrawCall.h"
#include "utilities.h"

#include <windows.graphics.directx.direct3d11.interop.h>

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.UI.Core.h>
#include <winrt/Windows.Perception.h>
#include <winrt/Windows.Graphics.Holographic.h>
#include <winrt/Windows.Graphics.DirectX.Direct3D11.h>

using namespace winrt::Windows::UI::Core;
using namespace winrt::Windows::Perception;
using namespace winrt::Windows::Graphics::Holographic;
using namespace winrt::Windows::Graphics::DirectX::Direct3D11;
using namespace Windows::Graphics::DirectX::Direct3D11;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HolographicSpace g_space = nullptr;
static HolographicFrame g_frame = nullptr;
static HolographicFramePrediction g_prediction = nullptr;
static Microsoft::WRL::ComPtr<ID3D11Device> g_device = nullptr;
static ID3D11DeviceContext* g_context = NULL;
static ID3D11Texture2D* g_texture_marker = NULL;
static ID3D11Texture2D* g_texture_empty = NULL;
static bool g_enable_marker = false;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void HolographicSpace_EnableMarker(bool state)
{
    g_enable_marker = state;
}

// OK
static void HolographicSpace_CreateMarkerTexture()
{
    int const line_thickness = 16;
    UINT32 const line_color = 0xFFFFFFFF;

    int const buffer_width = 1440;
    int const buffer_height = 936;
    int const buffer_bpt = sizeof(UINT32);
    int const buffer_size = buffer_width * buffer_height * buffer_bpt;
    int const y0 = buffer_height - line_thickness;

    D3D11_TEXTURE2D_DESC dtd;
    D3D11_SUBRESOURCE_DATA dsd[2];
    BYTE* data; // delete[]

    dtd.Width              = buffer_width;
    dtd.Height             = buffer_height;
    dtd.MipLevels          = 1;
    dtd.ArraySize          = 2;
    dtd.Format             = DXGI_FORMAT_B8G8R8A8_UNORM;
    dtd.SampleDesc.Count   = 1;
    dtd.SampleDesc.Quality = 0;
    dtd.Usage              = D3D11_USAGE_DEFAULT;
    dtd.BindFlags          = 40;
    dtd.CPUAccessFlags     = 0;
    dtd.MiscFlags          = 2050;

    data = new BYTE[buffer_size];
    
    for (int i = 0; i < (sizeof(dsd) / sizeof(D3D11_SUBRESOURCE_DATA)); ++i)
    {
    dsd[i].pSysMem = data;
    dsd[i].SysMemPitch = buffer_width * buffer_bpt;
    dsd[i].SysMemSlicePitch = 0;
    }

    memset(data, 0, buffer_size);

    g_device->CreateTexture2D(&dtd, dsd, &g_texture_empty);

    for (int y = y0 - line_thickness; y < (y0 + line_thickness); ++y) { for (int x = 0; x < buffer_width; ++x) { ((UINT32*)data)[y * buffer_width + x] = line_color; } }

    g_device->CreateTexture2D(&dtd, dsd, &g_texture_marker);
   
    delete[] data;
}

// OK
void HolographicSpace_Initialize()
{
    DrawCall::Initialize();

    g_space = HolographicSpace::CreateForCoreWindow(CoreWindow::GetForCurrentThread());
    g_device = DrawCall::GetD3DDevice();

    Microsoft::WRL::ComPtr<IDXGIDevice3> dxgiDevice;
    g_device.As(&dxgiDevice);

    winrt::com_ptr<IInspectable> object;
    CreateDirect3D11DeviceFromDXGIDevice(dxgiDevice.Get(), (IInspectable**)(winrt::put_abi(object)));
    IDirect3DDevice interopDevice = object.as<IDirect3DDevice>();

    g_space.SetDirect3D11Device(interopDevice);
    g_device->GetImmediateContext(&g_context);

    HolographicSpace_CreateMarkerTexture();
}

// OK
void HolographicSpace_Update()
{
    g_frame = g_space.CreateNextFrame();
}

// OK
PerceptionTimestamp HolographicSpace_GetTimestamp()
{
    return g_frame.CurrentPrediction().Timestamp();
}

// OK
void HolographicSpace_Clear()
{
    Microsoft::WRL::ComPtr<ID3D11Texture2D> d3dBackBuffer;

    for (HolographicCameraPose const& pose : g_frame.CurrentPrediction().CameraPoses())
    {
    auto rp = g_frame.GetRenderingParameters(pose);
    rp.Direct3D11BackBuffer().as<IDirect3DDxgiInterfaceAccess>()->GetInterface(IID_PPV_ARGS(&d3dBackBuffer));
    g_context->CopyResource(d3dBackBuffer.Get(), g_enable_marker ? g_texture_marker : g_texture_empty);
    }
}

// OK
void HolographicSpace_Present()
{
    g_frame.PresentUsingCurrentPrediction(HolographicFramePresentWaitBehavior::WaitForFrameToFinish);
}
