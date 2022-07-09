
#include "Cannon/DrawCall.h"

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

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void HolographicSpace_Initialize()
{
    DrawCall::Initialize();

    g_space = HolographicSpace::CreateForCoreWindow(CoreWindow::GetForCurrentThread());

    Microsoft::WRL::ComPtr<ID3D11Device> device = DrawCall::GetD3DDevice();

    Microsoft::WRL::ComPtr<IDXGIDevice3> dxgiDevice;
    device.As(&dxgiDevice);

    winrt::com_ptr<IInspectable> object;
    CreateDirect3D11DeviceFromDXGIDevice(dxgiDevice.Get(), (IInspectable**)(winrt::put_abi(object)));
    IDirect3DDevice interopDevice = object.as<IDirect3DDevice>();

    g_space.SetDirect3D11Device(interopDevice);
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
    auto viewport = pose.Viewport();
    DrawCall::SetBackBuffer(d3dBackBuffer.Get(), CD3D11_VIEWPORT(viewport.X, viewport.Y, viewport.Width, viewport.Height));
    DrawCall::GetBackBuffer()->Clear(0, 0, 0, 0);
    }
}

// OK
void HolographicSpace_Present()
{
    g_frame.PresentUsingCurrentPrediction(HolographicFramePresentWaitBehavior::WaitForFrameToFinish);
}
