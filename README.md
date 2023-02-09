# HoloLens 2 Sensor Streaming

HoloLens 2 server application for streaming sensor data via TCP. Created to stream HoloLens data to a Linux machine for research purposes. Also works on Windows and OS X.

**Supported streams**

- Research Mode Visible Light Cameras (4 cameras, 640x480 @ 30 FPS, Grayscale, H264 or HEVC encoded)
- Research Mode Depth
  - AHAT (512x512 @ 45 FPS, 16-bit Depth + 16-bit AB as NV12 luma+chroma, H264 or HEVC encoded) 
  - Long Throw (320x288 @ 5 FPS, 16-bit Depth + 16-bit AB encoded as a single 32-bit PNG)
- Research Mode IMU
  - Accelerometer
  - Gyroscope
  - Magnetometer
- Front Camera (1920x1080 @ 30 FPS, RGB, H264 or HEVC encoded)
- Microphone (2 channels, 48000 Hz, AAC encoded)
- Spatial Input (60 Hz)
  - Head Tracking
  - Eye Tracking
  - Hand Tracking
  
**Additional features**

- Download calibration (e.g., camera intrinsics) for the Front Camera and Research Mode sensors (except RM IMU Magnetometer).
- Optional per-frame pose for the Front Camera and Research Mode sensors streams.
- Client can configure the bitrate of the H264, HEVC, and AAC encoded streams.
- For the Front Camera, the client can configure the resolution, framerate, focus, white balance, and exposure (see [etc/hl2_capture_formats.txt](https://github.com/jdibenes/hl2ss/blob/main/etc/hl2_capture_formats.txt) for a list of supported formats).
- Access to Spatial Mapping data (Experimental).

## Preparation

Before using the server software, configure your HoloLens as follows:

1. Enable developer mode: Settings -> Update & Security -> For developers -> Use developer features.
2. Enable device portal: Settings -> Update & Security -> For developers -> Device Portal.
3. Enable research mode: Refer to the **Enabling Research Mode** section in [HoloLens Research Mode](https://docs.microsoft.com/en-us/windows/mixed-reality/develop/advanced-concepts/research-mode).

Please note that **enabling Research Mode on the HoloLens increases battery usage**.

## Installation (sideloading)

The server software is distributed as a single appxbundle file.

1. Download the [latest appxbundle](https://github.com/jdibenes/hl2ss/releases).
2. Go to the Device Portal (type the IP address of your HoloLens in the address bar of your preferred web browser) and upload the appxbundle to the HoloLens (System -> File explorer -> Downloads).
3. On your HoloLens, open the File Explorer and locate the appxbundle. Tap the appxbundle file to open the installer and tap Install.

You can find the server application (hl2ss) in the All apps list.

## Permissions

The first time the server runs it will ask for the necessary permissions to access sensor data. If there are any issues, please verify that the server application (hl2ss.exe) has access to:

- Camera (Settings -> Privacy -> Camera).
- Eye tracker (Settings -> Privacy -> Eye tracker).
- Microphone (Settings -> Privacy -> Microphone).
- User movements (Settings -> Privacy -> User movements).

## Python client

The Python scripts in the [viewer](https://github.com/jdibenes/hl2ss/tree/main/viewer) directory demonstrate how to connect to the server, receive the data, unpack it, and decode it in real time. Run the server application on your HoloLens and set the host variable of the Python scripts to your HoloLens IP address.
- RM VLC: [viewer/client_rm_vlc.py](https://github.com/jdibenes/hl2ss/blob/main/viewer/client_rm_vlc.py)
- RM Depth AHAT: [viewer/client_rm_depth_ahat.py](https://github.com/jdibenes/hl2ss/blob/main/viewer/client_rm_depth_ahat.py)
- RM Depth Long Throw: [viewer/client_rm_depth_longthrow.py](https://github.com/jdibenes/hl2ss/blob/main/viewer/client_rm_depth_longthrow.py)
- RM IMU: [viewer/client_rm_imu.py](https://github.com/jdibenes/hl2ss/blob/main/viewer/client_rm_imu.py)
- Front Camera: [viewer/client_pv.py](https://github.com/jdibenes/hl2ss/blob/main/viewer/client_pv.py)
- Microphone: [viewer/client_microphone.py](https://github.com/jdibenes/hl2ss/blob/main/viewer/client_microphone.py)
- Spatial Input: [viewer/client_si.py](https://github.com/jdibenes/hl2ss/blob/main/viewer/client_si.py)
- Remote Configuration: [viewer/client_rc.py](https://github.com/jdibenes/hl2ss/blob/main/viewer/client_rc.py)
- Spatial Mapping: [viewer/client_sm.py](https://github.com/jdibenes/hl2ss/blob/main/viewer/client_sm.py)

Required packages:

- [OpenCV](https://github.com/opencv/opencv-python) `pip install opencv-python`
- [PyAV](https://github.com/PyAV-Org/PyAV) `pip install av`
- [NumPy](https://numpy.org/) `pip install numpy`

Optional packages used by some of the samples:

- [pynput](https://github.com/moses-palmer/pynput) `pip install pynput`
- [Open3D](http://www.open3d.org/) `pip install open3d`
- [PyAudio](https://people.csail.mit.edu/hubert/pyaudio/) `pip install PyAudio`

## Known issues and limitations

- Multiple streams can be active at the same time but only one client per stream is allowed.
- Ocassionally, the application might crash when accessing the Front Camera and RM Depth Long Throw streams simultaneously. See https://github.com/microsoft/HoloLens2ForCV/issues/142.
- Currently, it is not possible to access the Front Camera and RM Depth AHAT streams simultaneously without downgrading the HoloLens 2 OS. See https://github.com/microsoft/HoloLens2ForCV/issues/133.
- The RM Depth AHAT and RM Depth Long Throw streams cannot be accessed simultaneously.
- The Python scripts might freeze under some configurations. See https://github.com/PyAV-Org/PyAV/issues/978 and https://github.com/opencv/opencv/issues/21952 for details and workarounds.
- Building for x86 and x64 (HoloLens emulator), and ARM is not supported.

## Build from source and deploy

Building requires a Windows 10 machine:

1. [Install the tools](https://docs.microsoft.com/en-us/windows/mixed-reality/develop/install-the-tools).
2. Open the Visual Studio solution (sln file in the [hl2ss](https://github.com/jdibenes/hl2ss/tree/main/hl2ss) folder) in Visual Studio 2022.
3. [Pair your HoloLens 2](https://learn.microsoft.com/en-us/windows/mixed-reality/develop/advanced-concepts/using-visual-studio?tabs=hl2#pairing-your-device).
4. Build Release ARM64.
    - If you get an error saying that hl2ss.winmd does not exist, copy the hl2ss.winmd file from [etc](https://github.com/jdibenes/hl2ss/tree/main/etc) into the hl2ss\ARM64\Release\hl2ss folder.
5. In the Solution Explorer, right click the hl2ss project and select Properties. Navigate to Configuration Properties -> Debugging and set Machine Name to your HoloLens IP address.
6. Run. The application will remain installed on the HoloLens even after power off.

This process also builds the Unity plugin.

## Unity plugin (DLL)

For streaming sensor data from a Unity application.
All streams are supported.
However, to enable Spatial Input stream support, the plugin must be initialized from the UI thread.
This process is described later in this section.

**Using the plugin**

1. Download the [plugin](https://github.com/jdibenes/hl2ss/releases) and extract the Assets folder into your Unity project.
    - If you wish to create a new Unity project to test the plugin, first follow the intructions [here](https://learn.microsoft.com/en-us/training/modules/learn-mrtk-tutorials/1-1-introduction), and then continue with the instructions presented in this section.
2. In the Unity Editor, configure the plugin as UWP ARM64.
    1. Find the plugin in the Project window, select it, then go to the Inspector window.
    2. Set SDK to UWP.
    3. Set CPU to ARM64.
    4. Click Apply.
3. Add the [hl2ss.cs](https://github.com/jdibenes/hl2ss/blob/main/unity/hl2ss.cs) script to the Main Camera and set the Material field to [BasicMaterial](https://github.com/jdibenes/hl2ss/blob/main/unity/BasicMaterial.mat).
    - The streams are initialized in the Start function (unless Skip Initialization is set). When the streams are initialized in this way, Spatial Input streaming is not supported and Enable SI must be unset.
4. Build the project for UWP (File -> Build Settings).
    1. Add your Unity scenes to Scenes in Build.
    2. Set Platform to Universal Windows Platform.
    3. Click Switch Platform.
    4. Set Target Device to HoloLens.
    5. Set Architecture to ARM64.
    6. Set Build Type to D3D Project.
    7. Set Target SDK Version to Latest installed.
    8. Set Visual Studio Version to Latest installed.
    9. Set Build and Run on Local Machine.
    10. Set Build configuration to Release.
    11. Click Build. Unity will ask for a destination folder. You can create a new one named Build.
5. Navigate to the Build folder and open the Visual Studio solution.
6. Open Package.appxmanifest and enable the following capabilities:
    - Gaze Input
    - Internet (Client & Server)
    - Internet (Client)
    - Microphone
    - Private Networks (Client & Server)
    - Spatial Perception
    - Webcam
7. Right click Package.appxmanifest, select Open With, and select HTML Editor. Edit Package.appxmanifest as follows:
    1. In Package add `xmlns:rescap="http://schemas.microsoft.com/appx/manifest/foundation/windows10/restrictedcapabilities"`.
    2. Under Capabilities add `<rescap:Capability Name="perceptionSensorsExperimental"/>`.
    3. Under Capabilities add `<DeviceCapability Name="backgroundSpatialPerception"/>`.
    - See the [Package.appxmanifest](https://github.com/jdibenes/hl2ss/blob/main/hl2ss/hl2ss/Package.appxmanifest) of the server for an example. Note that [the order in which Capabilites are declared matters](https://docs.microsoft.com/en-us/answers/questions/92754/how-to-use-extendedexecutionunconstrained.html).
8. Build Release ARM64.
9. In the Solution Explorer, right click the project and select Properties.
10. Navigate to Configuration Properties -> Debugging and set Machine Name to your HoloLens IP address.
11. Run. The application will remain installed on the HoloLens even after power off.

**Using the plugin with Spatial Input support**

1. Follow steps 1 through 3 of the previous section.
2. For the hl2ss script component of the Main Camera, set Skip Initialization and set Enable SI.
3. Follow steps 4 through 10 of the previous section.
4. In the Solution Explorer, right click the project and select Properties.
5. Nagivate to Configuration Properties -> C/C++ -> General -> Additional Include Directories and add the include directory of the plugin folder.
6. Nagivate to Configuration Properties -> Linker -> General -> Additional Library Directories and add the lib folder of the plugin folder.
7. Navigate to Configuration Properties -> Linker -> Input -> Additional Dependencies and add hl2ss.lib.
8. Open App.cpp and edit it as follows:
    1. `#include <hl2ss.h>` after the other includes.
    2. At the end of the `App::SetWindow(CoreWindow^ window)` method, right before the closing `}`, add `InitializeStreams(HL2SS_ENABLE_RM | HL2SS_ENABLE_MC | HL2SS_ENABLE_PV | HL2SS_ENABLE_SI | HL2SS_ENABLE_RC | HL2SS_ENABLE_SM);`.
9. Follow step 11 of the previous section.

**Remote Unity Scene**

The plugin has basic support for creating and controlling 3D primitives and text objects via TCP for the purpose of sending feedback to the HoloLens user. See the unity_demo Python scripts in the [viewer](https://github.com/jdibenes/hl2ss/tree/main/viewer) directory for some examples. Some of the supported features include:

- Create primitive: sphere, capsule, cylinder, cube, plane, and quad.
- Set active: enable or disable game object.
- Set world transform: position, rotation, and scale.
- Set color: rgba with support for semi-transparency.
- Set texture: upload png or jpg file.
- Create text: creates a TextMeshPro object.
- Set text: sets the text, font size and color of a TextMeshPro object.
- Remove: destroy game object.
- Remove all: destroy all game objects created by the plugin.

## References

This project uses the HoloLens 2 Research Mode API and the Cannon library, both available at the [HoloLens2ForCV](https://github.com/microsoft/HoloLens2ForCV) repository.
