# HoloLens 2 Sensor Streaming

HoloLens 2 server application for streaming sensor data via TCP. Created to stream HoloLens data to a Linux machine for research purposes.

**Supported streams**

- Front Camera (1920x1080 @ 30 FPS, H264 or HEVC encoded)
- Microphone (2 channels, 48000 Hz, AAC encoded)
- Spatial Input (60 Hz)
  - Head Tracking
  - Eye Tracking
  - Hand Tracking
- Research Mode Visible Light Cameras (4 cameras, 640x480 @ 30 FPS, Grayscale, H264 or HEVC encoded)
- Research Mode Depth
  - Long Throw (320x288 @ 5 FPS, 16-bit Depth + 16-bit AB encoded as a single 32-bit PNG)
- Research Mode IMU
  - Accelerometer
  - Gyroscope
  - Magnetometer
  
**Unsupported streams**

- Research Mode Depth
  - AHAT
  
**Additional features**

- Download calibration (e.g., camera intrinsics) for the Front Camera and Research Mode sensors (except Magnetometer).
- Optional per-frame pose for the Front Camera and Research Mode sensors streams.
- Client can configure the bitrate of the H264, HEVC, and AAC encoded streams.
- Client can configure the resolution and framerate of the Front Camera (see [etc/hl2_capture_formats.txt](https://github.com/jdibenes/hl2ss/blob/main/etc/hl2_capture_formats.txt) for a list of supported formats).

The Python scripts in the [viewer](https://github.com/jdibenes/hl2ss/tree/main/viewer) directory demonstrate how to connect to the server, receive the data, unpack it, and decode it in real time.

## Preparation

Before using the server software, configure your HoloLens as follows:

1. Enable developer mode: Settings -> Update & Security -> For developers -> Use developer features.
2. Enable device portal: Settings -> Update & Security -> For developers -> Device Portal.
3. Enable research mode: Refer to the Setup section of https://github.com/microsoft/HoloLens2ForCV.

## Installation (sideloading)

The server software is distributed as a single appxbundle file.

1. Download the [latest appxbundle](https://github.com/jdibenes/hl2ss/releases).
2. Go to the Device Portal (type the IP address of your HoloLens in the address bar of your preferred web browser) and upload the appxbundle to the HoloLens (System -> File explorer -> Downloads).
3. On your HoloLens, open the File Explorer and locate the appxbundle.
4. Tap the appxbundle file to open the installer and tap Install.

## Permissions

The first time the server runs it will ask for the necessary permissions to access sensor data. If there are any issues, please verify that the server application (hl2ss.exe) has access to:

- Camera (Settings -> Privacy -> Camera).
- Eye tracker (Settings -> Privacy -> Eye tracker).
- Microphone (Settings -> Privacy -> Microphone).
- User movements (Settings -> Privacy -> User movements).

## Python dependencies

The sample Python scripts depend on the following packages:

- [OpenCV](https://github.com/opencv/opencv-python) `pip install opencv-python`
- [PyAV](https://github.com/PyAV-Org/PyAV) `pip install av`
- [PyAudio](https://people.csail.mit.edu/hubert/pyaudio/) `pip install PyAudio`
- [NumPy](https://numpy.org/) `pip install numpy`

## Building

Building requires a Windows 10 machine:

1. Install the tools: https://docs.microsoft.com/en-us/windows/mixed-reality/develop/install-the-tools.
2. Open the Visual Studio Solution (sln file in [hl2ss](https://github.com/jdibenes/hl2ss/tree/main/hl2ss) folder) in Visual Studio 2022.
3. Build Release ARM64.
4. In the Solution Explorer, right click the hl2ss project and select Properties.
5. Navigate to Configuration Properties -> Debugging and set Machine Name to your HoloLens IP address.
6. Run. The application will remain installed on the HoloLens even after power off.

## Details

The server application is a Native C++ Universal Windows Platform (UWP) application.
Each sensor stream has its own thread.
Multiple streams can be active at the same time but only one client per stream is allowed.

## Unity plugin (preview)

For streaming sensor data from a Unity application and creating primitives remotely. Spatial input and IMU streams are currently not supported. Front Camera stream is currently disabled in order to use the Device Portal Mixed Reality capture for Hologram visualization.

**Installation**

1. Download the [plugin](https://github.com/jdibenes/hl2ss/releases) and add it to your Unity project (put the plugin in Assets/Plugins/WSA).
2. In the Unity Editor, configure the plugin as UWP ARM64.
3. Add the [hl2ss.cs](https://github.com/jdibenes/hl2ss/blob/main/unity/hl2ss.cs) script to the Main Camera.
4. Build the project for UWP but do not run it yet.
5. Navigate to the Build folder and open the Visual Studio solution.
6. Open Package.appxmanifest and enable the following capabilities:
    - Gaze Input
    - Internet (Client & Server)
    - Internet (Client)
    - Microphone
    - Private Networks (CLient & Server)
    - Spatial Perception
    - Webcam
7. Right click the project -> Properties -> Configuration Properties -> Debugging and set Machine Name to your HoloLens IP address.
8. Run.

The Python scripts in the [unity](https://github.com/jdibenes/hl2ss/tree/main/unity) directory show how to connect to the server and create primitives (e.g., cubes, textured quads) or 3D text objects via TCP.

## References

This project uses the HoloLens 2 Research Mode API and the Cannon library, both available at https://github.com/microsoft/HoloLens2ForCV.
