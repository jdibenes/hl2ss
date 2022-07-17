# HoloLens 2 Sensor Streaming

This README is WIP

Supported streams:

Front RGB Camera (PV): 1920x1080, 30 FPS, H264 or HEVC encoding. Resolution, framerate, encoding profile and average bitrate are configurable. See etc/hl2_capture_formats.txt for a list of supported resolutions and framerates.

Microphone (MC): 2 channels, 48000 Hz, AAC encoded. Bitrate configurable.

Research Mode (RM) Grayscale Cameras (VLC): 4 cameras, 640x480, 30 FPS, H264 or HEVC encoding. Encoding profile and average bitrate configurable.

Research Mode (RM) Long Throw Depth (LT): 320x288, 5 FPS, Depth+AB encoded as 32 bpp PNG.

Research Mode (RM) IMU: accelerometer, gyroscope, magnetometer.

Pose for the RGB camera and RM sensors (except magnetometer)

Intrinsics for the RGB, depth and VLC cameras, extrinsics for RM sensors (except magnetometer)

Head Tracking

Eye Tracking

Hand Tracking

The Python scripts in the viewer directory show how to receive the sensor data over TCP and decode it in realtime.

Unsupported streams:

Research Mode (RM) AHAT (not implemented)

Future work:

Voice commands from the winrt API

Send video, images, text, etc. to the HoloLens and display to the user

## Python dependencies

OpenCV

PyAV

PyAudio (optional)

Numpy

## Installing

Download the appxbundle located in the releases folder

Upload to the HoloLens via the Device Portal (see https://docs.microsoft.com/en-us/windows/mixed-reality/develop/advanced-concepts/using-the-windows-device-portal)

In your HoloLens, locate the appxbundle file in the File Explorer and tap it to open the installer (see: Instalation Method in https://docs.microsoft.com/en-us/hololens/app-deploy-app-installer)

Install

## Building:

Install the tools: https://docs.microsoft.com/en-us/windows/mixed-reality/develop/install-the-tools

Open the Visual Studio Solution (sln file in hl2ss folder) in Visual Studio 2022

Build Release ARM64

## Running:

Set Debugging -> Machine Name to your HoloLens IP address

Run

## Permissions:

Camera

Movements

Eye Tracker

Microphone

## References

Hololens2ForCV github

Universal Windows Platform Samples

MediaFoundation Samples

Classic Windows Samples

PyAV cookbook

Cannon library (included in the Visual Studio project)

Research Mode API (included in the Visual Studio project)

and others...
