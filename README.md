# HoloLens 2 Sensor Streaming

Supported Sensors:

Front RGB Camera (PV): 1920x1080, 30 FPS, H264 or HEVC encoding.

Microphone (MC): 2 channels, 48000 Hz, AAC encoded.

Research Mode (RM) Grayscale Cameras (VLC): 4 cameras, 640x480, 30 FPS, H264 or HEVC encoding.

Research Mode (RM) Long Throw Depth (LT): 320x288, 5 FPS, Depth+AB encoded as 32 bpp PNG.

Research Mode (RM) IMU: accelerometer, gyroscope, magnetometer.

The Python scripts in the viewer directory show how to receive the sensor data over TCP and decode it in realtime.

Python dependencies:

OpenCV

PyAV

PyAudio

Numpy

Building:

Open the solution in Visual Studio 2022

Build Release ARM64

Running:

Set Debugging -> Machine Name to your HoloLens IP address

Run









