# Optional Extensions

## client_cpp

C++ client library. Requires [OpenCV](https://opencv.org/releases/) and [FFmpeg](https://ffmpeg.org/download.html).

**Build**

Add `hl2ss.cpp`, `hl2ss.h`, `hl2ss_lnm.cpp`, `hl2ss_lnm.h`, `hl2ss_mt.cpp`, and `hl2ss_mt.h` to your C++ project.
See `main.cpp` for examples.
The `tasks_[...].json` files are provided as a reference for building with `cl.exe` (Windows) and `g++` (Linux).

Alternatively, you can build `hl2ss_ulm.cpp` as a shared library (.dll, .so, ...) and include `hl2ss_ulm.h` in your C++ project.
See `main_ulm.cpp` for examples.

If you build `main.cpp` or `main_ulm.cpp` on Windows, you may need to copy the FFmpeg (e.g., `avcodec-60.dll`, `avutil-58.dll`, `swresample-4.dll`) and OpenCV (e.g., `opencv_world480.dll`)
 DLLs to the client_cpp folder.

## client_matlab

C++ MEX client library for MATLAB. It uses client_cpp as a base so it has the same requirements. See the scripts in the [viewer](https://github.com/jdibenes/hl2ss/tree/main/extensions/client_matlab/viewer) folder for examples. See `viewer_simulink.slx` for using the client library in Simulink.

**Build**

Run `hl2ss_build_[...].m` to generate the MEX file. Add the client_matlab folder to MATLAB path (you can type `pathtool` in the MATLAB command window to do this).

On Windows, set the `ffmpeg_path` and `opencv_path` variables in the build script to the folders containing the FFmpeg and OpenCV libraries. After building the MEX file, copy the FFmpeg and OpenCV DLLs to the client_matlab folder.

## client_unity

C# wrapper for the hl2ss_ulm shared library. Has the same requirements as client_cpp. See the scripts in the Assets/Scripts/test folder for examples.

**Build**

Build the hl2ss_ulm shared library and copy it to the appropriate subfolder in Assets/Plugins. Also, copy the shared libraries for OpenCV and FFmpeg if necessary. Finally, build the Unity project.

## client_labview

Collection of LabVIEW VIs for streaming HoloLens 2 sensor data using the hl2ss_ulm shared library. Has the same requirements as client_cpp. See the `test_[...].vi` VIs for examples.

**Build**

Build the hl2ss_ulm shared library and copy it to the hl2ss_labview folder. Also, copy the shared libraries for OpenCV and FFmpeg if necessary.

## pyzdepth 

Zdepth wrapper for Python. Required to decompress RM Depth AHAT data configured with `hl2ss.DepthProfile.ZDEPTH`.

**Build**

`python3 extension_zdepth.py build`

After building copy the `pyzdepth.[...].pyd` (or `pyzdepth.[...].so`) file in the `build/lib.[...]` folder to the [viewer](https://github.com/jdibenes/hl2ss/tree/main/viewer) folder. On Windows, you might need to use `py` instead of `python3`.
