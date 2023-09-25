# Optional Extensions

## client_cpp

C++ client library. Requires [OpenCV](https://opencv.org/releases/) and [FFmpeg](https://ffmpeg.org/download.html). See `main.cpp` for examples.

**Build**

Add `hl2ss.cpp`, `hl2ss.h`, `hl2ss_lnm.cpp`, and `hl2ss_lnm.h` to your C++ project.
The `tasks_[...].json` files are provided as a reference for building with `cl.exe` (Windows) and `g++` (Linux).

If you build `main.cpp` on Windows, you may need to copy the FFmpeg and OpenCV DLLs to the client_cpp folder.

## client_matlab

C++ MEX client library for MATLAB. It uses client_cpp as a base so it has the same requirements. See the scripts in the [viewer](https://github.com/jdibenes/hl2ss/tree/main/extensions/client_matlab/viewer) folder for examples.

**Build**

Run `hl2ss_build_[...].m` to generate the MEX file. Add the client_matlab folder to MATLAB path (you can type `pathtool` in the MATLAB command window to do this).

On Windows, set the `ffmpeg_path` and `opencv_path` variables in the build script to the folders containing the FFmpeg and OpenCV libraries. After building the MEX file, copy the FFmpeg and OpenCV DLLs to the client_matlab folder.

## pyzdepth 

Zdepth wrapper for Python. Required to decompress RM Depth AHAT data configured with `hl2ss.DepthProfile.ZDEPTH`.

**Build**

`python3 extension_zdepth.py build`

After building copy the `pyzdepth.[...].pyd` (or `pyzdepth.[...].so`) file in the `build/lib.[...]` folder to the [viewer](https://github.com/jdibenes/hl2ss/tree/main/viewer) folder. On Windows, you might need to use `py` instead of `python3`.
