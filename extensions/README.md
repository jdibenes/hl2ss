# Optional Extensions

## client_cpp

C++ client library. Requires OpenCV and FFmpeg. See `main.cpp` for examples.

**Build**

Include `hl2ss.cpp`, `hl2ss.h`, `hl2ss_lnm.cpp`, `hl2ss_lnm.h`, and `types.h` in your C++ project.
See the `tasks_[...].json.bak` files for building with `cl.exe` (Windows) and `g++` (Linux).

## pyzdepth 

Zdepth wrapper for Python. Required to decompress RM Depth AHAT data configured with `hl2ss.DepthProfile.ZDEPTH`.

**Build**

`python3 extension_zdepth.py build`

On Windows, you might need to use `py` instead of `python3`. After building copy the `pyzdepth.[...].pyd` (or `pyzdepth.[...].so`) file in the `build/lib.[...]` folder to the [viewer](https://github.com/jdibenes/hl2ss/tree/main/viewer) folder.
