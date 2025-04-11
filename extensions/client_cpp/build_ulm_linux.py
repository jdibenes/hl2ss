#------------------------------------------------------------------------------
# Script for building hl2ss_ulm.so
# Install OpenCV and FFmpeg libraries on your system then run this script
#------------------------------------------------------------------------------

import os

# Settings --------------------------------------------------------------------
# Adjust as needed

external_include_paths = [
    '/usr/include/opencv4',
]

external_library_paths = [
    '/usr/local/lib',
]

external_library_files = [
    'avcodec',
    'avutil',
    'opencv_highgui',
    'opencv_imgcodecs',
    'opencv_imgproc',
    'opencv_core',
]

preprocessor_defines = [
]

#------------------------------------------------------------------------------
# No need to modify anything below this line, hopefully

internal_include_paths = [
    '../../3rdparty/Zdepth/include',
    '../../3rdparty/Zdepth/zstd/include',
    '../../3rdparty/Zdepth/zstd/src',
]

internal_source_files = [
    'hl2ss.cpp',
    'hl2ss_lnm.cpp',
    'hl2ss_mt.cpp',
    'hl2ss_ulm.cpp',
    '../../3rdparty/Zdepth/src/*.cpp',
    '../../3rdparty/Zdepth/zstd/src/*.c',
]

internal_library_files = [
]

options = [
    '-fdiagnostics-color=always',
    '-g',
    '-o libhl2ss_ulm.so',
    '-shared',
    '-fPIC',
    '--std=c++17',
    '-O3',
]

string_sources  = ' '.join(internal_source_files)
string_compiler = ' '.join(options)
string_defines  = ' '.join([f'-D{pdef}' for pdef in (preprocessor_defines)])
string_includes = ' '.join([f'-I{path}' for path in (external_include_paths + internal_include_paths)])
string_libpath  = ' '.join([f'-L{lpth}' for lpth in (external_library_paths)])
string_inputs   = ' '.join([f'-l{lbin}' for lbin in (internal_library_files + external_library_files)])

set_compiler = ' '.join([string_sources, string_compiler, string_defines, string_includes, string_libpath, string_inputs])

cmd = f'g++ {set_compiler}'

print(cmd)
os.system(cmd)
