#------------------------------------------------------------------------------
# Script for building hl2ss_ulm.dll
# Run x64 Native Tools Command Prompt for VS 2022 then run this script in the
# prompt
#------------------------------------------------------------------------------

import os

# Settings --------------------------------------------------------------------
# Adjust paths, library names, and defines as needed

external_include_paths = [
    '"C:/Users/jcds/SDK/opencv/build/include"',
    '"C:/Users/jcds/SDK/ffmpeg-master-latest-win64-gpl-shared/include"',
]

external_library_paths = [
    '"C:/Users/jcds/SDK/opencv/build/x64/vc16/lib"',
    '"C:/Users/jcds/SDK/ffmpeg-master-latest-win64-gpl-shared/lib"',
]

external_library_files = [
    'opencv_world480.lib',
    'avcodec.lib',
    'avutil.lib',
]

preprocessor_defines = [
]

#------------------------------------------------------------------------------
# No need to modify anything below this line, hopefully

internal_include_paths = [
    '../../3rdparty/Zdepth/include',
    '../../3rdparty/Zdepth/zstd/include',
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
    'kernel32.lib',
    'user32.lib',
    'gdi32.lib',
    'winspool.lib',
    'comdlg32.lib',
    'advapi32.lib',
    'shell32.lib',
    'ole32.lib',
    'oleaut32.lib',
    'uuid.lib',
    'odbc32.lib',
    'odbccp32.lib',                
    'Ws2_32.lib',
]

compiler_options = [
    '/permissive-',
    '/GS',
    '/GL',
    '/W3',
    '/Gy',
    '/Zc:wchar_t',
    '/Zi',
    '/Gm-',
    '/O2',
    '/sdl-',
    '/Zc:inline',
    '/fp:precise',
    '/WX-',
    '/Zc:forScope',
    '/Gd',
    '/Oi',
    '/MD',
    '/FC',
    '/EHsc',
    '/nologo',
    '/Fehl2ss_ulm.dll',
]

linker_options = [
    "/NXCOMPAT",
    "/DYNAMICBASE",
    "/MACHINE:X64",
    "/SUBSYSTEM:CONSOLE",
    "/OPT:ICF",
    "/NOLOGO",
    "/DLL",
]

string_includes = ' '.join([f'/I{path}' for path in (external_include_paths + internal_include_paths)])
string_defines  = ' '.join([f'/D{pdef}' for pdef in (preprocessor_defines)])
string_compiler = ' '.join(compiler_options)
string_sources  = ' '.join(internal_source_files)
string_libpath  = ' '.join([f'/LIBPATH:{lpth}' for lpth in (external_library_paths)])
string_linker   = ' '.join(linker_options)
string_inputs   = ' '.join(internal_library_files + external_library_files)

set_compiler = ' '.join([string_includes, string_defines, string_compiler, string_sources])
set_linker   = ' '.join([string_libpath, string_linker, string_inputs])

cmd = f'cl.exe {set_compiler} /link {set_linker}'

print(cmd)
os.system(cmd)
