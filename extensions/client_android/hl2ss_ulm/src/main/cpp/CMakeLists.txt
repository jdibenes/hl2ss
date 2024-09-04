# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html.
# For more examples on how to use CMake, see https://github.com/android/ndk-samples.

# Sets the minimum CMake version required for this project.
cmake_minimum_required(VERSION 3.22.1)

# Declares the project name. The project name can be accessed via ${ PROJECT_NAME},
# Since this is the top level CMakeLists.txt, the project name is also accessible
# with ${CMAKE_PROJECT_NAME} (both CMake variables are in-sync within the top level
# build script scope).
project("hl2ss_ulm")

# TODO: set path to ffmpeg headers
set(FFMPEG_INCLUDE_DIR "C:/Users/jcds/SDK/ffmpeg-master-latest-win64-gpl-shared/include")

# TODO: set path to ffmpeg .so files
set(FFMPEG_LIB_DIR "C:/Users/jcds/SDK/Android/ffmpeg")

# TODO: set path to opencv headers
set(OPENCV_INCLUDE_DIR "C:/Users/jcds/SDK/opencv/build/include")

# TODO: set path to opencv .so files
set(OPENCV_LIB_DIR "C:/Users/jcds/SDK/Android/opencv")

# HL2SS C++ library path
set(HL2SS_CPP_DIR "../../../../../client_cpp")

# Zdepth library path
set(ZDEPTH_DIR "../../../../../../3rdparty/Zdepth")

# ffmpeg
add_library(avcodec             SHARED IMPORTED)
add_library(avdevice            SHARED IMPORTED)
add_library(avfilter            SHARED IMPORTED)
add_library(avformat            SHARED IMPORTED)
add_library(avutil              SHARED IMPORTED)
add_library(cpp_shared          SHARED IMPORTED)
add_library(ffmpegkit           SHARED IMPORTED)
add_library(ffmpegkit_abidetect SHARED IMPORTED)
add_library(swresample          SHARED IMPORTED)
add_library(swscale             SHARED IMPORTED)

set_target_properties(avcodec             PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libavcodec.so)
set_target_properties(avdevice            PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libavdevice.so)
set_target_properties(avfilter            PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libavfilter.so)
set_target_properties(avformat            PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libavformat.so)
set_target_properties(avutil              PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libavutil.so)
set_target_properties(cpp_shared          PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libc++_shared.so)
set_target_properties(ffmpegkit           PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libffmpegkit.so)
set_target_properties(ffmpegkit_abidetect PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libffmpegkit_abidetect.so)
set_target_properties(swresample          PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libswresample.so)
set_target_properties(swscale             PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libswscale.so)

add_library(avcodec_neon_arm64-v8a    STATIC library_stub.cpp)
add_library(avdevice_neon_arm64-v8a   STATIC library_stub.cpp)
add_library(avfilter_neon_arm64-v8a   STATIC library_stub.cpp)
add_library(avformat_neon_arm64-v8a   STATIC library_stub.cpp)
add_library(avutil_neon_arm64-v8a     STATIC library_stub.cpp)
add_library(ffmpegkit_neon_arm64-v8a  STATIC library_stub.cpp)
add_library(swresample_neon_arm64-v8a STATIC library_stub.cpp)
add_library(swscale_neon_arm64-v8a    STATIC library_stub.cpp)

add_library(avcodec_neon_armeabi-v7a    SHARED IMPORTED)
add_library(avdevice_neon_armeabi-v7a   SHARED IMPORTED)
add_library(avfilter_neon_armeabi-v7a   SHARED IMPORTED)
add_library(avformat_neon_armeabi-v7a   SHARED IMPORTED)
add_library(avutil_neon_armeabi-v7a     SHARED IMPORTED)
add_library(ffmpegkit_neon_armeabi-v7a  SHARED IMPORTED)
add_library(swresample_neon_armeabi-v7a SHARED IMPORTED)
add_library(swscale_neon_armeabi-v7a    SHARED IMPORTED)

set_target_properties(avcodec_neon_armeabi-v7a    PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libavcodec_neon.so)
set_target_properties(avdevice_neon_armeabi-v7a   PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libavdevice_neon.so)
set_target_properties(avfilter_neon_armeabi-v7a   PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libavfilter_neon.so)
set_target_properties(avformat_neon_armeabi-v7a   PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libavformat_neon.so)
set_target_properties(avutil_neon_armeabi-v7a     PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libavutil_neon.so)
set_target_properties(ffmpegkit_neon_armeabi-v7a  PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libffmpegkit_armv7a_neon.so)
set_target_properties(swresample_neon_armeabi-v7a PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libswresample_neon.so)
set_target_properties(swscale_neon_armeabi-v7a    PROPERTIES IMPORTED_LOCATION ${FFMPEG_LIB_DIR}/${ANDROID_ABI}/libswscale_neon.so)

add_library(avcodec_neon_x86    STATIC library_stub.cpp)
add_library(avdevice_neon_x86   STATIC library_stub.cpp)
add_library(avfilter_neon_x86   STATIC library_stub.cpp)
add_library(avformat_neon_x86   STATIC library_stub.cpp)
add_library(avutil_neon_x86     STATIC library_stub.cpp)
add_library(ffmpegkit_neon_x86  STATIC library_stub.cpp)
add_library(swresample_neon_x86 STATIC library_stub.cpp)
add_library(swscale_neon_x86    STATIC library_stub.cpp)

add_library(avcodec_neon_x86_64    STATIC library_stub.cpp)
add_library(avdevice_neon_x86_64   STATIC library_stub.cpp)
add_library(avfilter_neon_x86_64   STATIC library_stub.cpp)
add_library(avformat_neon_x86_64   STATIC library_stub.cpp)
add_library(avutil_neon_x86_64     STATIC library_stub.cpp)
add_library(ffmpegkit_neon_x86_64  STATIC library_stub.cpp)
add_library(swresample_neon_x86_64 STATIC library_stub.cpp)
add_library(swscale_neon_x86_64    STATIC library_stub.cpp)

# opencv
add_library(opencv SHARED IMPORTED)

set_target_properties(opencv PROPERTIES IMPORTED_LOCATION ${OPENCV_LIB_DIR}/${ANDROID_ABI}/libopencv_java4.so)

# Creates and names a library, sets it as either STATIC
# or SHARED, and provides the relative paths to its source code.
# You can define multiple libraries, and CMake builds them for you.
# Gradle automatically packages shared libraries with your APK.
#
# In this top level CMakeLists.txt, ${CMAKE_PROJECT_NAME} is used to define
# the target library name; in the sub-module's CMakeLists.txt, ${PROJECT_NAME}
# is preferred for the same purpose.
#
# In order to load a library into your app from Java/Kotlin, you must call
# System.loadLibrary() and pass the name of the library defined here;
# for GameActivity/NativeActivity derived applications, the same library name must be
# used in the AndroidManifest.xml file.
add_library(${CMAKE_PROJECT_NAME} SHARED
        # List C/C++ source files with relative paths to this CMakeLists.txt.
        ${HL2SS_CPP_DIR}/hl2ss_ulm.cpp
        ${HL2SS_CPP_DIR}/hl2ss.cpp
        ${HL2SS_CPP_DIR}/hl2ss_lnm.cpp
        ${HL2SS_CPP_DIR}/hl2ss_mt.cpp
        ${ZDEPTH_DIR}/src/zdepth.cpp
        ${ZDEPTH_DIR}/zstd/src/debug.c
        ${ZDEPTH_DIR}/zstd/src/entropy_common.c
        ${ZDEPTH_DIR}/zstd/src/error_private.c
        ${ZDEPTH_DIR}/zstd/src/fse_compress.c
        ${ZDEPTH_DIR}/zstd/src/fse_decompress.c
        ${ZDEPTH_DIR}/zstd/src/hist.c
        ${ZDEPTH_DIR}/zstd/src/huf_compress.c
        ${ZDEPTH_DIR}/zstd/src/huf_decompress.c
        ${ZDEPTH_DIR}/zstd/src/pool.c
        ${ZDEPTH_DIR}/zstd/src/threading.c
        ${ZDEPTH_DIR}/zstd/src/xxhash.c
        ${ZDEPTH_DIR}/zstd/src/zstd_common.c
        ${ZDEPTH_DIR}/zstd/src/zstd_compress.c
        ${ZDEPTH_DIR}/zstd/src/zstd_compress_literals.c
        ${ZDEPTH_DIR}/zstd/src/zstd_compress_sequences.c
        ${ZDEPTH_DIR}/zstd/src/zstd_ddict.c
        ${ZDEPTH_DIR}/zstd/src/zstd_decompress.c
        ${ZDEPTH_DIR}/zstd/src/zstd_decompress_block.c
        ${ZDEPTH_DIR}/zstd/src/zstd_double_fast.c
        ${ZDEPTH_DIR}/zstd/src/zstd_fast.c
        ${ZDEPTH_DIR}/zstd/src/zstd_lazy.c
        ${ZDEPTH_DIR}/zstd/src/zstd_ldm.c
        ${ZDEPTH_DIR}/zstd/src/zstd_opt.c
        ${ZDEPTH_DIR}/zstd/src/zstdmt_compress.c)

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        ${FFMPEG_INCLUDE_DIR}
        ${OPENCV_INCLUDE_DIR}
        ${HL2SS_CPP_DIR}
        ${ZDEPTH_DIR}/include
        ${ZDEPTH_DIR}/zstd/include
        ${ZDEPTH_DIR}/zstd/src)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE HL2SS_ENABLE_ZDEPTH)

# Specifies libraries CMake should link to your target library. You
# can link libraries from various origins, such as libraries defined in this
# build script, prebuilt third-party libraries, or Android system libraries.
target_link_libraries(${CMAKE_PROJECT_NAME}
        # List libraries link to the target library
        android
        log
        avcodec
        avdevice
        avfilter
        avformat
        avutil
        cpp_shared
        ffmpegkit
        ffmpegkit_abidetect
        swresample
        swscale
        avcodec_neon_${ANDROID_ABI}
        avdevice_neon_${ANDROID_ABI}
        avfilter_neon_${ANDROID_ABI}
        avformat_neon_${ANDROID_ABI}
        avutil_neon_${ANDROID_ABI}
        ffmpegkit_neon_${ANDROID_ABI}
        swresample_neon_${ANDROID_ABI}
        swscale_neon_${ANDROID_ABI}
        opencv)