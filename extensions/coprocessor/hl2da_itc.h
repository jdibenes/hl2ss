
#pragma once

#include <stdint.h>

#define PLUGIN_IMPORT extern "C" __declspec(dllimport)

PLUGIN_IMPORT
void Crop(void* image_in, int width_in, int height_in, int bpp_in, void* image_out, int x_out, int y_out, int width_out, int height_out);

PLUGIN_IMPORT
void RM_Undistort(int id, float* mapxy, int interpolation, int border_mode, uint16_t border_value, void* image_in, void* image_out);

PLUGIN_IMPORT
void RM_ToBGRX(int id, void* image_in, bool alpha, void* image_out);

PLUGIN_IMPORT
void RM_DepthInitializeRays(int id, float* uv2xy);

PLUGIN_IMPORT
void RM_DepthNormalize(int id, uint16_t* depth_in, float* depth_out);

PLUGIN_IMPORT
void RM_DepthRepeat(int id, float* depth, float* depth3);

PLUGIN_IMPORT
void RM_DepthTo3D(int id, float* depth3, float* points);

PLUGIN_IMPORT
void RM_DepthFill2Build(int id, float* depth3, float* points_ul, float* points_br);

PLUGIN_IMPORT
void RM_DepthFill1(int id, float* depth, float* image_points, float* local_depths, float* out, int width, int height);

PLUGIN_IMPORT
void RM_DepthFill2(int id, float* depth, float* image_points_ul, float* image_points_br, float* local_depths, float* out, int width, int height);

PLUGIN_IMPORT
void TransformPoints3(float* transform, float* points_in, int count, float* points_out);

PLUGIN_IMPORT
void GetPoints3Channel(float* points_in, int count, int channel, float* points_out);

PLUGIN_IMPORT
void ProjectPoints3(float* intrinsics, float* transform, float* points_in, int count, float* points_out);
