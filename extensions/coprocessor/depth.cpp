
#include <opencv2/opencv.hpp>
#include "research_mode.h"
#include "export.h"

struct float2
{
    float x, y;
};

struct float3
{
    float x, y, z;
};

cv::Mat g_rays[] =
{ 
    cv::Mat(RM_ZHT_HEIGHT, RM_ZHT_WIDTH, CV_32FC3), 
    cv::Mat(RM_ZLT_HEIGHT, RM_ZLT_WIDTH, CV_32FC3),
    cv::Mat(RM_ZHT_HEIGHT, RM_ZHT_WIDTH, CV_32FC3),
    cv::Mat(RM_ZLT_HEIGHT, RM_ZLT_WIDTH, CV_32FC3),
};

static bool RM_DepthGetIndex(int id, int& index)
{
    switch (id)
    {
    case DEPTH_AHAT:       index = 0; break;
    case DEPTH_LONG_THROW: index = 1; break;
    default: return false;
    }

    return true;
}

PLUGIN_EXPORT
void RM_DepthInitializeRays(int id, float* uv2xy)
{
    int index;

    if (!RM_DepthGetIndex(id, index)) { return; }

    cv::Mat& rays1 = g_rays[index];
    cv::Mat& rays2 = g_rays[index + 2];

    float3* r1 = (float3*)rays1.data;
    float3* r2 = (float3*)rays2.data;

    int count = rays1.rows * rays1.cols;

    float* uv2x = uv2xy;
    float* uv2y = uv2xy + count;

    for (int row1 = 0; row1 < rays1.rows; ++row1)
    {
    for (int col1 = 0; col1 < rays1.cols; ++col1)
    {
    int row2 = (row1 + rays1.rows - 1) % rays1.rows;
    int col2 = (col1 + rays1.cols - 1) % rays1.cols;

    int i1 = (row1 * rays1.cols) + col1;
    int i2 = (row2 * rays1.cols) + col2;
 
    float x = uv2x[i1];
    float y = uv2y[i1];
    float n = sqrt((x * x) + (y * y) + 1.0f);

    r2[i2] = r1[i1] = { x / n, y / n, 1.0f / n };    
    }
    }
}

PLUGIN_EXPORT
void RM_DepthNormalize(int id, uint16_t* depth_in, float* depth_out)
{
    int index;

    if (!RM_DepthGetIndex(id, index)) { return; }

    cv::Mat& rays = g_rays[index];

    cv::Mat in  = cv::Mat(rays.rows, rays.cols, CV_16UC1, depth_in);
    cv::Mat out = cv::Mat(rays.rows, rays.cols, CV_32FC1, depth_out);

    in.convertTo(out, CV_32FC1, 1.0 / 1000.0);
}

PLUGIN_EXPORT
void RM_DepthRepeat(int id, float* depth, float* depth3)
{
    int index;

    if (!RM_DepthGetIndex(id, index)) { return; }

    cv::Mat& rays = g_rays[index];

    cv::Mat i1 = cv::Mat(rays.rows, rays.cols, CV_32FC1, depth);
    cv::Mat o3 = cv::Mat(rays.rows, rays.cols, CV_32FC3, depth3);

    cv::cvtColor(i1, o3, cv::COLOR_GRAY2BGR);
}

PLUGIN_EXPORT
void RM_DepthTo3D(int id, float* depth3, float* points)
{
    int index;

    if (!RM_DepthGetIndex(id, index)) { return; }

    cv::Mat& rays = g_rays[index];

    cv::Mat in3 = cv::Mat(rays.rows, rays.cols, CV_32FC3, depth3);
    cv::Mat out = cv::Mat(rays.rows, rays.cols, CV_32FC3, points);
    
    cv::multiply(rays, in3, out);
}

PLUGIN_EXPORT
void RM_DepthFill2Build(int id, float* depth3, float* points_ul, float* points_br)
{
    int index;

    if (!RM_DepthGetIndex(id, index)) { return; }

    cv::Mat& rays1 = g_rays[index];
    cv::Mat& rays2 = g_rays[index + 2];

    cv::Mat in3    = cv::Mat(rays1.rows, rays1.cols, CV_32FC3, depth3);
    cv::Mat out_ul = cv::Mat(rays1.rows, rays1.cols, CV_32FC3, points_ul);
    cv::Mat out_ur = cv::Mat(rays1.rows, rays1.cols, CV_32FC3, points_br);

    cv::multiply(rays1, in3, out_ul);
    cv::multiply(rays2, in3, out_ur);
}

PLUGIN_EXPORT
void RM_DepthFill1(int id, float* depth, float* image_points, float* local_depths, float* out, int width, int height)
{
    int index;

    if (!RM_DepthGetIndex(id, index)) { return; }

    cv::Mat& rays = g_rays[index];

    float2* uv = (float2*)image_points;
    int br = rays.cols + 1;

    for (int row = 0; row < rays.rows - 1; ++row)
    {
    int h = row * rays.cols;
    for (int col = 0; col < rays.cols - 1; ++col)
    {
    int i1 = h + col;
    int i2 = i1 + br;

    if (depth[i1] <= 0) { continue; }
    if (depth[i2] <= 0) { continue; }

    float2 p1 = uv[i1];
    float2 p2 = uv[i2];
    
    float z1 = local_depths[i1];
  //float z2 = local_depths[i2];

    for (int v = (int)p1.y; v < (((int)p2.y) + 1); ++v)
    {
    if ((v < 0) || (v >= height)) { continue; }
    int c = v * width;
    for (int u = (int)p1.x; u < (((int)p2.x) + 1); ++u)
    {
    if ((u < 0) || (u >= width)) { continue; }
    out[c + u] = z1;
    }
    }
    }
    }
}

PLUGIN_EXPORT
void RM_DepthFill2(int id, float* depth, float* image_points_ul, float* image_points_br, float* local_depths, float* out, int width, int height)
{
    int index;

    if (!RM_DepthGetIndex(id, index)) { return; }

    cv::Mat& rays = g_rays[index];

    float2* uv1 = (float2*)image_points_ul;
    float2* uv2 = (float2*)image_points_br;

    for (int row = 0; row < (rays.rows - 1); ++row)
    {
    int h = row * rays.cols;
    for (int col = 0; col < (rays.cols - 1); ++col)
    {
    int i = h + col;

    if (depth[i] <= 0) { continue; }

    float2 p1 = uv1[i];
    float2 p2 = uv2[i];

    for (int v = (int)p1.y; v < ((int)p2.y) + 1; ++v)
    {
    if ((v < 0) || (v >= height)) { continue; }
    int c = v * width;
    for (int u = (int)p1.x; u < ((int)p2.x) + 1; ++u)
    {
    if ((u < 0) || (u >= width)) { continue; }
    out[c + u] = local_depths[i];
    }
    }
    }
    }
}
