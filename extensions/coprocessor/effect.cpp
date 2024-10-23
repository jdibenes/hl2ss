
#include <opencv2/opencv.hpp>
#include "research_mode.h"
#include "export.h"

PLUGIN_EXPORT
void Crop(void* image_in, int width_in, int height_in, int bpp_in, void* image_out, int x_out, int y_out, int width_out, int height_out)
{
    cv::Mat p = cv::Mat(height_in,  width_in,  CV_8UC(bpp_in), image_in);
    cv::Mat q = cv::Mat(height_out, width_out, CV_8UC(bpp_in), image_out);

    p(cv::Range(y_out, y_out + height_out), cv::Range(x_out, x_out + width_out)).copyTo(q);
}

PLUGIN_EXPORT
void RM_Undistort(int id, float* mapxy, int interpolation, int border_mode, uint16_t border_value, void* image_in, void* image_out)
{
    int width;
    int height;
    int type;

    switch (id)
    {
    case LEFT_FRONT:
    case LEFT_LEFT:
    case RIGHT_FRONT:
    case RIGHT_RIGHT:      width = RM_VLC_WIDTH; height = RM_VLC_HEIGHT; type = CV_8UC1;  break;
    case DEPTH_AHAT:       width = RM_ZHT_WIDTH; height = RM_ZHT_HEIGHT; type = CV_16UC1; break;
    case DEPTH_LONG_THROW: width = RM_ZLT_WIDTH; height = RM_ZLT_HEIGHT; type = CV_16UC1; break; // do not undistort sigma
    default: return;
    }

    cv::Mat in  = cv::Mat(height, width, type, image_in);
    cv::Mat out = cv::Mat(height, width, type, image_out);

    cv::Mat x = cv::Mat(height, width, CV_32FC1, mapxy);
    cv::Mat y = cv::Mat(height, width, CV_32FC1, mapxy + (width * height));

    cv::remap(in, out, x, y, interpolation, border_mode, cv::Scalar(border_value));
}

PLUGIN_EXPORT
void RM_ToBGRX(int id, void* image_in, bool alpha, void* image_out)
{
    int c = alpha ? 4 : 3;
    int width;
    int height;
    int type_in;
    int type_out;

    switch (id)
    {
    case LEFT_FRONT:
    case LEFT_LEFT:
    case RIGHT_FRONT:
    case RIGHT_RIGHT:      width = RM_VLC_WIDTH; height = RM_VLC_HEIGHT; type_in = CV_8UC1;  type_out = CV_8UC(c);  break;
    case DEPTH_AHAT:       width = RM_ZHT_WIDTH; height = RM_ZHT_HEIGHT; type_in = CV_16UC1; type_out = CV_16UC(c); break;
    case DEPTH_LONG_THROW: width = RM_ZLT_WIDTH; height = RM_ZLT_HEIGHT; type_in = CV_16UC1; type_out = CV_16UC(c); break;
    default: return;
    }

    cv::Mat in  = cv::Mat(height, width, type_in,  image_in);
    cv::Mat out = cv::Mat(height, width, type_out, image_out);

    cv::cvtColor(in, out, alpha ? cv::COLOR_GRAY2BGRA : cv::COLOR_GRAY2BGR);
}
