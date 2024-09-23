
using System;
using System.Runtime.InteropServices;

public static partial class hl2da
{
    public static class itc
    {
        [DllImport("hl2da_itc")]
        public static extern void Crop(IntPtr image_in, int width_in, int height_in, int bpp_in, IntPtr image_out, int x_out, int y_out, int width_out, int height_out);

        [DllImport("hl2da_itc")]
        public static extern void RM_Undistort(int id, IntPtr mapxy, int interpolation, int border_mode, ushort border_value, IntPtr image_in, IntPtr image_out);

        [DllImport("hl2da_itc")]
        public static extern void RM_ToBGRX(int id, IntPtr image_in, byte alpha, IntPtr image_out);

        [DllImport("hl2da_itc")]
        public static extern void RM_DepthInitializeRays(int id, IntPtr uv2xy);

        [DllImport("hl2da_itc")]
        public static extern void RM_DepthNormalize(int id, IntPtr depth_in, IntPtr depth_out);

        [DllImport("hl2da_itc")]
        public static extern void RM_DepthRepeat(int id, IntPtr depth, IntPtr depth3);

        [DllImport("hl2da_itc")]
        public static extern void RM_DepthTo3D(int id, IntPtr depth3, IntPtr points);

        [DllImport("hl2da_itc")]
        public static extern void RM_DepthFill2Build(int id, IntPtr depth3, IntPtr points_ul, IntPtr points_br);

        [DllImport("hl2da_itc")]
        public static extern void RM_DepthFill1(int id, IntPtr depth, IntPtr image_points, IntPtr local_depths, IntPtr depth_out, int width, int height);

        [DllImport("hl2da_itc")]
        public static extern void RM_DepthFill2(int id, IntPtr depth, IntPtr image_points_ul, IntPtr image_points_br, IntPtr local_depths, IntPtr depth_out, int width, int height);

        [DllImport("hl2da_itc")]
        public static extern void TransformPoints3(IntPtr transform, IntPtr points_in, int count, IntPtr points_out);

        [DllImport("hl2da_itc")]
        public static extern void GetPoints3Channel(IntPtr points_in, int count, int channel, IntPtr points_out);

        [DllImport("hl2da_itc")]
        public static extern void ProjectPoints3(IntPtr intrinsics, IntPtr transform, IntPtr points_in, int count, IntPtr points_out);
    }
}
