
using System;

public static partial class hl2da
{
    public static class coprocessor
    {
        public static T[,,] Crop<T>(IntPtr image_in, int width_in, int height_in, int channels_in, int x_out, int y_out, int width_out, int height_out)
        {
            T[,,] r = new T[height_out, width_out, channels_in];
            using (pointer h = pointer.get(r)) { hl2da.itc.Crop(image_in, width_in, height_in, channels_in, h.value, x_out, y_out, width_out, height_out); }
            return r;
        }

        public static T[,,] RM_Undistort<T>(hl2da.SENSOR_ID id, float[,] mapxy, int interpolation, int border_mode, ushort border_value, IntPtr image_in)
        {
            if (!RM_GetResolution(id, out int w, out int h)) { return null; }
            T[,,] r = new T[h, w, 1];
            using (pointer h1 = pointer.get(mapxy), h2 = pointer.get(r)) { hl2da.itc.RM_Undistort((int)id, h1.value, interpolation, border_mode, border_value, image_in, h2.value); }
            return r;
        }

        public static T[,,] RM_ToBGRX<T>(hl2da.SENSOR_ID id, IntPtr image_in, bool alpha)
        {
            if (!RM_GetResolution(id, out int w, out int h)) { return null; }
            T[,,] r = new T[h, w, alpha ? 4 : 3];
            using (pointer p = pointer.get(r)) { hl2da.itc.RM_ToBGRX((int)id, image_in, (byte)(alpha ? 1U : 0), p.value); }
            return r;
        }

        public static void RM_DepthInitializeRays<T>(hl2da.SENSOR_ID id, T uv2xy)
        {
            using (pointer p = pointer.get(uv2xy)) { hl2da.itc.RM_DepthInitializeRays((int)id, p.value); }
        }

        public class rgbd_align : IDisposable
        {
            private int id;
            private int pixels;

            private float[,] identity;
            private float[,] K;
            private float[,,] depth_f32;
            private float[,,] depth3_f32;
            private float[,,] depth_points;
            private float[,,] camera_points;
            private float[,,] local_depth;
            private float[,,] local_points;
            private float[,,] depth_points2;
            private float[,,] camera_points2;
            private float[,,] local_points2;

            private pointer p_identity;
            private pointer p_K;
            private pointer p_depth_f32;
            private pointer p_depth3_f32;
            private pointer p_depth_points;
            private pointer p_camera_points;
            private pointer p_local_depth;
            private pointer p_local_points;
            private pointer p_depth_points2;
            private pointer p_local_points2;

            private rgbd_align(SENSOR_ID sensor_id, int w, int h)
            {
                id = (int)sensor_id;
                pixels = w * h;

                identity = new float[4, 4] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
                K = new float[3, 3] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

                depth_f32 = new float[h, w, 1];
                depth3_f32 = new float[h, w, 3];
                depth_points = new float[h, w, 3];
                camera_points = new float[h, w, 3];
                local_depth = new float[h, w, 1];
                local_points = new float[h, w, 2];
                depth_points2 = new float[h, w, 3];
                camera_points2 = new float[h, w, 3];
                local_points2 = new float[h, w, 2];

                p_identity = pointer.get(identity);
                p_K = pointer.get(K);
                p_depth_f32 = pointer.get(depth_f32);
                p_depth3_f32 = pointer.get(depth3_f32);
                p_depth_points = pointer.get(depth_points);
                p_camera_points = pointer.get(camera_points);
                p_local_depth = pointer.get(local_depth);
                p_local_points = pointer.get(local_points);
                p_depth_points2 = pointer.get(depth_points2);
                p_local_points2 = pointer.get(local_points2);
            }

            public static rgbd_align create(hl2da.SENSOR_ID id)
            {
                return RM_GetResolution(id, out int w, out int h) ? new rgbd_align(id, w, h) : null;
            }

            public float[,,] align(int algorithm, IntPtr depth_u16, IntPtr depth2camera, float[] k, int width, int height)
            {
                K[0, 0] = k[0];
                K[1, 1] = k[1];
                K[0, 2] = k[2];
                K[1, 2] = k[3];

                float[,,] camera_depth = new float[height, width, 1];

                using pointer p_camera_depth = pointer.get(camera_depth);

                switch (algorithm)
                {
                case 0: align_1(depth_u16, depth2camera, k, width, height, p_camera_depth.value); break;
                case 1: align_2(depth_u16, depth2camera, k, width, height, p_camera_depth.value); break;
                }

                return camera_depth;
            }

            private void align_1(IntPtr depth_u16, IntPtr depth2camera, float[] k, int width, int height, IntPtr depth_out)
            {
                hl2da.itc.RM_DepthNormalize(id, depth_u16, p_depth_f32.value);
                hl2da.itc.RM_DepthRepeat(id, p_depth_f32.value, p_depth3_f32.value);
                hl2da.itc.RM_DepthTo3D(id, p_depth3_f32.value, p_depth_points.value);
                hl2da.itc.TransformPoints3(depth2camera, p_depth_points.value, pixels, p_camera_points.value);
                hl2da.itc.GetPoints3Channel(p_camera_points.value, pixels, 2, p_local_depth.value);
                hl2da.itc.ProjectPoints3(p_K.value, p_identity.value, p_camera_points.value, pixels, p_local_points.value);
                hl2da.itc.RM_DepthFill1(id, p_depth_f32.value, p_local_points.value, p_local_depth.value, depth_out, width, height);
            }

            private void align_2(IntPtr depth_u16, IntPtr depth2camera, float[] k, int width, int height, IntPtr depth_out)
            {
                hl2da.itc.RM_DepthNormalize(id, depth_u16, p_depth_f32.value);
                hl2da.itc.RM_DepthRepeat(id, p_depth_f32.value, p_depth3_f32.value);
                hl2da.itc.RM_DepthFill2Build(id, p_depth3_f32.value, p_depth_points.value, p_depth_points2.value);
                hl2da.itc.TransformPoints3(depth2camera, p_depth_points.value, pixels, p_camera_points.value);
                hl2da.itc.GetPoints3Channel(p_camera_points.value, pixels, 2, p_local_depth.value);
                hl2da.itc.ProjectPoints3(p_K.value, p_identity.value, p_camera_points.value, pixels, p_local_points.value);
                hl2da.itc.ProjectPoints3(p_K.value, depth2camera, p_depth_points2.value, pixels, p_local_points2.value);
                hl2da.itc.RM_DepthFill2(id, p_depth_f32.value, p_local_points.value, p_local_points2.value, p_local_depth.value, depth_out, width, height);
            }

            protected virtual void Dispose(bool disposing)
            {
                if (!disposing) { return; }
                p_identity.Dispose();
                p_K.Dispose();
                p_depth_f32.Dispose();
                p_depth3_f32.Dispose();
                p_depth_points.Dispose();
                p_camera_points.Dispose();
                p_local_depth.Dispose();
                p_local_points.Dispose();
                p_depth_points2.Dispose();
                p_local_points2.Dispose();
            }

            public void Dispose()
            {
                Dispose(true);
                GC.SuppressFinalize(this);
            }

            ~rgbd_align()
            {
                Dispose(false);
            }
        }
    }
}
