using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Dummiesman
{
    public static class BinaryExtensions
    {
        public static Color32 ReadColor32RGBR(this BinaryReader r)
        {
            var bytes = r.ReadBytes(4);
            return new Color32(bytes[0], bytes[1], bytes[2], 255);
        }

        public static Color32 ReadColor32RGBA(this BinaryReader r)
        {
            var bytes = r.ReadBytes(4);
            return new Color32(bytes[0], bytes[1], bytes[2], bytes[3]);
        }

        public static Color32 ReadColor32RGB(this BinaryReader r)
        {
            var bytes = r.ReadBytes(3);
            return new Color32(bytes[0], bytes[1], bytes[2], 255);
        }

        public static Color32 ReadColor32BGR(this BinaryReader r)
        {
            var bytes = r.ReadBytes(3);
            return new Color32(bytes[2], bytes[1], bytes[0], 255);
        }
    }
}