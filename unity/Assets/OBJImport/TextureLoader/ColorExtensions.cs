using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Dummiesman.Extensions
{
    public static class ColorExtensions
    {
        public static Color FlipRB(this Color color)
        {
            return new Color(color.b, color.g, color.r, color.a);
        }

        public static Color32 FlipRB(this Color32 color)
        {
            return new Color32(color.b, color.g, color.r, color.a);
        }
    }
}