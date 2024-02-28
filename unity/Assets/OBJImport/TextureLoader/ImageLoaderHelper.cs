using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

namespace Dummiesman
{
    public class ImageLoaderHelper
    {
        /// <summary>
        /// Verifies that a 32bpp texture is actuall 32bpp
        /// </summary>
        /// <returns>The verified texture</returns>
        public static Texture2D VerifyFormat(Texture2D tex)
        {
            if (tex.format != UnityEngine.TextureFormat.ARGB32 && tex.format != UnityEngine.TextureFormat.RGBA32 && tex.format != UnityEngine.TextureFormat.DXT5)
                return tex;

            //get pixels
            var pixels = tex.GetPixels32();
            bool validFormat = false;

            //check each pixel alpha
            foreach(var px in pixels)
            {
                if(px.a < 255)
                {
                    validFormat = true;
                    break;
                }
            }

            //if it's not a valid format return a new 24bpp image
            if (!validFormat)
            {
                var tex24 = new Texture2D(tex.width, tex.height, UnityEngine.TextureFormat.RGB24, tex.mipmapCount > 0);
                tex24.SetPixels32(pixels);
                tex24.Apply(true);
                return tex24;
            }

            //return original if valid
            return tex;
        }

        /// <summary>
        /// A cluster for creating arrays of Unity Color 
        /// </summary>
        /// <param name="fillArray"></param>
        /// <param name="pixelData"></param>
        /// <param name="bytesPerPixel"></param>
        /// <param name="bgra"></param>
        public static void FillPixelArray(Color32[] fillArray, byte[] pixelData, int bytesPerPixel, bool bgra = false)
        {
            //special case for TGA :(
            if (bgra)
            {
                if (bytesPerPixel == 4)
                {
                    for (int i = 0; i < fillArray.Length; i++)
                    {
                        int bi = i * bytesPerPixel;
                        fillArray[i] = new Color32(pixelData[bi + 2], pixelData[bi + 1], pixelData[bi], pixelData[bi + 3]);
                    }
                }
                else
                {
                    //24 bit BGR to Color32 (RGBA)
                    //this is faster than safe code
                    for (int i = 0; i < fillArray.Length; i++)
                    {
                        fillArray[i].r = pixelData[(i * 3) + 2];
                        fillArray[i].g = pixelData[(i * 3) + 1];
                        fillArray[i].b = pixelData[(i * 3) + 0];
                    }
                }
            }
            else
            {
                if (bytesPerPixel == 4)
                {
                    for (int i = 0; i < fillArray.Length; i++)
                    {
                        fillArray[i].r = pixelData[i * 4];
                        fillArray[i].g = pixelData[(i * 4) + 1];
                        fillArray[i].b = pixelData[(i * 4) + 2];
                        fillArray[i].a = pixelData[(i * 4) + 3];
                    }
                }
                else
                {
                    //with RGB we can't! :(
                    int bi = 0;
                    for (int i = 0; i < fillArray.Length; i++)
                    {
                        fillArray[i].r = pixelData[bi++];
                        fillArray[i].g = pixelData[bi++];
                        fillArray[i].b = pixelData[bi++];
                        fillArray[i].a = 255;
                    }
                }
            }
        }
    }
}