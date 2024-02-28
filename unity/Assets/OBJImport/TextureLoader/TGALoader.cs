using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System;
using Dummiesman.Extensions;
using System.Runtime.InteropServices;

namespace Dummiesman
{
    public class TGALoader
    {
        private static int GetBits(byte b, int offset, int count)
        {
            return (b >> offset) & ((1 << count) - 1);
        }

        private static Color32[] LoadRawTGAData(BinaryReader r, int bitDepth, int width, int height)
        {
            Color32[] pulledColors = new Color32[width * height];

            byte[] colorData = r.ReadBytes(width * height * (bitDepth / 8));
            ImageLoaderHelper.FillPixelArray(pulledColors, colorData, (bitDepth / 8), true);

            return pulledColors;
        }

        private static Color32[] LoadRLETGAData(BinaryReader r, int bitDepth, int width, int height)
        {
            Color32[] pulledColors = new Color32[width * height];
            int pulledColorCount = 0;

            while (pulledColorCount < pulledColors.Length)
            {
                byte rlePacket = r.ReadByte();
                int RLEPacketType = GetBits(rlePacket, 7, 1);
                int RLEPixelCount = GetBits(rlePacket, 0, 7) + 1;


                if (RLEPacketType == 0)
                {
                    //raw packet
                    for (int i = 0; i < RLEPixelCount; i++)
                    {
                        var color = (bitDepth == 32) ? r.ReadColor32RGBA().FlipRB() : r.ReadColor32RGB().FlipRB();
                        pulledColors[i + pulledColorCount] = color;
                    }

                }
                else
                {
                    //rle packet
                    var color = (bitDepth == 32) ? r.ReadColor32RGBA().FlipRB() : r.ReadColor32RGB().FlipRB();

                    for (int i = 0; i < RLEPixelCount; i++)
                    {
                        pulledColors[i + pulledColorCount] = color;
                    }
                }

                pulledColorCount += RLEPixelCount;
            }

            return pulledColors;
        }

        public static Texture2D Load(string fileName)
        {
            using (var imageFile = File.OpenRead(fileName))
            {
                return Load(imageFile);
            }
        }

        public static Texture2D Load(byte[] bytes)
        {
            using (var ms = new MemoryStream(bytes))
            {
                return Load(ms);
            }
        }

        public static Texture2D Load(Stream TGAStream)
        {

            using (BinaryReader r = new BinaryReader(TGAStream))
            {
                // Skip some header info we don't care about.
                r.BaseStream.Seek(2, SeekOrigin.Begin);

                byte imageType = r.ReadByte();
                if (imageType != 10 && imageType != 2)
                {
                    Debug.LogError($"Unsupported targa image type. ({imageType})");
                    return null;
                }

                //Skip right to some more data we need
                r.BaseStream.Seek(12, SeekOrigin.Begin);

                short width = r.ReadInt16();
                short height = r.ReadInt16();
                int bitDepth = r.ReadByte();

                if (bitDepth < 24)
                    throw new Exception("Tried to load TGA with unsupported bit depth");

                // Skip a byte of header information we don't care about.
                r.BaseStream.Seek(1, SeekOrigin.Current);

                Texture2D tex = new Texture2D(width, height, (bitDepth == 24) ? TextureFormat.RGB24 :  TextureFormat.ARGB32, true);
                if (imageType == 2)
                {
                    tex.SetPixels32(LoadRawTGAData(r, bitDepth, width, height));
                }
                else
                {
                    tex.SetPixels32(LoadRLETGAData(r, bitDepth, width, height));
                }

                tex.Apply();
                return tex;

            }
        }
    }
}
