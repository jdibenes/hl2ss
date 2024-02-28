using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

namespace Dummiesman
{
    public static class DDSLoader
    {
        public static Texture2D Load(Stream ddsStream)
        {
            byte[] buffer = new byte[ddsStream.Length];
            ddsStream.Read(buffer, 0, (int)ddsStream.Length);
            return Load(buffer);
        }

        public static Texture2D Load(string ddsPath)
        {
           return Load(File.ReadAllBytes(ddsPath));
        }

        public static Texture2D Load(byte[] ddsBytes)
        {
            try
            {

                //do size check
                byte ddsSizeCheck = ddsBytes[4];
                if (ddsSizeCheck != 124)
                    throw new System.Exception("Invalid DDS header. Structure length is incrrrect."); //this header byte should be 124 for DDS image files

                //verify we have a readable tex
                byte DXTType = ddsBytes[87];
                if (DXTType != 49 && DXTType != 53)
                    throw new System.Exception("Cannot load DDS due to an unsupported pixel format. Needs to be DXT1 or DXT5.");

                int height = ddsBytes[13] * 256 + ddsBytes[12];
                int width = ddsBytes[17] * 256 + ddsBytes[16];
                bool mipmaps = ddsBytes[28] > 0;
                TextureFormat textureFormat = DXTType == 49 ? TextureFormat.DXT1 : TextureFormat.DXT5;

                int DDS_HEADER_SIZE = 128;
                byte[] dxtBytes = new byte[ddsBytes.Length - DDS_HEADER_SIZE];
                Buffer.BlockCopy(ddsBytes, DDS_HEADER_SIZE, dxtBytes, 0, ddsBytes.Length - DDS_HEADER_SIZE);

                Texture2D texture = new Texture2D(width, height, textureFormat, mipmaps);
                texture.LoadRawTextureData(dxtBytes);
                texture.Apply();

                return texture;
            }
            catch (System.Exception ex)
            {
                throw new Exception("An error occured while loading DirectDraw Surface: " + ex.Message);
            }
        }
    }
}
