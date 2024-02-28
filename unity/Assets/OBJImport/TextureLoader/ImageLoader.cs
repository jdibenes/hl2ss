/*
 * Created by Dummiesman 2013-2019
 * Thanks to mikezila for improving the initial TGA loading code
*/

using System;
using UnityEngine;
using System.Collections;
using System.IO;
using B83.Image.BMP;

namespace Dummiesman
{
    public class ImageLoader
    {
        /// <summary>
        /// Converts a DirectX normal map to Unitys expected format
        /// </summary>
        /// <param name="tex">Texture to convert</param>
        public static void SetNormalMap(ref Texture2D tex)
        {
            Color[] pixels = tex.GetPixels();
            for (int i = 0; i < pixels.Length; i++)
            {
                Color temp = pixels[i];
                temp.r = pixels[i].g;
                temp.a = pixels[i].r;
                pixels[i] = temp;
            }
            tex.SetPixels(pixels);
            tex.Apply(true);
        }

        public enum TextureFormat
        {
            DDS,
            TGA,
            BMP,
            PNG,
            JPG,
            CRN
        }


        /// <summary>
        /// Loads a texture from a stream
        /// </summary>
        /// <param name="stream">The stream</param>
        /// <param name="format">The format **NOT UNITYENGINE.TEXTUREFORMAT**</param>
        /// <returns></returns>
        public static Texture2D LoadTexture(Stream stream, TextureFormat format)
        {
            if (format == TextureFormat.BMP)
            {
                return new BMPLoader().LoadBMP(stream).ToTexture2D();
            }
            else if (format == TextureFormat.DDS)
            {
                return DDSLoader.Load(stream);
            }
            else if (format == TextureFormat.JPG || format == TextureFormat.PNG)
            {
                byte[] buffer = new byte[stream.Length];
                stream.Read(buffer, 0, (int)stream.Length);

                Texture2D texture = new Texture2D(1, 1);
                texture.LoadImage(buffer);
                return texture;
            }
            else if (format == TextureFormat.TGA)
            {
                return TGALoader.Load(stream);
            }
            else
            {
                return null;
            }
        }

      
        /// <summary>
        /// Loads a texture from a file
        /// </summary>
        /// <param name="fn"></param>
        /// <param name="normalMap"></param>
        /// <returns></returns>
        public static Texture2D LoadTexture(string fn)
        {
            if (!File.Exists(fn))
                return null;

            var textureBytes = File.ReadAllBytes(fn);
            string ext = Path.GetExtension(fn).ToLower();
            string name = Path.GetFileName(fn);
            Texture2D returnTex = null;

            switch (ext)
            {
                case ".png":
                case ".jpg":
                case ".jpeg":
                    returnTex = new Texture2D(1, 1);
                    returnTex.LoadImage(textureBytes);
                    break;
                case ".dds":
                    returnTex = DDSLoader.Load(textureBytes);
                    break;
                case ".tga":
                    returnTex = TGALoader.Load(textureBytes);
                    break;
                case ".bmp":
                    returnTex = new BMPLoader().LoadBMP(textureBytes).ToTexture2D();
                    break;
                case ".crn":
                    byte[] crnBytes = textureBytes;
                    ushort crnWidth = System.BitConverter.ToUInt16(new byte[2] { crnBytes[13], crnBytes[12] }, 0);
                    ushort crnHeight = System.BitConverter.ToUInt16(new byte[2] { crnBytes[15], crnBytes[14] }, 0);
                    byte crnFormatByte = crnBytes[18];

                    var crnTextureFormat = UnityEngine.TextureFormat.RGB24;
                    if (crnFormatByte == 0)
                    {
                        crnTextureFormat = UnityEngine.TextureFormat.DXT1Crunched;
                    }else if(crnFormatByte == 2)
                    {
                        crnTextureFormat = UnityEngine.TextureFormat.DXT5Crunched;
                    }
                    else if(crnFormatByte == 12)
                    {
                        crnTextureFormat = UnityEngine.TextureFormat.ETC2_RGBA8Crunched;
                    }
                    else
                    {
                        Debug.LogError("Could not load crunched texture " + name + " because its format is not supported (" + crnFormatByte + "): " + fn);
                        break;
                    }

                    returnTex = new Texture2D(crnWidth, crnHeight, crnTextureFormat, true);
                    returnTex.LoadRawTextureData(crnBytes);
                    returnTex.Apply(true);

                    break;
                default:
                    Debug.LogError("Could not load texture " + name + " because its format is not supported : " + fn);
                    break;
            }
            
            if (returnTex != null)
            {
                returnTex = ImageLoaderHelper.VerifyFormat(returnTex);
                returnTex.name = Path.GetFileNameWithoutExtension(fn);
            }                                           

            return returnTex;
        }

    }
}