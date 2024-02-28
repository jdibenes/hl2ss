/*
 * Copyright (c) 2019 Dummiesman
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
*/

using Dummiesman;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class MTLLoader {
    public List<string> SearchPaths = new List<string>() { "%FileName%_Textures", string.Empty};

    private FileInfo _objFileInfo = null;

    /// <summary>
    /// The texture loading function. Overridable for stream loading purposes.
    /// </summary>
    /// <param name="path">The path supplied by the OBJ file, converted to OS path seperation</param>
    /// <param name="isNormalMap">Whether the loader is requesting we convert this into a normal map</param>
    /// <returns>Texture2D if found, or NULL if missing</returns>
    public virtual Texture2D TextureLoadFunction(string path, bool isNormalMap)
    {
        //find it
        foreach (var searchPath in SearchPaths)
        {
            //replace varaibles and combine path
            string processedPath = (_objFileInfo != null) ? searchPath.Replace("%FileName%", Path.GetFileNameWithoutExtension(_objFileInfo.Name)) 
                                                          : searchPath;
            string filePath = Path.Combine(processedPath, path);

            //return if eists
            if (File.Exists(filePath))
            {
                var tex = ImageLoader.LoadTexture(filePath);

                if(isNormalMap)
                    tex = ImageUtils.ConvertToNormalMap(tex);

                return tex;
            }
        }

        //not found
        return null;
    }

    private Texture2D TryLoadTexture(string texturePath, bool normalMap = false)
    {
        //swap directory seperator char
        texturePath = texturePath.Replace('\\', Path.DirectorySeparatorChar);
        texturePath = texturePath.Replace('/', Path.DirectorySeparatorChar);

        return TextureLoadFunction(texturePath, normalMap);
    }
    
    private int GetArgValueCount(string arg)
    {
        switch (arg)
        {
            case "-bm":
            case "-clamp":
            case "-blendu":
            case "-blendv":
            case "-imfchan":
            case "-texres":
                return 1;
            case "-mm":
                return 2;
            case "-o":
            case "-s":
            case "-t":
                return 3;
        }
        return -1;
    }

    private int GetTexNameIndex(string[] components)
    {
        for(int i=1; i < components.Length; i++)
        {
            var cmpSkip = GetArgValueCount(components[i]);
            if(cmpSkip < 0)
            {
                return i;
            }
            i += cmpSkip;
        }
        return -1;
    }

    private float GetArgValue(string[] components, string arg, float fallback = 1f)
    {
        string argLower = arg.ToLower();
        for(int i=1; i < components.Length - 1; i++)
        {
            var cmp = components[i].ToLower();
            if(argLower == cmp)
            {
                return OBJLoaderHelper.FastFloatParse(components[i+1]);
            }
        }
        return fallback;
    }

    private string GetTexPathFromMapStatement(string processedLine, string[] splitLine)
    {
        int texNameCmpIdx = GetTexNameIndex(splitLine);
        if(texNameCmpIdx < 0)
        {
            Debug.LogError($"texNameCmpIdx < 0 on line {processedLine}. Texture not loaded.");
            return null;
        }

        int texNameIdx = processedLine.IndexOf(splitLine[texNameCmpIdx]);
        string texturePath = processedLine.Substring(texNameIdx);

        return texturePath;
    }

    /// <summary>
    /// Loads a *.mtl file
    /// </summary>
    /// <param name="input">The input stream from the MTL file</param>
    /// <returns>Dictionary containing loaded materials</returns>
    public Dictionary<string, Material> Load(Stream input)
    {
        var inputReader = new StreamReader(input);
        var reader = new StringReader(inputReader.ReadToEnd());

        Dictionary<string, Material> mtlDict = new Dictionary<string, Material>();
        Material currentMaterial = null;

        for (string line = reader.ReadLine(); line != null; line = reader.ReadLine())
        {
            if (string.IsNullOrWhiteSpace(line))
                continue;

            string processedLine = line.Clean();
            string[] splitLine = processedLine.Split(' ');

            //blank or comment
            if (splitLine.Length < 2 || processedLine[0] == '#')
                continue;

            //newmtl
            if (splitLine[0] == "newmtl")
            {
                string materialName = processedLine.Substring(7);

                var newMtl = new Material(Shader.Find("Standard (Specular setup)")) { name = materialName };
                mtlDict[materialName] = newMtl;
                currentMaterial = newMtl;

                continue;
            }

            //anything past here requires a material instance
            if (currentMaterial == null)
                continue;

            //diffuse color
            if (splitLine[0] == "Kd" || splitLine[0] == "kd")
            {
                var currentColor = currentMaterial.GetColor("_Color");
                var kdColor = OBJLoaderHelper.ColorFromStrArray(splitLine);

                currentMaterial.SetColor("_Color", new Color(kdColor.r, kdColor.g, kdColor.b, currentColor.a));
                continue;
            }

            //diffuse map
            if (splitLine[0] == "map_Kd" || splitLine[0] == "map_kd")
            {
                string texturePath = GetTexPathFromMapStatement(processedLine, splitLine);
                if(texturePath == null)
                {
                    continue; //invalid args or sth
                }

                var KdTexture = TryLoadTexture(texturePath);
                currentMaterial.SetTexture("_MainTex", KdTexture);

                //set transparent mode if the texture has transparency
                if(KdTexture != null && (KdTexture.format == TextureFormat.DXT5 || KdTexture.format == TextureFormat.ARGB32))
                {
                    OBJLoaderHelper.EnableMaterialTransparency(currentMaterial);
                }

                //flip texture if this is a dds
                if(Path.GetExtension(texturePath).ToLower() == ".dds")
                {
                    currentMaterial.mainTextureScale = new Vector2(1f, -1f);
                }

                continue;
            }

            //bump map
            if (splitLine[0] == "map_Bump" || splitLine[0] == "map_bump")
            {
                string texturePath = GetTexPathFromMapStatement(processedLine, splitLine);
                if(texturePath == null)
                {
                    continue; //invalid args or sth
                }

                var bumpTexture = TryLoadTexture(texturePath, true);
                float bumpScale = GetArgValue(splitLine, "-bm", 1.0f);

                if (bumpTexture != null) {
                    currentMaterial.SetTexture("_BumpMap", bumpTexture);
                    currentMaterial.SetFloat("_BumpScale", bumpScale);
                    currentMaterial.EnableKeyword("_NORMALMAP");
                }

                continue;
            }

            //specular color
            if (splitLine[0] == "Ks" || splitLine[0] == "ks")
            {
                currentMaterial.SetColor("_SpecColor", OBJLoaderHelper.ColorFromStrArray(splitLine));
                continue;
            }

            //emission color
            if (splitLine[0] == "Ka" || splitLine[0] == "ka")
            {
                currentMaterial.SetColor("_EmissionColor", OBJLoaderHelper.ColorFromStrArray(splitLine, 0.05f));
                currentMaterial.EnableKeyword("_EMISSION");
                continue;
            }

            //emission map
            if (splitLine[0] == "map_Ka" || splitLine[0] == "map_ka")
            {
                string texturePath = GetTexPathFromMapStatement(processedLine, splitLine);
                if(texturePath == null)
                {
                    continue; //invalid args or sth
                }

                currentMaterial.SetTexture("_EmissionMap", TryLoadTexture(texturePath));
                continue;
            }

            //alpha
            if (splitLine[0] == "d" || splitLine[0] == "Tr")
            {
                float visibility = OBJLoaderHelper.FastFloatParse(splitLine[1]);
                
                //tr statement is just d inverted
                if(splitLine[0] == "Tr")
                    visibility = 1f - visibility;  

                if(visibility < (1f - Mathf.Epsilon))
                {
                    var currentColor = currentMaterial.GetColor("_Color");

                    currentColor.a = visibility;
                    currentMaterial.SetColor("_Color", currentColor);

                    OBJLoaderHelper.EnableMaterialTransparency(currentMaterial);
                }
                continue;
            }

            //glossiness
            if (splitLine[0] == "Ns" || splitLine[0] == "ns")
            {
                float Ns = OBJLoaderHelper.FastFloatParse(splitLine[1]);
                Ns = (Ns / 1000f);
                currentMaterial.SetFloat("_Glossiness", Ns);
            }
        }

        //return our dict
        return mtlDict;
    }

    /// <summary>
    /// Loads a *.mtl file
    /// </summary>
    /// <param name="path">The path to the MTL file</param>
    /// <returns>Dictionary containing loaded materials</returns>
	public Dictionary<string, Material> Load(string path)
    {
        _objFileInfo = new FileInfo(path); //get file info
        SearchPaths.Add(_objFileInfo.Directory.FullName); //add root path to search dir

        using (var fs = new FileStream(path, FileMode.Open))
        {
            return Load(fs); //actually load
        }
        
    }
}
