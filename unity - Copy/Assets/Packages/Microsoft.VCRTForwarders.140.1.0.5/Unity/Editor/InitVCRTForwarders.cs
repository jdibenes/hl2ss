using System;
using System.IO;
using System.Runtime.InteropServices;
using UnityEditor;
using UnityEngine;

[InitializeOnLoad]
public class InitVCRTForwarders : MonoBehaviour
{
    [DllImport("kernel32.dll", SetLastError = true)]
    static extern IntPtr LoadLibraryExW([MarshalAs(UnmanagedType.LPWStr)] string fileName, IntPtr fileHandle, uint flags);

    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern bool FreeLibrary(IntPtr moduleHandle);

    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    static extern int AddDllDirectory(string lpPathName);

    const uint LOAD_LIBRARY_SEARCH_USER_DIRS = 0x00000400;

    const string moduleName = "vcruntime140_app.dll";

    static InitVCRTForwarders()
    {
        IntPtr modulePtr = LoadLibraryExW(moduleName, IntPtr.Zero, LOAD_LIBRARY_SEARCH_USER_DIRS);
        if (modulePtr != IntPtr.Zero)
        {
            // DLL search paths already configured in this process; nothing more to do.
            FreeLibrary(modulePtr);
            return;
        }

        // Find a representative VCRTForwarders binary - there should be only one.
        var assets = AssetDatabase.FindAssets(Path.GetFileNameWithoutExtension(moduleName));
        if (assets.Length != 1)
        {
            Debug.LogError(string.Format("Failed to find single asset for {0}; found {1} instead!", moduleName, assets.Length));
            return;
        }

        char[] delims = { '/', '\\' };
        var assetDirectoryPath = Application.dataPath;
        var lastDelim = assetDirectoryPath.TrimEnd(delims).LastIndexOfAny(delims); // trim off Assets folder since it's also included in asset path
        var dllDirectory = Path.Combine(assetDirectoryPath.Substring(0, lastDelim), Path.GetDirectoryName(AssetDatabase.GUIDToAssetPath(assets[0]))).Replace('/', '\\');
        if (AddDllDirectory(dllDirectory) == 0)
        {
            Debug.LogError(string.Format("Failed to set DLL directory {0}: Win32 error {1}", dllDirectory, Marshal.GetLastWin32Error()));
            return;
        }

        Debug.Log(string.Format("Added DLL directory {0} to the user search path.", dllDirectory));
    }
}
