using System;
using System.IO;
using System.Runtime.InteropServices;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;
using static UnityEditor.PlayerSettings;

namespace Microsoft.MixedReality.QR
{
    [InitializeOnLoad]
    internal class Init : IPreprocessBuildWithReport
    {
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        static extern int AddDllDirectory(string lpPathName);

        [DllImport("kernel32.dll", SetLastError = true)]
        static extern IntPtr LoadLibraryExW([MarshalAs(UnmanagedType.LPWStr)] string fileName, IntPtr fileHandle, uint flags);

        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        static extern bool FreeLibrary(IntPtr moduleHandle);

        const uint LOAD_LIBRARY_SEARCH_DEFAULT_DIRS = 0x00001000;

        public int callbackOrder => 0;

        static Init()
        {
            IntPtr modulePtr = LoadLibraryExW($"{typeof(Init).Namespace}.dll", IntPtr.Zero, LOAD_LIBRARY_SEARCH_DEFAULT_DIRS);
            if (modulePtr != IntPtr.Zero)
            {
                // DLL search paths already configured in this process; nothing more to do.
                FreeLibrary(modulePtr);
                return;
            }

            // Find the path to this script
            string assetName = $"{typeof(Init).FullName}.cs";
            var assets = AssetDatabase.FindAssets(Path.GetFileNameWithoutExtension(assetName));
            if (assets.Length != 1)
            {
                Debug.LogError($"Failed to find single asset for {assetName}; found {assets.Length} instead!");
                return;
            }

            char[] delims = { '/', '\\' };
            var assetDirectoryPath = Application.dataPath;
            var lastDelim = assetDirectoryPath.TrimEnd(delims).LastIndexOfAny(delims); // trim off Assets folder since it's also included in asset path
            var dllDirectory = Path.Combine(assetDirectoryPath.Substring(0, lastDelim), Path.GetDirectoryName(AssetDatabase.GUIDToAssetPath(assets[0]))).Replace('/', '\\');
            dllDirectory = Path.Combine(dllDirectory.Substring(0, dllDirectory.LastIndexOf("Editor")), @"x64");
            if (AddDllDirectory(dllDirectory) == 0)
            {
                Debug.LogError($"Failed to set DLL directory {dllDirectory}: Win32 error {Marshal.GetLastWin32Error()}");
                return;
            }

            Debug.Log(string.Format("Added DLL directory {0} to the user search path.", dllDirectory));
        }

        public void OnPreprocessBuild(BuildReport report)
        {
            if (report.summary.platform == BuildTarget.WSAPlayer && !WSA.GetCapability(WSACapability.WebCam))
            {
                Debug.LogError("The webcam capability is missing from Universal Windows Platform player settings. You must add this capability and additionally call QRCodeWatcher.RequestAccessAsync() for QR functionality to work.");
            }
        }
    }
}