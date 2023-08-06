using AOT;
using System;
using System.Runtime.InteropServices;
using UnityEngine;


namespace tcn
{
    public class DebugLogHelper : MonoBehaviour
    {

        // Use this for initialization
        void OnEnable()
        {
            RegisterLoggingCallback(OnLogCallback);
        }

        [DllImport(hl2comm.DllName, CallingConvention = CallingConvention.Cdecl)]
        static extern void RegisterLoggingCallback(debugCallback cb);

        //Create string param callback delegate
        delegate void debugCallback(IntPtr request, int color, int size);
        enum Color { red, green, blue, black, white, yellow, orange };
        [MonoPInvokeCallback(typeof(debugCallback))]
        static void OnLogCallback(IntPtr request, int color, int size)
        {
            //Ptr to string
            string debug_string = Marshal.PtrToStringAnsi(request, size);

            //Add Specified Color
            debug_string =
                String.Format("{0}{1}{2}{3}{4}",
                "<color=",
                ((Color)color).ToString(),
                ">",
                debug_string,
                "</color>"
                );

            UnityEngine.Debug.Log(debug_string);
        }
    }

}
