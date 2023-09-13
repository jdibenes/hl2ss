//******************************************************************************
// This class demonstrates how to receive and process messages from a client.
// For a sample client implementation see client_umq.py in the viewer directory.
//******************************************************************************
using AOT;
using System;
using System.Text;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;


namespace tcn
{

    public class EET_Sample
    {

        //[StructLayout(LayoutKind.Explicit)]
        //[FieldOffset(0)]
        [StructLayout(LayoutKind.Sequential)]
        public struct Vector3NativeType // z_owned_sample_t
        {
            internal float x;
            internal float y;
            internal float z;
        }

        //[StructLayout(LayoutKind.Explicit)]
        //[FieldOffset(0)]
        [StructLayout(LayoutKind.Sequential)]
        public struct QuaternionNativeType // z_owned_sample_t
        {
            internal float x;
            internal float y;
            internal float z;
            internal float w;
        }

        //[StructLayout(LayoutKind.Explicit)]
        //[FieldOffset(0)]
        [StructLayout(LayoutKind.Sequential)]
        public struct NativeType // z_owned_sample_t
        {
            internal UInt64 timestamp;
            internal Vector3NativeType position;
            internal QuaternionNativeType orientation;
            internal Vector3NativeType center_origin;
            internal Vector3NativeType center_direction;
            internal Vector3NativeType left_origin;
            internal Vector3NativeType left_direction;
            internal Vector3NativeType right_origin;
            internal Vector3NativeType right_direction;
            internal float left_openness;
            internal float right_openness;
            internal float vergence_dist;
            internal UInt32 valid;
        }

        public UInt64 Timestamp { get; } 
        public Vector3 Position { get; }
        public Quaternion Orientation { get; }
        public Vector3 CenterOrigin { get; }
        public Vector3 CenterDirection { get; }
        public Vector3 LeftOrigin { get; }
        public Vector3 LeftDirection { get; }
        public Vector3 RightOrigin { get; }
        public Vector3 RightDirection { get; }
        public float LeftOpenness { get; }
        public float RightOpenness { get; }
        public float VergenceDist { get; }
        public UInt32 Valid { get; }



        internal EET_Sample(NativeType sample)
        {
            Timestamp = sample.timestamp;
            Position.Set(sample.position.x, sample.position.y, sample.position.z);
            Orientation.Set(sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w);
            CenterOrigin.Set(sample.center_origin.x, sample.center_origin.y, sample.center_origin.z);
            CenterDirection.Set(sample.center_direction.x, sample.center_direction.y, sample.center_direction.z);
            LeftOrigin.Set(sample.left_origin.x, sample.left_origin.y, sample.left_origin.z);
            LeftDirection.Set(sample.left_direction.x, sample.left_direction.y, sample.left_direction.z);
            RightOrigin.Set(sample.right_origin.x, sample.right_origin.y, sample.right_origin.z);
            RightDirection.Set(sample.right_direction.x, sample.right_direction.y, sample.right_direction.z);
            LeftOpenness = sample.left_openness;
            RightOpenness = sample.right_openness;
            VergenceDist = sample.vergence_dist;
            Valid = sample.valid;
        }

        public string ValueToString()
        {
            string result = Timestamp.ToString(); //System.Text.Encoding.UTF8.GetString(Timestamp, 0, Timestamp.Length);
            return result;
        }
    }

    public class EETSubscriber : MonoBehaviour
    {
        [Tooltip("Enter the Subscriber Name.")]
        public string subscriber_name;

        [Tooltip("Enter the Zenoh Topic to Subscribe to.")]
        public string topic_name;


        private UnityAction<bool> haveSessionEvent;

        void OnEnable()
        {
            if (haveSessionEvent == null)
                haveSessionEvent = new UnityAction<bool>(SessionEventCallback);

            // Register to the event
            //
            tcn.hl2comm.RegisterForSessionEvent(haveSessionEvent);
        }

        void OnDisable()
        {
            // Unregister from the event when this object is disabled or destroyed
            //
            tcn.hl2comm.UnregisterSessionEvent(haveSessionEvent);
            StopEETOnUI();
        }

        private void SessionEventCallback(bool status)
        {
            Debug.Log("Subscribing to topic: " + this.topic_name);
            StartEETOnUI(OnInternalMessageCallback, this.topic_name);
        }

        // Start is called before the first frame update
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }


        [DllImport(hl2comm.DllName, CallingConvention = CallingConvention.Cdecl)]
        static extern bool StartEETOnUI(EETSubscriptionCallback cb, string topic);

        [DllImport(hl2comm.DllName, CallingConvention = CallingConvention.Cdecl)]
        static extern bool StopEETOnUI();

        //Create string param callback delegate
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]  
        internal delegate bool EETSubscriptionCallback(ref tcn.EET_Sample.NativeType eet_sample);
        [MonoPInvokeCallback(typeof(EETSubscriptionCallback))]
        internal bool OnInternalMessageCallback(ref tcn.EET_Sample.NativeType eet_sample)
        {
            tcn.EET_Sample s = new tcn.EET_Sample(eet_sample);  
            this.HandleMessage(s);
            return true;
        }

        void HandleMessage(tcn.EET_Sample eet_sample)
        {
            UnityEngine.Debug.Log("received some message: " + eet_sample.ValueToString());
        }

    }

}
