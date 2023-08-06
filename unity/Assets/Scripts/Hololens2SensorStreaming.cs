
using UnityEngine;

namespace tcn
{
    public class Hololens2SensorStreaming : MonoBehaviour
    {
        [Tooltip("Enter the Zenoh Topic Prefix.")]
        public string topicPrefix = "tcn/loc/hl2/device00";

        [Tooltip("Optional Zenoh Configuration as json string.")]
        public string zenohConfig = "";

        [Tooltip("Enable Research Mode streams.")]
        public bool enableRM = true;

        [Tooltip("Enable Front Camera stream.")]
        public bool enablePV = true;

        [Tooltip("Enable Microphone stream.")]
        public bool enableMC = true;

        [Tooltip("Enable Spatial Input stream.")]
        public bool enableSI = true;

        [Tooltip("Enable Remote Configuration interface.")]
        public bool enableRC = true;

        [Tooltip("Enable Spatial Mapping interface.")]
        public bool enableSM = true;

        [Tooltip("Enable Scene Understanding interface.")]
        public bool enableSU = true;

        [Tooltip("Enable Voice Input interface.")]
        public bool enableVI = true;

        [Tooltip("Enable Message Queue interface.")]
        public bool enableMQ = true;

        [Tooltip("Enable Extended Eye Tracking Interface.")]
        public bool enableEET = true;

        void Start()
        {
            hl2comm.UpdateCoordinateSystem();
            hl2comm.Initialize(topicPrefix, zenohConfig, enableRM, enablePV, enableMC, enableSI, enableRC, enableSM, enableSU, enableVI, enableMQ, enableEET);
        }
    }

}
