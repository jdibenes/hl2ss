
using UnityEngine;

public class Hololens2SensorStreaming : MonoBehaviour
{
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
        hl2ss.UpdateCoordinateSystem();
        hl2ss.Initialize(enableRM, enablePV, enableMC, enableSI, enableRC, enableSM, enableSU, enableVI, enableMQ, enableEET);
    }
}
