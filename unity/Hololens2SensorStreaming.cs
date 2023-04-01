
using UnityEngine;

public class Hololens2SensorStreaming : MonoBehaviour
{
    [Tooltip("Must be enabled if InitializeStreams is called from the cpp code and disabled otherwise.")]
    public bool skipInitialization = false;

    [Tooltip("Enable Research Mode sensors streams. Has no effect if InitializeStreams is called from the cpp code.")]
    public bool enableRM = true;

    [Tooltip("Enable Front Camera stream. Has no effect if InitializeStreams is called from the cpp code.")]
    public bool enablePV = true;

    [Tooltip("Enable Microphone stream. Has no effect if InitializeStreams is called from the cpp code.")]
    public bool enableMC = true;

    [Tooltip("Enable Spatial Input stream. Allowed only if InitializeStreams is called from the cpp code and must be disabled otherwise.")]
    public bool enableSI = false;

    [Tooltip("Enable Remote Configuration interface. Has no effect if InitializeStreams is called from the cpp code.")]
    public bool enableRC = true;

    [Tooltip("Enable Spatial Mapping interface. Has no effect if InitializeStreams is called from the cpp code.")]
    public bool enableSM = true;

    [Tooltip("Enable Scene Understanding interface. Has no effect if InitializeStreams is called from the cpp code.")]
    public bool enableSU = true;

    [Tooltip("Enable Voice Input interface. Has no effect if InitializeStreams is called from the cpp code.")]
    public bool enableVI = false;

    void Start()
    {
        hl2ss.UpdateCoordinateSystem();
        if (!skipInitialization) { hl2ss.Initialize(enableRM, enablePV, enableMC, enableRC, enableSM, enableSU, enableVI); }
    }
}
