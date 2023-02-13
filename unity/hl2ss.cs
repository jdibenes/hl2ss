
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using TMPro;

public class hl2ss : MonoBehaviour
{
#if WINDOWS_UWP

    [DllImport("hl2ss")]
    private static extern void InitializeStreams(uint enable);
    [DllImport("hl2ss")]
    private static extern void DebugMessage(string str);
    [DllImport("hl2ss")]
    private static extern void MQ_SO_Push(uint id);
    [DllImport("hl2ss")]
    private static extern void MQ_SI_Pop(out uint command, byte[] data);
    [DllImport("hl2ss")]
    private static extern uint MQ_SI_Peek();
    [DllImport("hl2ss")]
    private static extern void MQ_Restart();
    [DllImport("hl2ss")]
    private static extern void SI_Update();
    [DllImport("hl2ss")]
    private static extern void GetLocalIPv4Address(byte[] data, int size);
#else
    private void InitializeStreams(uint enable)
    {
    }

    private void DebugMessage(string str)
    {
        Debug.Log(str);
    }

    private void MQ_SO_Push(uint id)
    {
    }

    private void MQ_SI_Pop(out uint command, byte[] data)
    {
        command = 0;
    }

    private uint MQ_SI_Peek()
    {
        return ~0U;
    }

    private void MQ_Restart()
    {
    }

    private void SI_Update()
    {
    }

    private void GetLocalIPv4Address(byte[] data, int size)
    {

    }
#endif

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

    [Tooltip("Set to BasicMaterial to support semi-transparent primitives.")]
    public Material m_material;

    private Dictionary<int, GameObject> m_remote_objects;
    private byte[] m_buffer;
    private bool m_loop;
    private bool m_mode;
    private int m_last_key;

    // Start is called before the first frame update
    void Start()
    {
        m_remote_objects = new Dictionary<int, GameObject>();
        m_buffer = new byte[8 * 1024 * 1024];
        m_loop = false;
        m_mode = false;

        if (!skipInitialization) { InitializeStreams((enableRM ? 1U : 0U) | (enablePV ? 2U : 0U) | (enableMC ? 4U : 0U) | (enableRC ? 16U : 0U) | (enableSM ? 32U : 0U) | (enableSU ? 64U : 0U)); }

        byte[] ipaddress = new byte[16 * 2];
        GetLocalIPv4Address(ipaddress, ipaddress.Length);
        string ip = System.Text.Encoding.Unicode.GetString(ipaddress);
        DebugMessage(string.Format("UNITY: Local IP Address is: {0}", ip));
    }

    // Update is called once per frame
    void Update()
    {
        if (enableSI) { SI_Update(); }
        do
        {
            uint size = MQ_SI_Peek();
            if (size == ~0U) { return; }
            uint command;
            if (size > m_buffer.Length) { m_buffer = new byte[size]; }
            MQ_SI_Pop(out command, m_buffer);
            MQ_SO_Push(ProcessMessage(command, size, m_buffer));
            if (command == ~0U) { MQ_Restart(); }
        }
        while (m_loop);
    }

    uint ProcessMessage(uint command, uint size, byte[] data)
    {
        uint ret = 0;

        switch (command)
        {
        case   0: ret = MSG_CreatePrimitive(  size, data); break;
        case   1: ret = MSG_SetActive(        size, data); break;
        case   2: ret = MSG_SetWorldTransform(size, data); break;
        case   3: ret = MSG_SetLocalTransform(size, data); break;
        case   4: ret = MSG_SetColor(         size, data); break;
        case   5: ret = MSG_SetTexture(       size, data); break;
        case   6: ret = MSG_CreateText(       size, data); break;
        case   7: ret = MSG_SetText(          size, data); break;

        case  16: ret = MSG_Remove(           size, data); break;
        case  17: ret = MSG_RemoveAll(        size, data); break;
        case  18: ret = MSG_BeginDisplayList( size, data); break;
        case  19: ret = MSG_EndDisplayList(   size, data); break;
        case  20: ret = MSG_SetTargetMode(    size, data); break;
        case ~0U: ret = MSG_Disconnect(       size, data); break;
        }

        return ret;
    }

    // OK
    uint AddGameObject(GameObject go)
    {
        int key = go.GetInstanceID();
        m_remote_objects.Add(key, go);
        m_last_key = key;

        return (uint)key;
    }

    // OK
    int GetKey(byte[] data)
    {
        return m_mode ? m_last_key : BitConverter.ToInt32(data, 0);
    }

    // OK
    void UnpackTransform(byte[] data, int offset, out Vector3 position, out Quaternion rotation, out Vector3 locscale)
    {
        float[] f = new float[10];
        for (int i = 0; i < f.Length; ++i) { f[i] = BitConverter.ToSingle(data, offset + (i * 4)); }

        position = new Vector3(f[0], f[1], f[2]);
        rotation = new Quaternion(f[3], f[4], f[5], f[6]);
        locscale = new Vector3(f[7], f[8], f[9]);
    }

    // OK
    uint MSG_Remove(uint size, byte[] data)
    {
        if (size < 4) { return 0; }

        GameObject go;
        int key = GetKey(data);
        if (!m_remote_objects.TryGetValue(key, out go)) { return 0; }    
        
        m_remote_objects.Remove(key);
        Destroy(go);

        return 1;
    }

    // OK
    uint MSG_RemoveAll(uint size, byte[] data)
    {
        foreach (var go in m_remote_objects.Values) { Destroy(go); }
        m_remote_objects.Clear();

        return 1;
    }

    // OK
    uint MSG_BeginDisplayList(uint size, byte[] data)
    {
        m_loop = true;

        return 1;
    }

    // OK
    uint MSG_EndDisplayList(uint size, byte[] data)
    {
        m_loop = false;

        return 1;
    }

    // OK
    uint MSG_SetTargetMode(uint size, byte[] data)
    {
        if (size < 4) { return 0; }

        m_mode = BitConverter.ToUInt32(data, 0) != 0;

        return 1;
    }

    // OK
    uint MSG_Disconnect(uint size, byte[] data)
    {
        m_loop = false;
        m_mode = false;
        m_last_key = 0;

        return ~0U;
    }

    // OK
    uint MSG_CreatePrimitive(uint size, byte[] data)
    {
        if (size < 4) { return 0; }

        PrimitiveType t;
        switch (BitConverter.ToUInt32(data, 0))
        {
        case 0:  t = PrimitiveType.Sphere;   break;
        case 1:  t = PrimitiveType.Capsule;  break;
        case 2:  t = PrimitiveType.Cylinder; break;
        case 3:  t = PrimitiveType.Cube;     break;
        case 4:  t = PrimitiveType.Plane;    break;
        default: t = PrimitiveType.Quad;     break;
        }

        GameObject go = GameObject.CreatePrimitive(t);

        go.GetComponent<Renderer>().material = m_material; // TODO new Material instead?
        go.SetActive(false);

        return AddGameObject(go);
    }

    // OK
    uint MSG_SetActive(uint size, byte[] data)
    {
        if (size < 8) { return 0; }
        
        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        go.SetActive(BitConverter.ToInt32(data, 4) != 0);

        return 1;
    }

    // OK
    uint MSG_SetWorldTransform(uint size, byte[] data)
    {
        if (size < 44) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        Vector3 position;
        Quaternion rotation;
        Vector3 locscale;

        UnpackTransform(data, 4, out position, out rotation, out locscale);

        go.transform.parent = null;
        go.transform.SetPositionAndRotation(position, rotation);
        go.transform.localScale = locscale;

        return 1;
    }

    // OK
    uint MSG_SetLocalTransform(uint size, byte[] data)
    {
        if (size < 44) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        go.transform.parent = transform;

        Vector3 position;
        Quaternion rotation;
        Vector3 locscale;

        UnpackTransform(data, 4, out position, out rotation, out locscale);

        Camera cam = gameObject.GetComponent<Camera>();

        var half = locscale / 2;

        var cc = transform.InverseTransformPoint(cam.ScreenToWorldPoint(position));
        var ul = transform.InverseTransformPoint(cam.ScreenToWorldPoint(new Vector3(cc.x - half.x, cc.y - half.y, cc.z)));
        var ur = transform.InverseTransformPoint(cam.ScreenToWorldPoint(new Vector3(cc.x + half.x, cc.y - half.y, cc.z)));
        var bl = transform.InverseTransformPoint(cam.ScreenToWorldPoint(new Vector3(cc.x - half.x, cc.y + half.y, cc.z)));

        var dx = ul - ur;
        var dy = ul - bl;

        go.transform.localPosition = cc;
        go.transform.localRotation = rotation;
        go.transform.localScale = new Vector3(Mathf.Abs(dx.x), Mathf.Abs(dy.y), locscale.z);

        return 1;
    }

    // OK
    uint MSG_SetColor(uint size, byte[] data)
    {
        if (size < 20) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        go.GetComponent<Renderer>().material.color = new Color(BitConverter.ToSingle(data, 4), BitConverter.ToSingle(data, 8), BitConverter.ToSingle(data, 12), BitConverter.ToSingle(data, 16));

        return 1;
    }

    // OK
    uint MSG_SetTexture(uint size, byte[] data)
    {
        if (size < 4) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        Texture2D tex;
        if (size > 4)
        {
            tex = new Texture2D(2, 2);
            byte[] image = new byte[size - 4];
            Array.Copy(data, 4, image, 0, image.Length);
            tex.LoadImage(image);
        }
        else
        {
            tex = null;
        }

        go.GetComponent<Renderer>().material.mainTexture = tex;

        return 1;
    }

    // OK
    uint MSG_CreateText(uint size, byte[] data)
    {
        GameObject go = new GameObject();
        TextMeshPro tmp = go.AddComponent<TextMeshPro>();

        go.SetActive(false);

        tmp.enableWordWrapping = false;
        tmp.autoSizeTextContainer = true;
        tmp.alignment = TextAlignmentOptions.Center;
        tmp.verticalAlignment = VerticalAlignmentOptions.Middle;
        tmp.text = "";

        return AddGameObject(go);
    }

    // OK
    uint MSG_SetText(uint size, byte[] data)
    {
        if (size < 24) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }
        TextMeshPro tmp = go.GetComponent<TextMeshPro>();
        if (tmp == null) { return 0; }

        tmp.fontSize = BitConverter.ToSingle(data, 4);
        tmp.color = new Color(BitConverter.ToSingle(data, 8), BitConverter.ToSingle(data, 12), BitConverter.ToSingle(data, 16), BitConverter.ToSingle(data, 20));

        string str;
        if (size > 24)
        {
            byte[] str_bytes = new byte[size - 24];
            Array.Copy(data, 24, str_bytes, 0, str_bytes.Length);
            try { str = System.Text.Encoding.UTF8.GetString(str_bytes); } catch { return 0; }
        }
        else
        {
            str = "";
        }

        tmp.text = str;

        return 1;
    }
}
