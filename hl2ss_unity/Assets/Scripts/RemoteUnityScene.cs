
using System;
using System.Text;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using Microsoft.MixedReality.Toolkit.Audio;

public class RemoteUnityScene : MonoBehaviour
{
    public GameObject m_tts;

    private Dictionary<int, GameObject> m_remote_objects;
    private bool m_loop;
    private bool m_mode;
    private int m_last_key;
    private hl2ss.NamedMutex m_mutex_pv;
    private hl2ss.NamedMutex m_mutex_ev;

    [Tooltip("Set to BasicMaterial to support semi-transparent primitives.")]
    public Material m_material;

    void Start()
    {
        m_remote_objects = new Dictionary<int, GameObject>();
        m_loop = false;
        m_mode = false;
        m_mutex_pv = hl2ss.NamedMutex.Create(hl2ss.MUTEX_NAME_PV);
        m_mutex_ev = hl2ss.NamedMutex.Create(hl2ss.MUTEX_NAME_EV);
    }

    void Update()
    {
        while (GetMessage() && m_loop);
    }

    bool GetMessage()
    {
        uint command;
        byte[] data;
        if (!hl2ss.PullMessage(out command, out data)) { return false; }
        hl2ss.PushResult(ProcessMessage(command, data));
        hl2ss.AcknowledgeMessage(command);
        return true;
    }

    uint ProcessMessage(uint command, byte[] data)
    {
        uint ret = 0;

        switch (command)
        {
        case 0x00000000: ret = MSG_CreatePrimitive(data); break;
        case 0x00000001: ret = MSG_SetActive(data); break;
        case 0x00000002: ret = MSG_SetWorldTransform(data); break;
        case 0x00000003: ret = MSG_SetLocalTransform(data); break;
        case 0x00000004: ret = MSG_SetColor(data); break;
        case 0x00000005: ret = MSG_SetTexture(data); break;
        case 0x00000006: ret = MSG_CreateText(data); break;
        case 0x00000007: ret = MSG_SetText(data); break;
        case 0x00000008: ret = MSG_Say(data); break; 

        case 0x00000010: ret = MSG_Remove(data); break;
        case 0x00000011: ret = MSG_RemoveAll(data); break;
        case 0x00000012: ret = MSG_BeginDisplayList(data); break;
        case 0x00000013: ret = MSG_EndDisplayList(data); break;
        case 0x00000014: ret = MSG_SetTargetMode(data); break;

        case 0xFFFFFF00: ret = MSG_DebugTryLockPV(data); break;
        case 0xFFFFFF01: ret = MSG_DebugUnlockPV(data); break;
        case 0xFFFFFF02: ret = MSG_DebugTryLockEV(data); break;
        case 0xFFFFFF03: ret = MSG_DebugUnlockEV(data); break;
        case 0xFFFFFFFE: ret = MSG_DebugMessage(data); break;

        case ~0U:        ret = MSG_Disconnect(data); break;
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
    uint MSG_Remove(byte[] data)
    {
        if (data.Length < 4) { return 0; }

        GameObject go;
        int key = GetKey(data);
        if (!m_remote_objects.TryGetValue(key, out go)) { return 0; }    
        
        m_remote_objects.Remove(key);
        Destroy(go);

        return 1;
    }

    // OK
    uint MSG_RemoveAll(byte[] data)
    {
        foreach (var go in m_remote_objects.Values) { Destroy(go); }
        m_remote_objects.Clear();
        return 1;
    }

    // OK
    uint MSG_BeginDisplayList(byte[] data)
    {
        m_loop = true;
        return 1;
    }

    // OK
    uint MSG_EndDisplayList(byte[] data)
    {
        m_loop = false;
        return 1;
    }

    // OK
    uint MSG_SetTargetMode(byte[] data)
    {
        if (data.Length < 4) { return 0; }
        m_mode = BitConverter.ToUInt32(data, 0) != 0;
        return 1;
    }

    // OK
    uint MSG_Disconnect(byte[] data)
    {
        m_loop = false;
        m_mode = false;
        m_last_key = 0;

        return ~0U;
    }

    // OK
    uint MSG_CreatePrimitive(byte[] data)
    {
        if (data.Length < 4) { return 0; }

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

        go.GetComponent<Renderer>().material = m_material;
        go.SetActive(false);

        return AddGameObject(go);
    }

    // OK
    uint MSG_SetActive(byte[] data)
    {
        if (data.Length < 8) { return 0; }
        
        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        go.SetActive(BitConverter.ToInt32(data, 4) != 0);

        return 1;
    }

    // OK
    uint MSG_SetWorldTransform(byte[] data)
    {
        if (data.Length < 44) { return 0; }

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
    uint MSG_SetLocalTransform(byte[] data)
    {
        if (data.Length < 44) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        Vector3 position;
        Quaternion rotation;
        Vector3 locscale;

        UnpackTransform(data, 4, out position, out rotation, out locscale);

        go.transform.parent = transform;

        go.transform.localPosition = position;
        go.transform.localRotation = rotation;
        go.transform.localScale    = locscale;

        return 1;
    }

    // OK
    uint MSG_SetColor(byte[] data)
    {
        if (data.Length < 20) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        go.GetComponent<Renderer>().material.color = new Color(BitConverter.ToSingle(data, 4), BitConverter.ToSingle(data, 8), BitConverter.ToSingle(data, 12), BitConverter.ToSingle(data, 16));

        return 1;
    }

    // OK
    uint MSG_SetTexture(byte[] data)
    {
        if (data.Length < 4) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }

        Texture2D tex;
        if (data.Length > 4)
        {
            tex = new Texture2D(2, 2);
            byte[] image = new byte[data.Length - 4];
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
    uint MSG_CreateText(byte[] data)
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
    uint MSG_SetText(byte[] data)
    {
        if (data.Length < 24) { return 0; }

        GameObject go;
        if (!m_remote_objects.TryGetValue(GetKey(data), out go)) { return 0; }
        TextMeshPro tmp = go.GetComponent<TextMeshPro>();
        if (tmp == null) { return 0; }

        tmp.fontSize = BitConverter.ToSingle(data, 4);
        tmp.color = new Color(BitConverter.ToSingle(data, 8), BitConverter.ToSingle(data, 12), BitConverter.ToSingle(data, 16), BitConverter.ToSingle(data, 20));

        string str;
        if (data.Length > 24)
        {
            byte[] str_bytes = new byte[data.Length - 24];
            Array.Copy(data, 24, str_bytes, 0, str_bytes.Length);
            try { str = Encoding.UTF8.GetString(str_bytes); } catch { return 0; }
        }
        else
        {
            str = "";
        }

        tmp.text = str;

        return 1;
    }

    // OK
    uint MSG_Say(byte[] data)
    {
        string str;
        try { str = Encoding.UTF8.GetString(data); } catch { return 0; }
        m_tts.GetComponent<TextToSpeech>().StartSpeaking(str);
        return 1;
    }

    // OK
    uint MSG_DebugTryLockPV(byte[] data)
    {
        return m_mutex_pv.Acquire(0) ? 1U : 0;
    }

    // OK
    uint MSG_DebugUnlockPV(byte[] data)
    {
        return m_mutex_pv.Release() ? 1U : 0;
    }

    // OK
    uint MSG_DebugTryLockEV(byte[] data)
    {
        return m_mutex_ev.Acquire(0) ? 1U : 0;
    }

    // OK
    uint MSG_DebugUnlockEV(byte[] data)
    {
        return m_mutex_ev.Release() ? 1U : 0;
    }

    // OK
    uint MSG_DebugMessage(byte[] data)
    {
        string str;
        try { str = Encoding.UTF8.GetString(data); } catch { return 0; }
        hl2ss.Print(str);
        return 1;
    }
}
