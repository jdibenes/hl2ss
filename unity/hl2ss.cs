
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

public class hl2ss : MonoBehaviour
{
#if WINDOWS_UWP

    [DllImport("hl2ss")]
    private static extern void InitializeStreams();
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
#else
    private void InitializeStreams()
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
        command = 18;
    }

    private uint MQ_SI_Peek()
    {
        return ~0U;
    }

    private void MQ_Restart()
    {

    }
#endif

    public Material m_material;
    private Dictionary<int, GameObject> m_remote_objects;
    //byte[] m_buffer;


    // Start is called before the first frame update
    void Start()
    {
        m_remote_objects = new Dictionary<int, GameObject>();
        //m_buffer = new byte[4 * 1024 * 1024];
        InitializeStreams();
    }

    // Update is called once per frame
    void Update()
    {
        uint next_size = MQ_SI_Peek();
        if (next_size == ~0U) { return; }
        uint command;
        byte[] data = (next_size > 0) ? new byte[next_size] : null;
        MQ_SI_Pop(out command, data);
        uint ret = ProcessMessage(command, data);
        MQ_SO_Push(ret);
        if (command == ~0U) { MQ_Restart(); }
    }

    uint CreatePrimitive(byte[] data)
    {
        if (data == null || data.Length != 4) { return 0; }

        uint type = BitConverter.ToUInt32(data, 0);
        PrimitiveType t;

        switch (type)
        {
        case 0:  t = PrimitiveType.Sphere;   break;
        case 1:  t = PrimitiveType.Capsule;  break;
        case 2:  t = PrimitiveType.Cylinder; break;
        case 3:  t = PrimitiveType.Cube;     break;
        case 4:  t = PrimitiveType.Plane;    break;
        default: t = PrimitiveType.Quad;     break;
        }

        GameObject cube = GameObject.CreatePrimitive(t);
        cube.transform.position = new Vector3(0, 0, 0);
        cube.transform.rotation = new Quaternion(0, 0, 0, 1);
        cube.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
        cube.GetComponent<Renderer>().material = m_material;
        cube.SetActive(true);
        int key = cube.GetInstanceID();
        m_remote_objects.Add(key, cube);
        return (uint)key;
    }

    uint SetActive(byte[] data)
    {
        if (data == null || data.Length != 8) { return 0; }

        int key   = BitConverter.ToInt32(data, 0);
        int state = BitConverter.ToInt32(data, 4);

        GameObject go;
        if (!m_remote_objects.TryGetValue(key, out go)) { return 0; }
        go.SetActive(state != 0);

        return 1;
    }

    uint SetTransform(byte[] data)
    {
        if (data == null || data.Length != 8 + (10 * 4)) { return 0; }

        int key   = BitConverter.ToInt32(data, 0);
        int space = BitConverter.ToInt32(data, 4);
        float[] f = new float[10];
        for (int i = 0; i < f.Length; ++i) { f[i] = BitConverter.ToSingle(data, 8 + (i * 4)); }

        GameObject go;
        if (!m_remote_objects.TryGetValue(key, out go)) { return 0; }

        var position = new Vector3(f[0], f[1], f[2]);
        var rotation = new Quaternion(f[3], f[4], f[5], f[6]);
        var scale = new Vector3(f[7], f[8], f[9]);

        if (space == 0)
        {
            go.transform.parent = null;

            go.transform.SetPositionAndRotation(position, rotation);
            go.transform.localScale = scale;
        }
        else
        {
            go.transform.parent = transform;

            Camera cam = gameObject.GetComponent<Camera>();

            var half = scale / 2;

            var cc = cam.ScreenToWorldPoint(position);
            var ul = cam.ScreenToWorldPoint(new Vector3(cc.x - half.x, cc.y - half.y, cc.z));
            var ur = cam.ScreenToWorldPoint(new Vector3(cc.x + half.x, cc.y - half.y, cc.z));
            var bl = cam.ScreenToWorldPoint(new Vector3(cc.x - half.x, cc.y + half.y, cc.z));

            var dx = ul - ur;
            var dy = ul - bl;

            go.transform.localPosition = cc;
            go.transform.localRotation = rotation;
            go.transform.localScale = new Vector3(Mathf.Abs(dx.x), Mathf.Abs(dy.y), scale.z);
        }

        return 1;
    }

    uint SetColor(byte[] data)
    {
        if (data == null || data.Length != 4+4*4) { return 0; }

        int key = BitConverter.ToInt32(data, 0);
        float r = BitConverter.ToSingle(data, 4);
        float g = BitConverter.ToSingle(data, 8);
        float b = BitConverter.ToSingle(data, 12);
        float a = BitConverter.ToSingle(data, 16);

        GameObject go;
        if (!m_remote_objects.TryGetValue(key, out go)) { return 0; }

        go.GetComponent<Renderer>().material.color = new Color(r, g, b, a);

        return 1;
    }

    uint SetTexture(byte[] data)
    {
        if (data == null || data.Length < 4) { return 0; }

        int key = BitConverter.ToInt32(data, 0);
        GameObject go;
        if (!m_remote_objects.TryGetValue(key, out go)) { return 0; }

        if (data.Length > 4)
        {
            Texture2D tex = new Texture2D(2, 2);
            byte[] image = new byte[data.Length - 4];
            Array.Copy(data, 4, image, 0, image.Length);
            tex.LoadImage(image);

            go.GetComponent<Renderer>().material.mainTexture = tex;
        }
        else
        {
            go.GetComponent<Renderer>().material.mainTexture = null;
        }
        return 1;
    }

    uint Remove(byte[] data)
    {
        if (data.Length != 4) { return 0; }
        int key = BitConverter.ToInt32(data, 0);

        GameObject go;
        if (!m_remote_objects.TryGetValue(key, out go)) { return 0; }
        m_remote_objects.Remove(key);
        Destroy(go);

        return 1;
    }

    uint ProcessMessage(uint command, byte[] data)
    {
        uint ret = 0;

        switch (command)
        {
        case 0: ret = CreatePrimitive(data); break;
        case 1: ret = SetActive(data); break;
        case 2: ret = SetTransform(data); break;
        case 3: ret = SetColor(data);  break;
        case 4: ret = SetTexture(data); break;
        case 5: ret = Remove(data); break;
        }

        return ret;
    }
}
