
using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using System.Data.SqlTypes;
using Microsoft.MixedReality.Toolkit.Input;

public class RemoteUnityScene : MonoBehaviour
{
    private Dictionary<int, GameObject> m_remote_objects;
    private bool m_loop;
    private bool m_mode;
    private int m_last_key;
    public PageInformationHolder page_information_holder_left;
    public PageInformationHolder page_information_holder_right;

    public GameObject descriptionPanel;

    public int set_page_item_mode = 0; // 1 = left, 2 = right, 0 = nah
    public int content_mode = 1; // 0 = offline content, 1 = webview
    public GameObject  controlPanelManager;
    [Tooltip("Set to BasicMaterial to support semi-transparent primitives.")]
    public Material m_material;

    
    void Start()
    {

        m_remote_objects = new Dictionary<int, GameObject>();
        m_loop = false;
        m_mode = false;
        page_information_holder_left = GameObject.Find("PageInformationHolderLeft").GetComponent<PageInformationHolder>();
        page_information_holder_right = GameObject.Find("PageInformationHolderRight").GetComponent<PageInformationHolder>();
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
            case   0: ret = MSG_CreatePrimitive(data);   break;
            case   1: ret = MSG_SetActive(data);         break;
            case   2: ret = MSG_SetWorldTransform(data); break;
            case   3: ret = MSG_SetLocalTransform(data); break;
            case   4: ret = MSG_SetColor(data);          break;
            case   5: ret = MSG_SetTexture(data);        break;
            case   6: ret = MSG_CreateText(data);        break;
            case   7: ret = MSG_SetText(data);           break;

            //TODO: function calls 

            //case   8: ret = MSG_SetColor(data);          break;
            case   9: ret = SetPanelContent(data);          break;
            case 21: ret = SetImageContent(data); break;
            case 22: ret = SetRefContent(data); break;

            //case  10: ret = MSG_SetColor(data);          break;
            case 11: ret = GetASingleItem(data);          break;
            
            case 12: ret = SetPageItemMode(data);         break;
            case 13: ret = SetPageSize(data); break;
            case 14: ret = Visualize(data); break;

            case 16: ret = MSG_Remove(data);            break;
            case  17: ret = MSG_RemoveAll(data);         break;
            case  18: ret = MSG_BeginDisplayList(data);  break;
            case  19: ret = MSG_EndDisplayList(data);    break;
            case  20: ret = MSG_SetTargetMode(data);     break;
            case ~0U: ret = MSG_Disconnect(data);        break;
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
            try { str = System.Text.Encoding.UTF8.GetString(str_bytes); } catch { return 0; }
        }
        else
        {
            str = "";
        }

        tmp.text = str;

        return 1;
    }

    // MY FUNCTIONALITIES

    // NOT SURE IF IT IS OK

    uint SetPanelContent(byte[] data) {
        switch (content_mode) {
            case 0: return this.SetOfflinePanelContent(data);
            case 1: return this.SetWebViewPanelContent(data);
        }
        return 1;
    }

    uint SetImageContent(byte[] data)
    {
        if (data.Length < 4) { return 0; }
        //TCPTestServer scrip2 = GameObject.FindObjectOfType(typeof(TCPTestServer)) as TCPTestServer;
        //scrip2.SendMessage("OK1");

        string str;
        str = System.Text.Encoding.UTF8.GetString(data);
        //scrip2.SendMessage("ok2");

        int startIndex = str.IndexOf("<CONTENT>");
        int endIndex = str.IndexOf("<CONTENT>", startIndex);
        string description = str.Substring(endIndex + 9);

        string title = str.Substring(7, startIndex - 7);
        //scrip2.SendMessage(title + description);
        //DictionaryFunctionality script = GameObject.FindObjectOfType(typeof(DictionaryFunctionality)) as DictionaryFunctionality;
        //script.ChangeText(title, description);
        //TODO: do something
        WebViewPanelController script = GameObject.FindObjectOfType(typeof(WebViewPanelController)) as WebViewPanelController;
        script.ChangeWeb(description);
        return 1;
    }

    uint SetRefContent(byte[] data)
    {
        if (data.Length < 4) { return 0; }
        //TCPTestServer scrip2 = GameObject.FindObjectOfType(typeof(TCPTestServer)) as TCPTestServer;
        //scrip2.SendMessage("OK1");

        string str;
        str = System.Text.Encoding.UTF8.GetString(data);
        //scrip2.SendMessage("ok2");

        int startIndex = str.IndexOf("<CONTENT>");
        int endIndex = str.IndexOf("<CONTENT>", startIndex);
        string description = str.Substring(endIndex + 9);

        string title = str.Substring(7, startIndex - 7);
        //scrip2.SendMessage(title + description);
        //DictionaryFunctionality script = GameObject.FindObjectOfType(typeof(DictionaryFunctionality)) as DictionaryFunctionality;
        //script.ChangeText(title, description);
        WebViewPanelController script = GameObject.FindObjectOfType(typeof(WebViewPanelController)) as WebViewPanelController;
        script.ChangeWeb(description);

        return 1;
    }
    uint SetOfflinePanelContent(byte[] data) {
        if (data.Length < 4) { return 0; }
        //TCPTestServer scrip2 = GameObject.FindObjectOfType(typeof(TCPTestServer)) as TCPTestServer;
        //scrip2.SendMessage("OK1");

        string str;
        str = System.Text.Encoding.UTF8.GetString(data);
        //scrip2.SendMessage("ok2");

        int startIndex = str.IndexOf("<CONTENT>");
        int endIndex = str.IndexOf("<CONTENT>", startIndex);
        string description = str.Substring(endIndex + 9);

        string title = str.Substring(7, startIndex - 7);
        //scrip2.SendMessage(title + description);
        DictionaryFunctionality script = GameObject.FindObjectOfType(typeof(DictionaryFunctionality)) as DictionaryFunctionality;
        script.ChangeText(title, description);

        return 1;

    }

    uint SetWebViewPanelContent(byte[] data)
    {
        if (data.Length < 4) { return 0; }
        //TCPTestServer scrip2 = GameObject.FindObjectOfType(typeof(TCPTestServer)) as TCPTestServer;
        //scrip2.SendMessage("OK1");

        string str;
        str = System.Text.Encoding.UTF8.GetString(data);
        //scrip2.SendMessage("ok2");

        int startIndex = str.IndexOf("<CONTENT>");
        int endIndex = str.IndexOf("<CONTENT>", startIndex);
        string description = str.Substring(endIndex + 9);

        string title = str.Substring(7, startIndex - 7);
        //scrip2.SendMessage(title + description);
        WebViewPanelController script = GameObject.FindObjectOfType(typeof(WebViewPanelController)) as WebViewPanelController;
        script.ChangeWeb(script.CreateUrl(description));
        return 1;

    }


    int GetInt(byte[] data)
    {
        return BitConverter.ToInt32(data, 0);
    }


    uint GetASingleItem(byte[] data) {
        
        if (this.set_page_item_mode == 0) {
            return 0;
        } 
        if (data.Length < 4) { return 0; }
        string item_type;
;
        int xmin, xmax, ymin, ymax;
       
        byte[] tmp = new byte[4];
        Array.Copy(data, 0, tmp, 0, 4);
        xmin = GetInt(tmp);
        Array.Copy(data, 4, tmp, 0, 4);
        xmax = GetInt(tmp);
        Array.Copy(data, 8, tmp, 0, 4);
        ymin = GetInt(tmp);
        Array.Copy(data, 12, tmp, 0, 4);
        ymax = GetInt(tmp);
        //TCPTestServer scrip2 = GameObject.FindObjectOfType(typeof(TCPTestServer)) as TCPTestServer;
        //scrip2.SendMessage("REHECK HERE"  + xmax.ToString() + ", " + ymax.ToString() + ", " + xmin.ToString() + ", " + ymin.ToString());
        
        if (data.Length > 16)
        {
            byte[] str_bytes = new byte[data.Length - 16];
            Array.Copy(data, 16, str_bytes, 0, str_bytes.Length);
            try { item_type = System.Text.Encoding.UTF8.GetString(str_bytes); } catch { return 0; }
            
        }
        else
        {
            item_type = "";
        }
        
        item_type = "";
        //scrip2.SendMessage("REHECK HERE" + " " + item_type + " " + xmax.ToString() + ", " + ymax.ToString() + ", " + xmin.ToString() + ", " + ymin.ToString());

        if (this.set_page_item_mode == 1)
        {
            this.page_information_holder_left.AddItem(item_type, xmin, xmax, ymin, ymax);
        }
        else if (this.set_page_item_mode == 2)
        {
            this.page_information_holder_right.AddItem(item_type, xmin, xmax, ymin, ymax);
        }
      

        return 1;
    }
    uint SetPageItemMode(byte[] data) {
        TCPTestServer scrip2 = GameObject.FindObjectOfType(typeof(TCPTestServer)) as TCPTestServer;
        scrip2.SendMessage("REHECK HERE");

        if (data.Length < 4) { return 0; }
        this.set_page_item_mode = GetInt(data);
        scrip2.SendMessage("REHECK HERE" + this.set_page_item_mode.ToString());


        return 1;
    }

    uint SetPageSize(byte[] data)
    {
        byte[] tmp = new byte[4];

        if (this.set_page_item_mode == 1)
        {
            Array.Copy(data, 0, tmp, 0, 4);
            this.page_information_holder_left.width_page = GetInt(tmp);
            Array.Copy(data, 4, tmp, 0, 4);
            this.page_information_holder_left.height_page = GetInt(tmp);
        }
        else if (this.set_page_item_mode == 2)
        {
            Array.Copy(data, 0, tmp, 0, 4);
            this.page_information_holder_right.width_page = GetInt(tmp);
            Array.Copy(data, 4, tmp, 0, 4);
            this.page_information_holder_right.height_page = GetInt(tmp);
        }
        return 1;
    }

    uint Visualize(byte[] data) {
        if (this.set_page_item_mode == 1)
        {
           
            this.page_information_holder_left.VisualizeAll();
        }
        else if (this.set_page_item_mode == 2)
        {
         
            this.page_information_holder_right.VisualizeAll();
        }
        return 1;
    }
}
