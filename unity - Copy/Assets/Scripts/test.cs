using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

public class test : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        byte[] bytes = Encoding.UTF8.GetBytes("zzzz<TITLE>SHKSDHFASKDJHFSADF<CONTENT>ASDLKFHALSDHJFAKJSDHFKASDHFKJH");
        string str = Encoding.UTF8.GetString(bytes);
        Debug.Log("TAG" + str.Length.ToString());
        SetPanelContent(bytes);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    uint SetPanelContent(byte[] data)
    {
        if (data.Length < 4) { return 0; }

        string str;
        if (data.Length > 4)
        {
            byte[] str_bytes = new byte[data.Length - 4];
            Array.Copy(data, 4, str_bytes, 0, str_bytes.Length);
            try { str = System.Text.Encoding.UTF8.GetString(str_bytes); } catch { return 0; }
        }
        else
        {
            str = "";
        }

        Debug.Log("TAG"+str);
        int startIndex = str.IndexOf("<CONTENT>");
        int endIndex = str.IndexOf("<CONTENT>", startIndex);
        string description = str.Substring(endIndex+9);

        string title = str.Substring(7, startIndex-7);
        TCPTestServer scrip2 = GameObject.FindObjectOfType(typeof(TCPTestServer)) as TCPTestServer;
        //scrip2.SendMessage(title + description);
        Debug.Log(title + "xxxx" + description);
        DictionaryFunctionality script = GameObject.FindObjectOfType(typeof(DictionaryFunctionality)) as DictionaryFunctionality;
        script.ChangeText(title, description);

        return 1;

    }
}
