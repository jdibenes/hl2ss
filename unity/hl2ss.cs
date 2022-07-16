
using System.Runtime.InteropServices;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class hl2ss : MonoBehaviour
{
    [DllImport("hl2ss")]
    private static extern void InitializeStreams();

    // Start is called before the first frame update
    void Start()
    {
        InitializeStreams();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
