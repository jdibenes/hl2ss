//******************************************************************************
// This class demonstrates how to receive and process messages from a client.
// For a sample client implementation see client_ipc_umq.py in the viewer
// directory.
//******************************************************************************

using System.Text;
using UnityEngine;

public class IPCSkeleton : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        GetMessage(); // Process next message (if any)
    }

    // Get next message and process it
    bool GetMessage()
    {
        uint command;
        byte[] data;
        if (!hl2ss.PullMessage(out command, out data)) { return false; } // If there are no messages in the queue return false else get the next message
        hl2ss.PushResult(ProcessMessage(command, data)); // Process the message and send the result (uint) to the client
        hl2ss.AcknowledgeMessage(command); // Signal to the library that the message has been processed
        return true; // Done
    }

    // Process message
    uint ProcessMessage(uint command, byte[] data)
    {
        uint ret = 0;

        switch (command)
        {
        // Add your custom message calls here ---------------------------------
        case 0xFFFFFFFEU: ret = MSG_DebugMessage(data); break; // Sample message, feel free to remove it (and its method)
        case 0xFFFFFFFFU: ret = MSG_Disconnect(data); break; // Reserved, do not use 0xFFFFFFFF for your custom messages
        }

        return ret;
    }

    // Client disconnected
    uint MSG_Disconnect(byte[] data)
    {
        // Implement your OnClientDisconnected logic here
        return ~0U; // Return value does not matter since there is no client anymore
    }

    // Add your custom message methods here -----------------------------------
    // You can use the BitConverter class to unpack data (such as floats, ints) from the data byte array
    // See the RemoteUnityScene.cs script for more examples

    // Client sent a string for the debugger (Visual Studio)
    uint MSG_DebugMessage(byte[] data)
    {
        string str;
        try { str = Encoding.UTF8.GetString(data); } catch { return 0; } // Decode string and return 0 to the client if decode failed
        hl2ss.Print(str); // Send string to debugger
        return 1; // Return 1 to the client to indicate success
    }
}
