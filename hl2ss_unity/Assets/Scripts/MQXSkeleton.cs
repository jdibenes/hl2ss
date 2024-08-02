
using System.Text;
using System.Runtime.InteropServices;
using UnityEngine;

public class MQXSkeleton : MonoBehaviour
{
    private uint m_state = 0;
    private string m_text = "Hello from HoloLens 2!";

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        switch (m_state)
        {
        case 0:
            byte[] data = Encoding.UTF8.GetBytes(m_text);
            GCHandle h = GCHandle.Alloc(data, GCHandleType.Pinned);
            hl2ss.PushMessage(0xFFFFFFFE, (uint)data.Length, h.AddrOfPinnedObject());
            h.Free();
            m_state = 1;
            break;
        case 1:
            uint result;
            if (!hl2ss.PullResult(out result)) { break; }
            hl2ss.AcknowledgeResult(result); // result == ~0U means client disconnected
            m_state = 0;
            break;
        }
    }
}
