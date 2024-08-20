
using UnityEngine;

public class run_once : MonoBehaviour
{
    public string host;
    public static string host_address;

    // Start is called before the first frame update
    void Awake()
    {
        host_address = host;
        hl2ss.svc.initialize();
    }
}
