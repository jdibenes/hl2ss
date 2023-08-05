using UnityEngine;

public class CommConfig : MonoBehaviour
{
    [Tooltip("Enter the Zenoh Topic Prefix.")]
    public string topicPrefix = "tcn/loc/app/entity00";

    [Tooltip("Optional Zenoh Configuration as json string.")]
    public string zenohConfig = "";

    void Start()
    {
        hl2comm.Initialize(topicPrefix, zenohConfig);
    }

}
