using UnityEngine;

namespace tcn
{
    public class CommConfig : MonoBehaviour
    {
        [Tooltip("Enter the Zenoh Topic Prefix.")]
        public string topicPrefix = "tcn/loc/app/entity00";

        [Tooltip("Optional Zenoh Configuration as json string.")]
        public string zenohConfig = "";

        void Start()
        {
            UnityEngine.Debug.Log("Initialize hl2comm");
            hl2comm.Initialize(topicPrefix, zenohConfig);
        }

        void OnDestroy()
        {
            UnityEngine.Debug.Log("Teardown hl2comm");
            hl2comm.Teardown();
            UnityEngine.Debug.Log("Teardown finished");
        }

    }
}