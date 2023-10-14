using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class CheckTouch : MonoBehaviour, IMixedRealityTouchHandler
{
    public AudioSource sound;
    public TCPTestServer script;
    public GameObject thisObject;
    void IMixedRealityTouchHandler.OnTouchCompleted(HandTrackingInputEventData eventData)
    {
    }

    void IMixedRealityTouchHandler.OnTouchStarted(HandTrackingInputEventData eventData)
    {
        sound.Play();

        Vector3 localScale = thisObject.transform.localScale;
        Vector3 relative = this.thisObject.transform.InverseTransformPoint(eventData.InputData);
        localScale = new Vector3(0.5f, -0.5f, 0f);
        string check = "";
        Vector3 tmp = (relative + localScale);
        check = new Vector2(tmp.x, -tmp.y).ToString("F8");


        script.SendMessage("MODE1" +  check);

        //string ptrName = eventData.Pointer.PointerName;
        Debug.Log($"Touch started from ");

    }
    
    void IMixedRealityTouchHandler.OnTouchUpdated(HandTrackingInputEventData eventData)
    {
    }
    /*
    void OnCollisionEnter(Collision collision)
    {
        ContactPoint contact = collision.GetContact(0);
        Vector3 pos = contact.point;
        script.SendMessage("contact");
        script.SendMessage(pos.ToString());
    }
    */
    void Awake() {
        script = GameObject.FindObjectOfType(typeof(TCPTestServer)) as TCPTestServer;
    }
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
}
}
