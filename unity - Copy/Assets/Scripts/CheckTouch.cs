using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public class CheckTouch : MonoBehaviour, IMixedRealityTouchHandler
{
    public AudioSource sound;
    public TCPTestServer script;
    public GameObject thisObject;
    public GameObject MainController;
    public MainControlPanelToggleManagement mainControlPanelToggleManagement;
    public int mode = 0;
    void IMixedRealityTouchHandler.OnTouchCompleted(HandTrackingInputEventData eventData)
    {
        sound.Play();

        Vector3 localScale = thisObject.transform.localScale;
        Vector3 relative = this.thisObject.transform.InverseTransformPoint(eventData.InputData);
        localScale = new Vector3(0.5f, -0.5f, 0f);
        string check = "";
        Vector3 tmp = (relative + localScale);
        check = new Vector2(tmp.x, -tmp.y).ToString("F8");
        if (mainControlPanelToggleManagement.GetToggleIndex() == 0)
        {
            if (this.thisObject.tag == "RightPlane")

                script.SendMessage("MODE2" + check);
            else script.SendMessage("MODE1" + check);
        }
        else if (mainControlPanelToggleManagement.GetToggleIndex() == 1)
        {
            if (this.thisObject.tag == "RightPlane")

                script.SendMessage("MODE7" + check);
            else script.SendMessage("MODE6" + check);
        } else if(mainControlPanelToggleManagement.GetToggleIndex() == 2)
        {
            if (this.thisObject.tag == "RightPlane")

                script.SendMessage("MODE5" + check);
            else script.SendMessage("MODE4" + check);
        }
      
        //string ptrName = eventData.Pointer.PointerName;
        Debug.Log($"Touch started from ");
    }

    void IMixedRealityTouchHandler.OnTouchStarted(HandTrackingInputEventData eventData)
    {
 

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
        mainControlPanelToggleManagement = GameObject.FindObjectOfType(typeof(MainControlPanelToggleManagement)) as MainControlPanelToggleManagement;

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
