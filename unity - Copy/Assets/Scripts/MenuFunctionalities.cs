using Microsoft.MixedReality.Toolkit;
using System.Collections;
using System.Collections.Generic;
using System.Transactions;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;

public class MenuFunctionalities : MonoBehaviour
{
    public int mode; // 1 = ; 2 = 
    public int min_left_page = 0;
    public int max = 300;
    public int currentPage;
    public GameObject tcp_object;
    public GameObject tcp_object_prefab;

    public TCPTestServer script;
    public TMPro.TextMeshPro textMeshPro;
    public GameObject left_panel_control;
    public GameObject right_panel_control;
    void Awake()
    {
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

    public void PreviousPage()
    {
        if (currentPage > 0)
        {

            currentPage -= 2; //left
            textMeshPro.text = currentPage.ToString() + " and " + (currentPage + 1).ToString(); 
            script.SendMessage("MODE3-1");

        }
    }
    public void NextPage()
    {
        if (currentPage <= max) {
            currentPage += 2;
            textMeshPro.text = currentPage.ToString() + " and " + (currentPage + 1).ToString();
            script.SendMessage("MODE3+1");

        }
         
    }

    public void ChangeMode(int mode){
        this.mode = mode;
    }

    public void Search() {
        // show the keyboard and allow user to search word
        // or using voice i guess

        return;
    }

    public void ResetPanel() {
        PageInformationHolder leftPageInformationHolder = this.left_panel_control.GetComponent<PageInformationHolder>();
        PageInformationHolder rightPageInformationHolder = this.right_panel_control.GetComponent<PageInformationHolder>();
        Destroy(tcp_object);
        tcp_object = Instantiate(this.tcp_object_prefab);
        script = GameObject.FindObjectOfType(typeof(TCPTestServer)) as TCPTestServer;

        leftPageInformationHolder.DestroyAllVisualizedObject();
        leftPageInformationHolder.VisualizeAll();
        rightPageInformationHolder.DestroyAllVisualizedObject();
        rightPageInformationHolder.VisualizeAll();
        return;
    }


   
}
