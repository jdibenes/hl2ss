using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
public class DictionaryFunctionality : MonoBehaviour
{
    public TextMeshPro title;
    public GameObject description;
    public GameObject dictionaryPanel;
    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        //this.ChangeText("TITLE", "This text turns<color=\"red\"> red\r\n\r\n");

    }

    public void ChangeText(string title, string description){
        this.dictionaryPanel.SetActive(true);
        if (!string.IsNullOrEmpty(title))
        {
            this.title.text = title;
        }
        if (!string.IsNullOrEmpty(description))
        {
            this.dictionaryPanel.transform.position = Camera.main.transform.forward * 0.25f ;

            this.description.GetComponent<TextMeshProUGUI>().text = description;
        }
        return;
    }
}
