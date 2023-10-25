using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static System.Net.WebRequestMethods;

public class WebViewPanelController : MonoBehaviour
{
    public WebviewBrowser webViewBrowser;
    public GameObject webviewPanel;
    public int web = 0;
    public DictionarySelectionToggle dictionarySelectionToggle;
    private void Start()
    {
        //ChangeWeb(CreateGoogleTranslationUrl("that is ver nice"));
    }
    public string CreateUrl( string content)

    {

        switch (dictionarySelectionToggle.GetToggleIndex()) {
            case 0: return this.CreateGoogleTranslationUrl(content);
            case 1: return this.CreateOxfordLearnerDictionaryUrl(content);

            default: return this.CreateGoogleTranslationUrl(content);
        }

    }

    private string CreateGoogleTranslationUrl(string content) {
        string url = string.Empty;
        const string HEAD = "https://translate.google.com/?sl=en&tl=vi&text=";
        url = HEAD + content.Replace(" ", "%20") + "&op=translate";
        return url;
    }

    private string CreateOxfordLearnerDictionaryUrl(string content) {
        string url = string.Empty;
        const string HEAD = "https://www.oxfordlearnersdictionaries.com/definition/english/?q=";
        url = HEAD + content.Replace(" ", "+");
        return url;
    }
   
    public void ChangeWeb(string url)
    {
        this.webviewPanel.SetActive(true);
        this.webviewPanel.transform.position = Camera.main.transform.position + Camera.main.transform.forward * 0.35f;
        this.webviewPanel.transform.rotation = new Quaternion(0.0f, Camera.main.transform.rotation.y, 0.0f, Camera.main.transform.rotation.w);

        webViewBrowser.URLField.text = url;
        webViewBrowser.Navigate();
    }
}
