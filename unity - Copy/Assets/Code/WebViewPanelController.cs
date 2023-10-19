using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static System.Net.WebRequestMethods;

public class WebViewPanelController : MonoBehaviour
{
    public WebviewBrowser webViewBrowser;

    public string CreateUrl(int web, string content)
    {

        switch (web) {
            case 0: return this.CreateGoogleTranslationUrl(content);
            default: return string.Empty;
        }
       
    }

    private string CreateGoogleTranslationUrl(string content) {
        string url = string.Empty;
        const string HEAD = "https://translate.google.com/?sl=en&tl=vi&text=";
        url = HEAD + content.Replace(" ", "%20") + "&op=translate";
        return url;
    }

    public void ChangeWeb(string url)
    {
        webViewBrowser.URLField.text = url;
        webViewBrowser.Navigate();
    }
}
