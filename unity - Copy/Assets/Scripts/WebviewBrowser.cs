using Microsoft.MixedReality.WebView;
using UnityEngine.UI;
using UnityEngine;
using TMPro;
using System;
using Microsoft.MixedReality.Toolkit.UI;

public class WebviewBrowser : MonoBehaviour
{
    // Declare UI elements: Back button, Go button, and URL input field
    public PressableButtonHoloLens2 BackButton;
    public PressableButtonHoloLens2 GoButton;
    public TMP_InputField URLField;
    public WebView webViewComponent;
    private void Start()
    {
        // Get the WebView component attached to the game object
        this.webViewComponent = gameObject.GetComponent<WebView>();
        webViewComponent.GetWebViewWhenReady((IWebView webView) =>
        {
            // If the WebView supports browser history, enable the Back button
            if (webView is IWithBrowserHistory history)
            {
                // Add an event listener for the Back button to navigate back in history
                BackButton.ButtonReleased.AddListener(() => history.GoBack());

                // Update the Back button's enabled state based on whether there's any history to go back to
                history.CanGoBackUpdated += CanGoBack;
            }

            // Add an event listener for the Go button to load the URL that was entered in the input field
            GoButton.ButtonReleased.AddListener(() => webView.Load(new Uri(URLField.text)));

            // Subscribe to the Navigated event to update the URL input field whenever a navigation occurs
            webView.Navigated += OnNavigated;

            // Set the initial value of the URL input field to the current URL of the WebView
            if (webView.Page != null)
            {
                URLField.text = webView.Page.AbsoluteUri;
            }
        });
    }

    // Update the URL input field with the new path after navigation
    private void OnNavigated(string path)
    {
        URLField.text = path;
    }

    // Enable or disable the Back button based on whether there's any history to go back to
    private void CanGoBack(bool value)
    {
        BackButton.enabled = value;
    }

    public void Navigate() {
        webViewComponent.GetWebViewWhenReady((IWebView webView) =>
        {
            webView.Load(new Uri(URLField.text));
            webView.Navigated += OnNavigated;

        });

    }
}