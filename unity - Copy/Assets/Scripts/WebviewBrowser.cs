using Microsoft.MixedReality.WebView;
using UnityEngine.UI;
using UnityEngine;
using TMPro;
using System;
using Microsoft.MixedReality.Toolkit.UI;
using UnityEngine.EventSystems;
using System.Threading.Tasks;
using Microsoft.MixedReality.Toolkit.Input;

public class WebviewBrowser : MonoBehaviour, IMixedRealityPointerHandler, IMixedRealityTouchHandler


{
    // Declare UI elements: Back button, Go button, and URL input field
    public PressableButtonHoloLens2 BackButton;   
    public PressableButtonHoloLens2 GoButton;
    public TMP_InputField URLField;
    public WebView webViewComponent;
    public MeshCollider collider;
    public event WebView_OnNavigated Navigated;
    public event WebView_OnNewWindowRequested NewWindowRequested;
    public event WebView_OnCloseRequested WindowCloseRequested;

    

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

    void Update()
    {
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
        // Get the WebView component attached to the game object
        this.webViewComponent = gameObject.GetComponent<WebView>();
        webViewComponent.GetWebViewWhenReady((IWebView webView) =>
        {
            // If the WebView supports browser history, enable the Back button
     

            // Add an event listener for the Go button to load the URL that was entered in the input field
            webView.Load(new Uri(URLField.text));

            // Subscribe to the Navigated event to update the URL input field whenever a navigation occurs
            webView.Navigated += OnNavigated;

            // Set the initial value of the URL input field to the current URL of the WebView
            if (webView.Page != null)
            {
                URLField.text = webView.Page.AbsoluteUri;
            }
        });
    }



    private Vector2 NormalizeWorldPoint(Vector3 worldPoint)
    {
        // Convert the world point to our control's local space.
        Vector3 localPoint = transform.InverseTransformPoint(worldPoint);

        var boundsSize = collider.sharedMesh.bounds.size;
        var boundsExtents = collider.sharedMesh.bounds.max;

        // Adjust the point to be based on a 0,0 origin.
        var uvTouchPoint = new Vector2((localPoint.x + boundsExtents.x), -1.0f * (localPoint.y - boundsExtents.y));

        // Normalize the point so it can be mapped to the WebView's texture.
        var normalizedPoint = new Vector2(uvTouchPoint.x / boundsSize.x, uvTouchPoint.y / boundsSize.y);

        return normalizedPoint;
    }

    public void OnPointerDown(MixedRealityPointerEventData eventData)
    {
        Debug.Log("444 Click");
        this.webViewComponent = gameObject.GetComponent<WebView>();
        webViewComponent.GetWebViewWhenReady((IWebView webView) =>
        {
            var hitCoord = NormalizeWorldPoint(eventData.Pointer.Result.Details.Point);

            hitCoord.x *= webView.Width;
            hitCoord.y *= webView.Height;

            var mouseEventsWebView = webView as IWithMouseEvents;
            if (mouseEventsWebView is IWithBrowserHistory history)
            {
                print("ok");
                WebViewMouseEventData mouseEvent = new WebViewMouseEventData
                {
                    X = (int)hitCoord.x,
                    Y = (int)hitCoord.y,
                    Device = WebViewMouseEventData.DeviceType.Mouse,
                    Type = WebViewMouseEventData.EventType.MouseDown,
                    Button = WebViewMouseEventData.MouseButton.ButtonLeft,
                };

                mouseEventsWebView.MouseEvent(mouseEvent);

                // To register as a click, the WebView needs to be a mouse-up event.
                mouseEvent.Type = WebViewMouseEventData.EventType.MouseUp;
                mouseEventsWebView.MouseEvent(mouseEvent);
            }
        });
    }

    public void OnPointerDragged(MixedRealityPointerEventData eventData)
    {
    }

    public void OnPointerUp(MixedRealityPointerEventData eventData)
    {
    }

    public void OnPointerClicked(MixedRealityPointerEventData eventData)
    {
        Debug.Log("444 Click");
        this.webViewComponent = gameObject.GetComponent<WebView>();
        webViewComponent.GetWebViewWhenReady((IWebView webView) =>
        {
            Debug.Log("55556" + eventData.Pointer.Result.Details.Point.ToString());

            var hitCoord = NormalizeWorldPoint(eventData.Pointer.Result.Details.Point);

            Debug.Log("5555" + hitCoord.ToString());

            hitCoord.x *= webView.Width;
            hitCoord.y *= webView.Height;
            Debug.Log("555" + hitCoord.ToString());

            var mouseEventsWebView = webView as IWithMouseEvents;
            WebViewMouseEventData mouseEvent = new WebViewMouseEventData
            {
                X = (int)hitCoord.x,
                Y = (int)hitCoord.y,
                Device = WebViewMouseEventData.DeviceType.Pointer,
                Type = WebViewMouseEventData.EventType.MouseDown,
                Button = WebViewMouseEventData.MouseButton.ButtonLeft,
                TertiaryAxisDeviceType = WebViewMouseEventData.TertiaryAxisDevice.PointingDevice
            };

            mouseEventsWebView.MouseEvent(mouseEvent);

            // To register as a click, the WebView needs to be a mouse-up event.
            mouseEvent.Type = WebViewMouseEventData.EventType.MouseUp;
            mouseEventsWebView.MouseEvent(mouseEvent);
        });
    }

    public void OnTouchStarted(HandTrackingInputEventData eventData)
    {
      
    }

    public void OnTouchCompleted(HandTrackingInputEventData eventData)
    {
        this.webViewComponent = gameObject.GetComponent<WebView>();
        webViewComponent.GetWebViewWhenReady((IWebView webView) =>
        {
            Vector3 localScale = this.transform.localScale;
            Vector3 relative = this.transform.InverseTransformPoint(eventData.InputData);
            localScale = new Vector3(0.5f, -0.5f, 0f);
            string check = "";
            Vector3 tmp = (relative + localScale);

            tmp.x *= webView.Width;
            tmp.y *= webView.Height;

            var mouseEventsWebView = webView as IWithMouseEvents;
            if (mouseEventsWebView is IWithBrowserHistory history)
            {
                WebViewMouseEventData mouseEvent = new WebViewMouseEventData
                {
                    X = (int)tmp.x,
                    Y = (int)tmp.y,
                    Device = WebViewMouseEventData.DeviceType.Mouse,
                    Type = WebViewMouseEventData.EventType.MouseDown,
                    Button = WebViewMouseEventData.MouseButton.ButtonLeft,
                };
                mouseEventsWebView.MouseEvent(mouseEvent);
                mouseEvent.Button = WebViewMouseEventData.MouseButton.ButtonRight;
                mouseEventsWebView.MouseEvent(mouseEvent);
            }
        });

    }

    public void OnTouchUpdated(HandTrackingInputEventData eventData)
    {
        throw new NotImplementedException();
    }
}