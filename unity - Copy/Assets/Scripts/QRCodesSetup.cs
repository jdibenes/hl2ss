using System;
using System.Collections;

using System.Collections.Generic;

using UnityEngine;

using Microsoft.MixedReality.QR;
namespace QRTracking
{
    public class QRCodesSetup : MonoBehaviour
    {
        [Tooltip("Determines if the QR codes scanner should be automatically started.")]
        public bool AutoStartQRTracking = true;

        [Tooltip("Visualize the detected QRCodes in the 3d space.")]
        public bool VisualizeQRCodes = true;

        QRCodesManager qrCodesManager = null;

        void Awake()
        {
            qrCodesManager = QRCodesManager.Instance;
            if (AutoStartQRTracking)
            {
                qrCodesManager.StartQRTracking();
            }
            if (VisualizeQRCodes)
            {
                gameObject.AddComponent(typeof(QRTracking.QRCodesVisualizer));
            }
        }

        void Start()
        {
            
        }

        void Update()
        {

        }
    }
}
