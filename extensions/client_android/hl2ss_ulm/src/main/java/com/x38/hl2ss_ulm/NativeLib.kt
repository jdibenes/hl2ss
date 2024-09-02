package com.x38.hl2ss_ulm

class NativeLib {

    /**
     * A native method that is implemented by the 'hl2ss_ulm' native library,
     * which is packaged with this application.
     */
    external fun stringFromJNI(): String

    companion object {
        // Used to load the 'hl2ss_ulm' library on application startup.
        init {
            System.loadLibrary("hl2ss_ulm")
        }
    }
}