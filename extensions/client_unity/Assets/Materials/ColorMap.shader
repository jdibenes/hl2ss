Shader "Hidden/ColorMap"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        _ColorMapTex ("Texture", 2D) = "white" {}
        _Lf ("Float", Float) = 0.0
        _Rf ("Float", Float) = 255.0
    }
    SubShader
    {
        // No culling or depth
        Cull Off ZWrite Off ZTest Always

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }

            sampler2D _MainTex;
            sampler2D _ColorMapTex;
            float _Lf;
            float _Rf;

            fixed4 frag (v2f i) : SV_Target
            {
                fixed4 col = tex2D(_MainTex, i.uv);
                float r = (col.r - _Lf) / (_Rf - _Lf);
                float2 r_uv = float2(r, r);
                fixed4 map = tex2D(_ColorMapTex, r_uv);
                return map;
            }
            ENDCG
        }
    }
}
