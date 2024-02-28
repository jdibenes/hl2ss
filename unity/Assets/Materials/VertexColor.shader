// Vertex color shader
// https://gist.github.com/jhorikawa/7a236ae0b801bf2aca2d8a3038b9bf40

Shader "Custom/VertexColor" {
	SubShader{
		Tags { "RenderType" = "Opaque" }
		LOD 200

		CGPROGRAM
		#pragma surface surf Lambert vertex:vert
		#pragma target 3.0

		struct Input {
			float4 vertColor;
		};

		void vert(inout appdata_full v, out Input o) {
			UNITY_INITIALIZE_OUTPUT(Input, o);
			o.vertColor = v.color;
		}

		void surf(Input IN, inout SurfaceOutput o) {
			o.Albedo = IN.vertColor.rgb;
		}
		ENDCG
	}
		FallBack "Diffuse"
}
