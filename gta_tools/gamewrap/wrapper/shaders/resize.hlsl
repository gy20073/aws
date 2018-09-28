#ifndef TYPE
#define TYPE float4
#endif

Texture2D<TYPE> image: register(t0);
SamplerState  s0 = sampler_state {
	AddressU = Clamp;
	AddressV = Clamp;
};
RWTexture2D<TYPE> result : register(u0);

#ifdef SRGB
float4 colorcvt(float4 RGB) {
	// RGB -> SRGB approximation
	float3 S1 = sqrt(RGB.xyz);
	float3 S2 = sqrt(S1);
	float3 S3 = sqrt(S2);
	return float4(0.662002687 * S1 + 0.684122060 * S2 - 0.323583601 * S3 - 0.0225411470 * RGB.xyz, RGB.w);
}
#endif

[numthreads(32, 16, 1)]
void main( uint3 id : SV_DispatchThreadID ) {
	uint2 xy = id.xy;
	uint iH, iW, oW, oH;
	image.GetDimensions(iW, iH);
	result.GetDimensions(oW, oH);
#ifdef LOAD
	float2 S = float2((iW - 1.) / (oW - 1.), (iH - 1.) / (oH - 1.));
#else
	float2 S = float2(1. / (oW - 1.), 1. / (oH - 1.));
	float LOD = min((iW - 1.) / (oW - 1.), (iH - 1.) / (oH - 1.)) - 1;
#endif
	if (xy.x < oW && xy.y < oH)
#ifdef SRGB
		result[xy] = colorcvt(image.SampleLevel(s0, xy*S, LOD));
#else
#ifdef LOAD
		// Nearest neighbor interpolation
		result[xy] = image.Load(int3(round(xy*S), 0));
#else
		result[xy] = image.SampleLevel(s0, xy*S, LOD);
#endif
#endif
}
