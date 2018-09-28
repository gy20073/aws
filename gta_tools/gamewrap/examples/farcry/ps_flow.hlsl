Texture2D<float> prev_depth: register(t1);
Texture2D<float4> flow_depth: register(t0);

SamplerState S : register(s0) {
	Filter = MIN_MAG_MIP_LINEAR;
	AddressU = BORDER;
	AddressV = BORDER;
	BorderColor = float4(0, 0, 0, 0);
};

void main(in float4 p: SV_POSITION, in float2 t : TEX_COORD, out float2 flow : SV_Target0, out float depth : SV_Target1, out float occlusion: SV_Target2){
	//float4 f = T.Sample(S, t);
	uint W, H;
	flow_depth.GetDimensions(W, H);
	int x = t.x*(W - 1), y = t.y*(H - 1);
	float4 f = flow_depth.Load(int3(x, y, 0));

	float X = (f.x + 1) * 0.5 * W, Y = (1 - f.y) * 0.5 * H, D = f.z;

	// Get the prior depth
	float DD = prev_depth.Sample(S, float2((f.x + 1) * 0.5, (1 - f.y) * 0.5));
	depth = f.w;
	occlusion = 1. / (f.z + 1e-4) - 1. / (DD + 1e-4);
	if (f.z <= 0)
		flow = 0. / 0.; // NaN
	else
		flow = float2(X-x, Y-y) - 0.5;
}
