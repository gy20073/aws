Texture2D<float4> flow_depth: register(t0);
RWTexture2D<float2> flow: register(u0);
RWTexture2D<float> depth: register(u1);
RWTexture2D<float> segment: register(u2);

[numthreads(32, 16, 1)]
void main(uint3 i : SV_DispatchThreadID) {
	uint2 xy = i.xy;
	uint W, H;
	flow_depth.GetDimensions(W, H);
	if (xy.x < W && xy.y < H) {
		float4 f = flow_depth.Load(int3(xy,0));
		depth[xy] = f.z;
		segment[xy] = f.w;
		if (f.z <= 0)
			flow[xy] = 0. / 0.; // NaN
		else
			flow[xy] = float2( (f.x+1) * 0.5 * W, (1 - f.y) * 0.5 * H ) - float2(xy) - 0.5;
	}
}
