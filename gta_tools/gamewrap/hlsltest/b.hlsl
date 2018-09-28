cbuffer InjectBuffer {
	float4x4 wvp;
	float4x4 w;
};
cbuffer InjectBuffer2 {
	float4x4 color_mat;
};
void main(in float3 pos: POSITION, in float3 col: COLOR, out float4 p: SV_Position, out float4 c: COL) {
	float4 pp = mul(float4(pos, 1), w);
	p = mul(float4(pos, 1), wvp);
	c = mul(float4(col, 1), color_mat) + pp;
}