RWTexture2D<uint> id: register(u2);

[earlydepthstencil]
void main(in float4 pos: SV_Position, in float4 c : ID_COORD) {
	int2 p = pos.xy;
	id[p] = c.x;
}
