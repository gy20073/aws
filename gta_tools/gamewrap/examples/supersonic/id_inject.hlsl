cbuffer InjectBuffer: register(b7) {
	uint base_id = 0;
};

float4 main(uint id: SV_InstanceID) : ID_COORD {
	return float4(base_id ? base_id + id : 0, 1, 2, 3);
}
