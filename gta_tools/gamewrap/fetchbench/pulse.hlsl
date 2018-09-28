cbuffer GID: register(b0) {
	float gid = 0;
}

float main(uint id: SV_VertexID): RESULT {
	return (float)id + gid;
}
