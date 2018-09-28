cbuffer InjectBuffer {
	uint base_id;
};
cbuffer InjectBuffer2 {
	uint base_id2;
};
int main( uint pid: SV_InstanceID) : ID
{
	return pid + base_id + base_id2;
}