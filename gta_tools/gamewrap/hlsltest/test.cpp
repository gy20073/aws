#include "hlsl.h"
#include "util.h"
#include <iostream>
#include <fstream>
#include <d3d11.h>
#include "a.h"
#include "b.h"

#include <D3Dcompiler.h>
#pragma comment(lib,"d3dcompiler.lib")

LOG::~LOG() {
	s_ << std::endl;
	s_.flush();
	std::cout << s_.str();
	std::cout.flush();
}
std::string strip(const std::string & i) {
	size_t a = 0, b = i.size();
	while (a < b && isblank(i[a])) a++;
	while (b > 1 && isblank(i[b - 1])) b--;
	return i.substr(a, b);
}
std::vector<std::string> split(const std::string & s, char c) {
	std::vector<std::string> r;
	for (size_t i = 0; i < s.size(); i++) {
		size_t j = s.find(c, i);
		r.push_back(s.substr(i, j));
		if (j == s.npos) break;
		i = j;
	}
	return r;
}

size_t hash_combine(size_t a, size_t b) {
	return a ^ (b + 0x9e3779b9 + (a << 6) + (a >> 2));
}

std::string dis(const HLSL::ByteCode & b) {
	ID3DBlob * dis;
	D3DDisassemble(b.data(), b.size(), 0, nullptr, &dis);
	std::string r = (const char*)dis->GetBufferPointer();
	dis->Release();
	return r;
}
std::string dis(const HLSL & h) {
	return dis(h.write());
}
LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) { return 0; }
#include <d3d11.h>
#pragma comment(lib, "d3d11.lib")
void testd3d() {
	static const D3D_FEATURE_LEVEL force_feature_11_1[] = { D3D_FEATURE_LEVEL_11_1 };
	ID3D11Device * d;
	ID3D11DeviceContext * c;
	D3D_FEATURE_LEVEL f;
	DXGI_SWAP_CHAIN_DESC desc = { 0 };
	desc.BufferCount = 1;
	desc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	desc.BufferDesc.Width = desc.BufferDesc.Height = 100;
	desc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	desc.SampleDesc.Count = 1;
	desc.Windowed = TRUE;

	WNDCLASS wc = {};
	wc.lpfnWndProc = WindowProc;
	wc.hInstance = nullptr;
	wc.lpszClassName = "Sample Window Class";

	RegisterClass(&wc);
	desc.OutputWindow = CreateWindow("Sample Window Class", 0, WS_OVERLAPPED, 0, 0, 100, 100, 0, 0, 0, 0);
	LOG(INFO) << GetLastError();
	IDXGISwapChain * sc;
	HRESULT r = D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, 0, D3D11_CREATE_DEVICE_DEBUG, force_feature_11_1, 1, D3D11_SDK_VERSION, &desc, &sc, &d, &f, &c);
}

int main() {
	//testd3d();
	//return 0;
	//HLSL::ByteCode s(b, b + sizeof(b));
	std::ifstream fin("/Users/philkr/Documents/v2.bin", std::ifstream::binary);
	fin.seekg(0, std::ifstream::end);
	HLSL::ByteCode s( fin.tellg() );
	fin.seekg(0, std::ifstream::beg);
	fin.read((char*)s.data(), s.size());
	HLSL hs(s);
	std::cout << dis(hs) << std::endl;
	std::cout << std::endl << "-----------" << std::endl << std::endl;

	HLSL subset_hs = hs.subset({ "SV_Position" });
	std::cout << dis(subset_hs) << std::endl;
	std::cin.ignore();
}