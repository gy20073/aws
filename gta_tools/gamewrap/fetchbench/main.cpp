#include <d3d11.h>
#include <Windows.h>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <mutex>
#include <vector>
#include <algorithm>
//
//#pragma comment (lib, "d3d11.lib")
//
//#include "pulse.h"
//
//#define N 1024
//
//ID3D11Device * d;
//ID3D11DeviceContext * c;
//ID3D11Buffer * out_buf, *c_buf, *pulse_cb;
//
//ID3D11VertexShader * pulse_vs;
//ID3D11GeometryShader * pulse_gs;
#define DOUT( s ) { \
   std::wostringstream os_;    \
   os_ << s;                   \
   OutputDebugStringW( os_.str().c_str() );  \
}
#define TIME( n, cmd ) {\
  LARGE_INTEGER t0, t1, freq;\
  QueryPerformanceFrequency(&freq);\
  QueryPerformanceCounter(&t0);\
  cmd;\
  QueryPerformanceCounter(&t1);\
  DOUT(n << 1000.0 * (t1.QuadPart - t0.QuadPart) / freq.QuadPart << " ms" << std::endl);\
}
//
//int setupBuf() {
//	D3D11_BUFFER_DESC desc = { N * sizeof(float), D3D11_USAGE_DEFAULT, D3D11_BIND_STREAM_OUTPUT, 0, 0, 0 };
//	HRESULT hr = d->CreateBuffer(&desc, 0, &out_buf);
//	if (FAILED(hr)) return hr;
//
//	desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
//	hr = d->CreateBuffer(&desc, 0, &c_buf);
//	if (FAILED(hr)) return hr;
//
//	desc.Usage = D3D11_USAGE_DYNAMIC;
//	desc.ByteWidth = 16;
//	desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
//	hr = d->CreateBuffer(&desc, 0, &pulse_cb);
//	if (FAILED(hr)) return hr;
//
//	return 0;
//}
//int setupShader() {
//	HRESULT hr = d->CreateVertexShader(pulse_code, sizeof(pulse_code), 0, &pulse_vs);
//	if (FAILED(hr)) return hr;
//
//	D3D11_SO_DECLARATION_ENTRY pDecl[] =
//	{
//		{ 0, "RESULT", 0, 0, 1, 0 },
//	};
//	hr = d->CreateGeometryShaderWithStreamOutput(pulse_code, sizeof(pulse_code), pDecl, 1, nullptr, 0, D3D11_SO_NO_RASTERIZED_STREAM, nullptr, &pulse_gs);
//	if (FAILED(hr)) return hr;
//
//	return 0;
//}
//int pulse(float o = 0) {
//	c->VSSetShader(pulse_vs, nullptr, 0);
//	if (o) {
//		D3D11_MAPPED_SUBRESOURCE msr;
//		c->Map(pulse_cb, 0, D3D11_MAP_WRITE_DISCARD, 0, &msr);
//		memcpy(msr.pData, &o, sizeof(o));
//		c->Unmap(pulse_cb, 0);
//		c->VSSetConstantBuffers(0, 1, &pulse_cb);
//	}
//	else
//		c->VSSetConstantBuffers(0, 0, nullptr);
//
//	c->IAGetVertexBuffers(0, 0, nullptr, nullptr, nullptr);
//	c->GSSetShader(pulse_gs, nullptr, 0);
//	UINT offset[1] = { 0 };
//	c->SOSetTargets(1, &out_buf, offset);
//
//	c->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_POINTLIST);
//	c->Draw(N, 0);
//
//	return 0;
//}
//int pulseAndCopy(float o = 0) {
//	pulse(o);
//	c->CopyResource(c_buf, out_buf);
//	return 0;
//}
//struct Fetcher {
//	Fetcher() = default;
//	Fetcher(const Fetcher&) = delete;
//	Fetcher& operator=(const Fetcher&) = delete;
//	float values[N] = { 0 };
//	virtual void begin() { memset(values, 0, sizeof(values)); }
//	virtual void end() {}
//	virtual float score(float v) {
//		float r = 0;
//		for (int i = 0; i < N; i++)
//			r += fabs(2*i+1 - values[i]);
//		return r;
//	}
//	virtual void fetch(int i) = 0;
//	virtual void bench(float v) {
//		float r = 0;
//		begin();
//		TIME((typeid(*this).name()+std::string("  fetch ")).c_str(), for (int i = 0; i < N; i++) { pulseAndCopy(i+1.f); fetch(i); });
//		TIME((typeid(*this).name() + std::string("  end   ")).c_str(), end());
//		DOUT("ERR = " << score(v) << std::endl);
//		//DOUT("V = [ " << values[0] << " , " << values[1] << " , " << values[2] << " , " << values[3] << " ]" << std::endl)
//	}
//};
//struct CopySubresource : public Fetcher {
//	ID3D11Buffer * stage;
//	virtual D3D11_BUFFER_DESC desc() { return { N * sizeof(float), D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ, 0, 0 }; }
//	CopySubresource() {
//		D3D11_BUFFER_DESC ds = desc();
//		HRESULT hr = d->CreateBuffer(&ds, 0, &stage);
//		if (FAILED(hr)) DOUT("FAILED to crate staging buffer!" << std::endl);
//	}
//	~CopySubresource() {
//		if (stage) stage->Release();
//	}
//	virtual void fetch(int i) {
//		D3D11_BOX bx = { i * sizeof(float), 0, 0, (i+1)*sizeof(float), 1, 1 };
//		c->CopySubresourceRegion(stage, 0, i * sizeof(float), 0, 0, c_buf, 0, &bx);
//	}
//	virtual void end() {
//		D3D11_MAPPED_SUBRESOURCE msr;
//		c->Map(stage, 0, D3D11_MAP_READ, 0, &msr);
//		if (msr.pData)
//			memcpy(values, msr.pData, sizeof(values));
//		c->Unmap(stage, 0);
//	}
//};
//struct IndividualCopySubresource : public CopySubresource {
//	virtual D3D11_BUFFER_DESC desc() { return { 1 * sizeof(float), D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ, 0, 0 }; }
//	virtual void fetch(int i) {
//		D3D11_BOX bx = { i * sizeof(float), 0, 0, (i + 1) * sizeof(float), 1, 1 };
//		c->CopySubresourceRegion(stage, 0, 0, 0, 0, c_buf, 0, &bx);
//
//		D3D11_MAPPED_SUBRESOURCE msr;
//		c->Map(stage, 0, D3D11_MAP_READ, 0, &msr);
//		if (msr.pData)
//			memcpy(values+i, msr.pData, sizeof(float));
//		c->Unmap(stage, 0);
//	}
//	virtual void end() {
//	}
//};
//struct OpenCL: public Fetcher {
//
//	ID3D11Buffer * stage;
//	virtual D3D11_BUFFER_DESC desc() { return { N * sizeof(float), D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ, 0, 0 }; }
//	OpenCL() {
//		//clGetDeviceIDsFromD3D11KHR(platform, CL_D3D11_DEVICE_KHR, d, CL_PREFERRED_DEVICES_FOR_D3D11_KHR, ...);
//	}
//	OpenCL(const OpenCL&) = delete;
//	OpenCL& operator=(const OpenCL&) = delete;
//	~OpenCL() {
//
//	}
//	virtual void fetch(int i) {
//
//	}
//	virtual void end() {
//
//	}
//};
//int fetch(int i) {
//	return 0;
//}
//
//#define COM_INTERFACE(T) LONG ref_count_ = 1;\
//HRESULT __stdcall QueryInterface(REFIID riid, void **ppObj) {\
//	if (riid == IID_IUnknown) {\
//		*ppObj = static_cast<IUnknown*>(this);\
//		AddRef();\
//		return S_OK;\
//	}\
//	if (riid == __uuidof(T)) {\
//		*ppObj = this;\
//		AddRef();\
//		return S_OK;\
//	}\
//	*ppObj = NULL;\
//	return E_NOINTERFACE;\
//}\
//ULONG __stdcall AddRef() {\
//	return InterlockedIncrement(&ref_count_);\
//}\
//ULONG __stdcall Release() {\
//	long rc = InterlockedDecrement(&ref_count_);\
//	if (rc == 0) delete this;\
//	return rc;\
//}
//
//class __declspec(uuid("05ab9669-5951-45b5-bb0a-0051992e5c9d")) Data : public IUnknown {
//	COM_INTERFACE(Data);
//};

//
//int main_old() {
//	std::mutex mutex;
//	const int M = 100000;
//	std::vector<ID3D11Buffer*> buffer(M);
//	std::vector<Data*> data(M);
//	std::unordered_map<IUnknown*, Data*> mp;
//
//	HRESULT hr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, nullptr, 0, D3D11_SDK_VERSION, &d, nullptr, &c);
//	D3D11_BUFFER_DESC desc = { 16, D3D11_USAGE_DEFAULT, 0 };
//	TIME("Create buffer  ", for (auto & i : buffer) { d->CreateBuffer(&desc, nullptr, &i); });
//	TIME("Create data    ", for (auto & i : data) { i = new Data(); });
//
//	TIME("MAP Insert     ", for (int i = 0; i < data.size(); i++) { mp[buffer[i]] = data[i]; });
//	TIME("MAP Access     ", for (int i = 0; i < data.size(); i++) { data[i] = mp[buffer[i]]; });
//
//
//	TIME("MAP Insert (L) ", for (int i = 0; i < data.size(); i++) { std::lock_guard<std::mutex> lock(mutex); mp[buffer[i]] = data[i]; });
//	TIME("MAP Access (L) ", for (int i = 0; i < data.size(); i++) { std::lock_guard<std::mutex> lock(mutex); data[i] = mp[buffer[i]]; });
//	//TIME("MAP Insert 10 ", for (int it = 0; it < 100; it++) for (int i = 0; i < data.size(); i++) { mp[buffer[i]] = data[i]; });
//	//TIME("MAP Access 10 ", for (int it = 0; it < 100; it++) for (int i = 0; i < data.size(); i++) { std::lock_guard<std::mutex> lock(mutex); data[i] = mp[buffer[i]]; });
//
//	REFGUID uuid = __uuidof(Data);
//	TIME("PRI Insert     ", for (int i = 0; i < data.size(); i++) { buffer[i]->SetPrivateDataInterface(uuid, data[i]); });
//	TIME("PRI Access     ", for (int i = 0; i < data.size(); i++) { IUnknown * d = nullptr; UINT ds = sizeof(d);  buffer[i]->GetPrivateData(uuid, &ds, &d); d->Release(); });
//	//TIME("PRI Insert 10 ", for (int it = 0; it < 100; it++) for (int i = 0; i < data.size(); i++) { buffer[i]->SetPrivateDataInterface(uuid, data[i]); buffer[i]->SetPrivateData(uuid, data[i]); });
//	//TIME("PRI Access 10 ", for (int it = 0; it < 100; it++) for (int i = 0; i < data.size(); i++) { IUnknown * d; UINT ds = sizeof(d);  buffer[i]->GetPrivateData(uuid, &ds, &d); });
//	IUnknown * d = nullptr; UINT ds = sizeof(d); buffer[0]->GetPrivateData(uuid, &ds, &d);
//	//if (FAILED(hr)) return 1;
//
//	//hr = setupBuf();
//	//if (FAILED(hr)) return 2;
//
//	//hr = setupShader();
//	//if (FAILED(hr)) return 3;
//
//	////TIME("single pulse ", pulse(););
//	////TIME("100    pulse ", for (int i = 0; i < 100; i++) pulse(); c->Flush(););
//	////TIME("10000  pulse ", for (int i = 0; i < 10000; i++) pulse(); c->Flush(););
//	////TIME("100000 pulse ", for (int i = 0; i < 100000; i++) pulse(); c->Flush(););
//
//	////TIME("single pulse f ", pulse(1.0););
//	////TIME("100    pulse f ", for (int i = 0; i < 100; i++) pulse(i + 0.01f); c->Flush(););
//	////TIME("10000  pulse f ", for (int i = 0; i < 10000; i++) pulse(i + 0.01f); c->Flush(););
//	////TIME("100000 pulse f ", for (int i = 0; i < 100000; i++) pulse(i + 0.01f); c->Flush(););
//
//	////TIME("single pulse & cp ", pulseAndCopy(););
//	////TIME("100    pulse & cp ", for (int i = 0; i < 100; i++) pulseAndCopy(); c->Flush(););
//	////TIME("10000  pulse & cp ", for (int i = 0; i < 10000; i++) pulseAndCopy(); c->Flush(););
//	////TIME("100000 pulse & cp ", for (int i = 0; i < 100000; i++) pulseAndCopy(); c->Flush(););
//
//
//	//CopySubresource().bench(0.1f);
//	//IndividualCopySubresource().bench(0.1f);
//
//	return 0;
//}


struct Vec2f {
	float x, y;
	bool operator==(const Vec2f & o) const {
		return x == o.x && y == o.y;
	}
	float operator[](size_t i) const {
		return i == 0 ? x : y;
	}
	float & operator[](size_t i) {
		return i == 0 ? x : y;
	}
};

template<typename T>
class NNSearch2D {
protected:
	float s;
	std::unordered_multimap<uint64_t, T> map;
public:
	NNSearch2D(float radius) : s(1.f / radius) {
	}
	void clear() {
		map.clear();
	}
	void insert(const Vec2f & v, const T & d) {
		uint64_t x = uint64_t(s * v.x), y = uint64_t(s * v.y), M = (1ull << 32) - 1;
		for (uint64_t o = 0; o < 4; o++) {
			uint64_t X = x + (o & 1), Y = y + ((o >> 1) & 1);
			uint64_t H = ((X & M) << 32) | (Y & M);
			map.insert({ H, d });
		}
	}
	std::vector<T> find(const Vec2f & v) const {
		uint64_t X = uint64_t(s * v.x), Y = uint64_t(s * v.y), M = (1ull << 32) - 1;
		uint64_t H = ((X & M) << 32) | (Y & M);
		auto rng = map.equal_range(H);
		std::vector<T> r;
		for (auto i = rng.first; i != rng.second; i++)
			r.push_back(i->second);
		return r;
	}
};

template<typename T>
class NNSearch2Db {
protected:
	float s, r2;
	std::unordered_multimap<uint64_t, std::pair<T, Vec2f> > map;
public:
	NNSearch2Db(float radius) : s(0.5f / radius), r2(radius*radius) {
	}
	void clear() {
		map.clear();
	}
	void insert(const Vec2f & v, const T & d) {
		uint64_t X = uint64_t(s * v.x), Y = uint64_t(s * v.y), M = (1ull << 32) - 1;
		uint64_t H = ((X & M) << 32) | (Y & M);
		map.insert({ H, {d, v} });
	}
	std::vector<T> find(const Vec2f & v) const {
		uint64_t x = uint64_t(s * v.x - 0.5), y = uint64_t(s * v.y - 0.5), M = (1ull << 32) - 1;
		std::vector<T> r;
		for (uint64_t o = 0; o < 4; o++) {
			uint64_t X = x + (o & 1), Y = y + ((o >> 1) & 1);
			uint64_t H = ((X & M) << 32) | (Y & M);
			auto rng = map.equal_range(H);
			for (auto i = rng.first; i != rng.second; i++) {
				const Vec2f & V = i->second.second;
				if ((v.x-V.x)*(v.x - V.x) + (v.y - V.y)*(v.y - V.y) < r2)
					r.push_back(i->second.first);
			}
		}
		return r;
	}
};
template<typename T>
class NNSearch2Dc {
protected:
	float s, r2;
	std::unordered_multimap<uint64_t, std::pair<T, Vec2f> > map;
public:
	NNSearch2Dc(float radius) : s(1.f/radius), r2(radius*radius) {
	}
	void clear() {
		map.clear();
	}
	void insert(const Vec2f & v, const T & d) {
		uint64_t X = uint64_t(s * v.x), Y = uint64_t(s * v.y), M = (1ull << 32) - 1;
		uint64_t H = ((X & M) << 32) | (Y & M);
		map.insert({ H,{ d, v } });
	}
	std::vector<T> find(const Vec2f & v) const {
		uint64_t x = uint64_t(s * v.x - 1), y = uint64_t(s * v.y - 1), M = (1ull << 32) - 1;
		std::vector<T> r;
		for (uint64_t o = 0; o < 9; o++) {
			uint64_t X = x + (o % 3), Y = y + (o / 3);
			uint64_t H = ((X & M) << 32) | (Y & M);
			auto rng = map.equal_range(H);
			for (auto i = rng.first; i != rng.second; i++) {
				const Vec2f & V = i->second.second;
				if ((v.x - V.x)*(v.x - V.x) + (v.y - V.y)*(v.y - V.y) < r2)
					r.push_back(i->second.first);
			}
		}
		return r;
	}
};
template<typename T>
class KDTree {
protected:
	struct Node {
		Vec2f p;
		T d;
		uint32_t c = 0;
		uint8_t dm = 0;
	};
	float r2;
	std::vector<Node> nodes;
public:
	KDTree(float radius) :r2(radius*radius) {
	}
	void clear() {
		nodes.clear();
	}
	void insert(const Vec2f & v, const T & d) {
		nodes.push_back({ v, d, 0 });
	}
	void build_range(typename std::vector<Node>::iterator start, typename std::vector<Node>::iterator end, int dim = 0) {
		uint32_t n = end - start;
		if (n == 1) {
			start->c = 0;
		}
		else {
			// Run quick select to find the split
			std::nth_element(start, start + n / 2, end, [&dim](const Node & a, const Node & b) { return a.p[dim] < b.p[dim]; });
			std::swap(*start, *(start + n / 2));
			start->c = (n + 1) / 2;
			start->dm = dim;
			if (start->c > 1) build_range(start + 1, start + start->c);
			if (start->c < n) build_range(start + start->c, end);
		}
	}
	void build() {
		build_range(nodes.begin(), nodes.end(), 0);
	}
	void findr(const Vec2f & v, uint32_t i, std::vector<T> & r) const {
		for (; i < nodes.size();) {
			if ((nodes[i].p.x - v.x)*(nodes[i].p.x - v.x) + (nodes[i].p.y - v.y)*(nodes[i].p.y - v.y) < r2)
				r.push_back(nodes[i].d);
			if (!nodes[i].c)
				break;
			float d = v[nodes[i].dm] - nodes[i].p[nodes[i].dm];
			if (d < 0) {
				if (d*d < r2 && nodes[i].c > 1)
					findr(v, i + nodes[i].c, r);
				i++;
			}
			else {
				if (d*d < r2 && nodes[i].c > 1)
					findr(v, i + 1, r);
				i += nodes[i].c;
			}
		}
	}
	std::vector<T> find(const Vec2f & v) const {
		std::vector<T> r;
		findr(v, 0, r);
		return r;
	}
};

template<typename T>
class Brute {
protected:
	struct Node {
		Vec2f p;
		T d;
		uint32_t c = 0;
		uint8_t dm = 0;
	};
	float r2;
	std::vector<Node> nodes;
public:
	Brute(float radius) :r2(radius*radius) {
	}
	void clear() {
		nodes.clear();
	}
	void insert(const Vec2f & v, const T & d) {
		nodes.push_back({ v, d, 0 });
	}
	std::vector<T> find(const Vec2f & v) const {
		std::vector<T> r;
		for (uint32_t i = 0; i < nodes.size(); i++)
			if ((nodes[i].p.x - v.x)*(nodes[i].p.x - v.x) + (nodes[i].p.y - v.y)*(nodes[i].p.y - v.y) < r2)
				r.push_back(nodes[i].d);
		return r;
	}
};


int main() {
	std::vector<Vec2f> d, q;
	srand(0);
	float r = 0.01f;
	for (int i = 0; i < 1000; i++) d.push_back({ 1.f*rand() / RAND_MAX, 1.f*rand() / RAND_MAX });
	for (int i = 0; i < 10000; i++) q.push_back({ 1.f*rand() / RAND_MAX, 1.f*rand() / RAND_MAX });
	{
		NNSearch2Dc<int> s(r);
		TIME("MAP Insert ", for (int i = 0; i < d.size(); i++) s.insert(d[i], i););
		size_t ts = 0;
		TIME("MAP Query  ", for (int i = 0; i < q.size(); i++) ts += s.find(q[i]).size(););
		DOUT("Avg Q size = " << 1. * ts / q.size() << std::endl << std::endl);
	}
	{
		NNSearch2Db<int> s(r);
		TIME("RMAP Insert ", for (int i = 0; i < d.size(); i++) s.insert(d[i], i););
		size_t ts = 0;
		TIME("RMAP Query  ", for (int i = 0; i < q.size(); i++) ts += s.find(q[i]).size(););
		DOUT("Avg Q size = " << 1. * ts / q.size() << std::endl <<std::endl);
	}
	{
		KDTree<int> s(r);
		TIME("KDT Insert ", for (int i = 0; i < d.size(); i++) s.insert(d[i], i););
		TIME("KDT Build ", s.build(););
		size_t ts = 0;
		// ts = q.size() * s.find(q[0]).size();
		TIME("KDT Query  ", for (int i = 0; i < q.size(); i++) ts += s.find(q[i]).size(););
		DOUT("Avg Q size = " << 1. * ts / q.size() << std::endl << std::endl);
	}
	{
		Brute<int> s(r);
		TIME("BRT Insert ", for (int i = 0; i < d.size(); i++) s.insert(d[i], i););
		size_t ts = 0;
		// ts = q.size() * s.find(q[0]).size();
		TIME("BRT Query  ", for (int i = 0; i < q.size(); i++) ts += s.find(q[i]).size(););
		DOUT("Avg Q size = " << 1. * ts / q.size() << std::endl << std::endl);
	}
}
