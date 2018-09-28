#pragma once
#include <string>
#include <unordered_map>
#include "sdk.h"

struct CBufferVariable {
	struct Location {
		uint32_t bind_point, offset;
	};
	std::string cbuffer_name, variable_name;
	std::vector<size_t> offset_, size_;
	std::unordered_map<ShaderHash, Location> position_hash_;
	CBufferVariable(const std::string & cbuffer_name, const std::string & variable_name, size_t size=0);
	CBufferVariable(const std::string & cbuffer_name, const std::string & variable_name, const std::vector<size_t> & offset, const std::vector<size_t> & size);
	bool scan(std::shared_ptr<Shader> s);
	bool has(const ShaderHash & h);
	
	std::shared_ptr<GPUMemory> fetch(GameController * c, const ShaderHash & h, const std::vector<Buffer> & cbuffers, bool immediate = false) const;
};

bool hasCBuffer(std::shared_ptr<Shader> s, const std::string & name);
bool hasSBuffer(std::shared_ptr<Shader> s, const std::string & name);
bool hasTexture(std::shared_ptr<Shader> s, const std::string & name);
bool hasBuffer(const std::vector<Shader::Buffer> & b, const std::string & name);

struct float4x4 {
	float d[4][4];
	float4x4(float v = 0);
	float4x4 & operator=(float v);
	float4x4 affine_inv();
	operator bool() const;
	float * operator[](size_t i) { return d[i]; }
	const float * operator[](size_t i) const { return d[i]; }
};
std::ostream & operator<<(std::ostream & s, const float4x4 & f);
void mul(float4x4 * out, const float4x4 & a, const float4x4 & b);
void add(float4x4 * out, const float4x4 & a, const float4x4 & b);
void div(float4x4 * out, const float4x4 & a, float b);
struct Vec2f {
	float x, y;
	bool operator==(const Vec2f & o) const {
		return x == o.x && y == o.y;
	}
};
struct Vec3f {
	float x, y, z;
	bool operator==(const Vec3f & o) const {
		return x == o.x && y == o.y && z == o.z;
	}
};
std::string toJSON(const Vec3f & v);

struct Quaternion {
	static Quaternion fromMatrix(const float4x4 & m) {
		int k0, k1, k2, k3;
		float s0, s1, s2;
		if (m[0][0] + m[1][1] + m[2][2] > 0.f) {
			k0 = 3; k1 = 2; k2 = 1; k3 = 0;
			s0 = s1 = s2 = 1.f;
		} else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
			k0 = 0; k1 = 1; k2 = 2; k3 = 3;
			s0 = 1.f; s1 = s2 = -1.f;
		} else if (m[1][1] > m[2][2]) {
			k0 = 1; k1 = 0; k2 = 3; k3 = 2;
			s1 = 1.f; s0 = s2 = -1.f;
		} else {
			k0 = 2; k1 = 3; k2 = 0; k3 = 1;
			s0 = s1 = -1.f; s2 = 1.f;
		}
		float t = s0 * m[0][0] + s1 * m[1][1] + s2 * m[2][2] + 1.f;
		float s = 0.5f / sqrt(t);
		float q[4] = { 0 };
		q[k0] = s * t;
		q[k1] = s * (m[0][1] - s2 * m[1][0]);
		q[k2] = s * (m[2][0] - s1 * m[0][2]);
		q[k3] = s * (m[1][2] - s0 * m[2][1]);
		return { q[0], q[1], q[2], q[3] };
	}
	float x, y, z, w;
	bool operator==(const Quaternion & o) const {
		return x == o.x && y == o.y && z == o.z && w == o.w;
	}
};
inline float D2(const Vec2f & a, const Vec2f & b) {
	return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y);
}
inline float D2(const Vec3f & a, const Vec3f & b) {
	return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z);
}
inline float D2(const Quaternion & a, const Quaternion & b) {
	return 1 - fabs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
}
template<typename T>
class NNSearch3D {
protected:
	float s, r2;
	std::unordered_multimap<uint64_t, std::pair<T, Vec3f> > map;
public:
	NNSearch3D(float radius): s(.5f/radius), r2(radius*radius) {
	}
	void clear() {
		map.clear();
	}
	void insert(const Vec3f & v, const T & d) {
		uint64_t X = uint64_t(s * v.x), Y = uint64_t(s * v.y), Z = uint64_t(s * v.z), M = (1 << 21) - 1;
		uint64_t H = ((X & M) << 42) | ((Y & M) << 21) | (Z & M);
		map.insert({ H, { d, v } });
	}
template<typename F>
	void find(const Vec3f & v, F f) const {
		uint64_t x = uint64_t(s * v.x), y = uint64_t(s * v.y), z = uint64_t(s * v.z), M = (1 << 21) - 1;
		for (uint64_t o = 0; o < 8; o++) {
			uint64_t X = x + (o & 1), Y = y + ((o >> 1) & 1), Z = z + ((o >> 2) & 1);
			uint64_t H = ((X & M) << 42) | ((Y & M) << 21) | (Z & M);
			auto rng = map.equal_range(H);
			for (auto i = rng.first; i != rng.second; i++) {
				const Vec3f & V = i->second.second;
				if (D2(v,V) < r2)
					f(i->second.first);
			}
		}
	}
	std::vector<T> find(const Vec3f & v) const {
		std::vector<T> r;
		find(v, [&r](const T&t) { r.push_back(t); });
		return r;
	}
	void swap(NNSearch3D & o) {
		std::swap(r2, o.r2);
		std::swap(s, o.s);
		map.swap(o.map);
	}
};

std::ostream &operator<<(std::ostream & s, const Vec2f & v);
std::ostream &operator<<(std::ostream & s, const Vec3f & v);
std::ostream &operator<<(std::ostream & s, const Quaternion & v);
template<typename T>
class NNSearch2D {
protected:
	float s, r2;
	std::unordered_multimap<uint64_t, std::pair<T, Vec2f> > map;
public:
	NNSearch2D(float radius) : s(0.5f / radius), r2(radius*radius) {
	}
	void clear() {
		map.clear();
	}
	void insert(const Vec2f & v, const T & d) {
		uint64_t X = uint64_t(s * v.x), Y = uint64_t(s * v.y), M = (1ull << 32) - 1;
		uint64_t H = ((X & M) << 32) | (Y & M);
		map.insert({ H,{ d, v } });
	}
	template<typename F> void find(const Vec2f & v, F f) const {
		uint64_t x = uint64_t(s * v.x - 0.5), y = uint64_t(s * v.y - 0.5), M = (1ull << 32) - 1;
		for (uint64_t o = 0; o < 4; o++) {
			uint64_t X = x + (o & 1), Y = y + ((o >> 1) & 1);
			uint64_t H = ((X & M) << 32) | (Y & M);
			auto rng = map.equal_range(H);
			for (auto i = rng.first; i != rng.second; i++) {
				const Vec2f & V = i->second.second;
				if (D2(v, V) < r2)
					f(i->second.first);
			}
		}
	}
	std::vector<T> find(const Vec2f & v) const {
		std::vector<T> r;
		find(v, [&r](const T&t) { r.push_back(t); });
		return r;
	}
	void swap(NNSearch2D & o) {
		std::swap(r2, o.r2);
		std::swap(s, o.s);
		map.swap(o.map);
	}
};
