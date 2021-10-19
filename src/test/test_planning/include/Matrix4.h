#pragma once
#include <cmath>
#include <iostream>

class Matrix4
{
public:
	float x, y, z, t;
	//Vector3 x3(), y3(), z3(), t3();
	static Matrix4 XVector;
	static Matrix4 YVector;
	static Matrix4 ZVector;
	static Matrix4 TVector;
	static Matrix4 ZeroVector;

	Matrix4() :x(0), y(0), z(0), t(0) {}
	Matrix4(float x1, float y1, float z1, float t1) :x(x1), y(y1), z(z1), t(t1) {}
	//Matrix4(Vector3 x1, Vector3 y1, Vector3 z1, Vector3 t1) :x3(x1), y3(y1), z3(z1), t3(t1) {}
	Matrix4(const Matrix4 &v);
	~Matrix4();

	void operator=(const Matrix4 &v);
	Matrix4 operator+(const Matrix4 &v);
	Matrix4 operator-(const Matrix4 &v);
	Matrix4 operator/(const Matrix4 &v);
	Matrix4 operator*(const Matrix4 &v);
	Matrix4 operator+(float f);
	Matrix4 operator-(float f);
	Matrix4 operator/(float f);
	Matrix4 operator*(float f);
	float dot(const Matrix4 &v);
	float length();
	void normalize();
	//Matrix4 crossProduct(const Matrix4 &v);
};

//复制构造函数，必须为常量引用参数，否则编译不通过
Matrix4::Matrix4(const Matrix4 &v) :x(v.x), y(v.y), z(v.z), t(v.t)
{
}

Matrix4::~Matrix4()
{
}

void Matrix4::operator=(const Matrix4 &v)
{
	x = v.x;
	y = v.y;
	z = v.z;
	t = v.t;
}

Matrix4 Matrix4::operator+(const Matrix4 &v)
{
	return Matrix4(x + v.x, y + v.y, z + v.z, t + v.t);
}

Matrix4 Matrix4::operator-(const Matrix4 &v)
{
	return Matrix4(x - v.x, y - v.y, z - v.z, t - v.t);
}

Matrix4 Matrix4::operator/(const Matrix4 &v)
{
	if (fabsf(v.x) <= 1e-6 || fabsf(v.y) <= 1e-6 || fabsf(v.z) <= 1e-6 || fabsf(v.t) <= 1e-6)
	{
		std::cerr << "Over flow!\n";
		return *this;
	}
	return Matrix4(x / v.x, y / v.y, z / v.z, t / v.t);
}

Matrix4 Matrix4::operator*(const Matrix4 &v)
{
	return Matrix4(x*v.x, y*v.y, z*v.z, t*v.t);
}

Matrix4 Matrix4::operator+(float f)
{
	return Matrix4(x + f, y + f, z + f, t + f);
}

Matrix4 Matrix4::operator-(float f)
{
	return Matrix4(x - f, y - f, z - f, t - f);
}

Matrix4 Matrix4::operator/(float f)
{
	if (fabsf(f) < 1e-6)
	{
		std::cerr << "Over flow!\n";
		return *this;
	}
	return Matrix4(x / f, y / f, z / f, t / f);
}

Matrix4 Matrix4::operator*(float f)
{
	return Matrix4(x*f, y*f, z*f, t*f);
}

float Matrix4::dot(const Matrix4 &v)
{
	return x * v.x + y * v.y + z * v.z + t * v.t;
}

float Matrix4::length()
{
	return sqrtf(dot(*this));
}

void Matrix4::normalize()
{
	float len = length();
	if (len < 1e-6) len = 1;
	len = 1 / len;

	x *= len;
	y *= len;
	z *= len;
	t *= len;
}

/*
Cross Product叉乘公式
aXb = | i,  j,  k  |
	 | a.x a.y a.z|
	 | b.x b.y b.z| = (a.y*b.z -a.z*b.y)i + (a.z*b.x - a.x*b.z)j + (a.x*b.y - a.y*b.x)k
*/
//Matrix4 Matrix4::crossProduct(const Matrix4 &v)
//{
//	return Matrix4(y * v.z - z * v.y,
//		z * v.x - x * v.z,
//		x * v.y - y * v.x);
//}