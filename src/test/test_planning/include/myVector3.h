#pragma once
#include <iostream>
#include <cmath>

#define PI acos(-1)

class myVector3
{
public:
    float x, y, z;
	static myVector3 XVector;
	static myVector3 YVector;
	static myVector3 ZVector;
	static myVector3 ZeroVector;

	myVector3() :x(0), y(0), z(0) {}
	myVector3(float x1, float y1, float z1) :x(x1), y(y1), z(z1) {}
	myVector3(const myVector3 &v);
	~myVector3();

	void operator=(const myVector3 &v);
	myVector3 operator+(const myVector3 &v);
	myVector3 operator-(const myVector3 &v);
	myVector3 operator/(const myVector3 &v);
	myVector3 operator*(const myVector3 &v);
	myVector3 operator+(float f);
	myVector3 operator-(float f);
	myVector3 operator/(float f);
	myVector3 operator*(float f);
	myVector3 operator+(double d);
	myVector3 operator-(double d);
	myVector3 operator/(double d);
	myVector3 operator*(double d);
	float dot(const myVector3 &v);
	float length();
	myVector3 normalize();
	myVector3 crossProduct(const myVector3 &v);
	double Angle(const myVector3 &v);
	void printVec3();
};

//复制构造函数，必须为常量引用参数，否则编译不通过
 myVector3::myVector3(const myVector3 &v) :x(v.x), y(v.y), z(v.z)
 {
 }
// myVector3::myVector3(const myVector3 &v){
//     x(v.x);
//     y(v.y);
//     z(v.z);
// }

myVector3::~myVector3()
{
}

void myVector3::operator=(const myVector3 &v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}

myVector3 myVector3::operator+(const myVector3 &v)
{
	return myVector3(x + v.x, y + v.y, z + v.z);
}

myVector3 myVector3::operator-(const myVector3 &v)
{
	return myVector3(x - v.x, y - v.y, z - v.z);
}

myVector3 myVector3::operator/(const myVector3 &v)
{
	if (fabsf(v.x) <= 1e-6 || fabsf(v.y) <= 1e-6 || fabsf(v.z) <= 1e-6)
	{
		std::cerr << "Over flow!\n";
		return *this;
	}
	return myVector3(x / v.x, y / v.y, z / v.z);
}

myVector3 myVector3::operator*(const myVector3 &v)
{
	return myVector3(x*v.x, y*v.y, z*v.z);
}

myVector3 myVector3::operator+(float f)
{
	return myVector3(x + f, y + f, z + f);
}

myVector3 myVector3::operator-(float f)
{
	return myVector3(x - f, y - f, z - f);
}

myVector3 myVector3::operator/(float f)
{
	if (fabsf(f) < 1e-6)
	{
		std::cerr << "Over flow!\n";
		return *this;
	}
	return myVector3(x / f, y / f, z / f);
}

myVector3 myVector3::operator*(float f)
{
	return myVector3(x*f, y*f, z*f);
}

myVector3 myVector3::operator+(double d)
{
	return myVector3(x + d, y + d, z + d);
}

myVector3 myVector3::operator-(double d)
{
	return myVector3(x - d, y - d, z - d);
}

myVector3 myVector3::operator/(double d)
{
	if (fabsf(d) < 1e-6)
	{
		std::cerr << "Over flow!\n";
		return *this;
	}
	return myVector3(x / d, y / d, z / d);
}

myVector3 myVector3::operator*(double d)
{
	return myVector3(x*d, y*d, z*d);
}

float myVector3::dot(const myVector3 &v)
{
	return x * v.x + y * v.y + z * v.z;
}

float myVector3::length()
{
	return sqrtf(dot(*this));
}

myVector3 myVector3::normalize()
{
	float len = length();
	if (len < 1e-6) len = 1;
	len = 1 / len;

	x *= len;
	y *= len;
	z *= len;
	return myVector3(x, y, z);
}

/*
Cross Product叉乘公式
aXb = | i,  j,  k  |
	  | a.x a.y a.z|
	  | b.x b.y b.z| = (a.y*b.z -a.z*b.y)i + (a.z*b.x - a.x*b.z)j + (a.x*b.y - a.y*b.x)k
*/
myVector3 myVector3::crossProduct(const myVector3 &v)
{
	return myVector3(y * v.z - z * v.y,
		z * v.x - x * v.z,
		x * v.y - y * v.x);
}

double myVector3::Angle(const myVector3 &v) {
	float xita = (x * v.x + y * v.y + z * v.z) / (sqrt(x * x + y * y + z * z) + sqrt(v.x*v.x + v.y*v.y + v.z*v.z));
	return acos(xita);
}

void myVector3::printVec3()
{
	std::cout << "(" << x << ", " << y << ", " << z << ")" << std::endl;
}