#pragma once
#include "myVector3.h"
#include "Matrix4.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>

#define PI acos(-1)
#define EPSILON 0.000001

using namespace std;

//三角面片数据
struct TriSurface {
public:
	bool flag;
	int number;
	double area;
	myVector3 normal;
	myVector3 center;
	vector<myVector3> vertex;
};

struct DividePart {
	vector<TriSurface> Part;
};

struct ViewPoint {
public:
	myVector3 position;
	myVector3 direction;
};

class DataForm
{
public:
	int numTriangles;
	
	vector<TriSurface> ReadFile(const char *cfilename);  //读取STL文件
	vector<TriSurface> TriSurfaces = vector<TriSurface>();  //用于读取每个面片的表面信息
	vector<TriSurface> points = vector<TriSurface>();  //用于存储每个面片的表面信息

	vector<vector<ViewPoint>> CalcViewPoint(vector<TriSurface> TriSurfaces);  //生成视点
	vector<myVector3> BuildVolume(vector<TriSurface> TriSurfaces);  //输入所有三角片，建立包围块，返回小立方体中心
	vector<DividePart> DivideModel(vector<TriSurface> TriSurfaces);  //输入所有三角片，建立包围块，返回划分好的包围块面片
	vector<ViewPoint> ViewPointScore(vector<TriSurface> TriSurfaces, vector<TriSurface> resPart, myVector3 center);
	int DirectionDivide(myVector3 direction);
	double AngleDivideScore(myVector3 A, myVector3 B);  //方向评价分类讨论
	bool isVisible(myVector3 center,myVector3 direction,TriSurface TriSurface);  //判断可见性

	double Angle2Rad(double angle);
	double Rad2Angle(double radian);

	void ReadASCII(const char *cfilename);
	void ReadBinary(const char *cfilename);

private:
	double FovX = 100, FovY = 100, FovZ = 100;  //相机视场，单位mm
	double camRadius = 350;  //mm

	/*double Rad2Angle(double radian);
	double Angle2Rad(double angle);*/

	char* memWriter;
	int cpyint(const char*& p);
	float cpyfloat(const char*& p);
};

vector<TriSurface> DataForm::ReadFile(const char *cfilename) {
	FILE * pFile;
	long lSize;
	char* buffer;
	size_t res;

	//采用二进制打开文件，保证文件全部读入
	pFile = fopen(cfilename, "rb");
	if (pFile == NULL) {
		fputs("File error", stderr);
		exit(1);
	}

	//读取文件大小
	fseek(pFile, 0, SEEK_END);
	lSize = ftell(pFile);
	rewind(pFile); //清除缓冲区

	//分配内存存储整个文件
	buffer = (char*)malloc(sizeof(char)*lSize);

	//将文件拷贝到buffer中
	res = fread(buffer, 1, lSize, pFile);
	if (res != lSize) {
		fputs("Reading error", stderr);
		exit(3);
	}

	//结束演示，关闭文件并释放内存
	fclose(pFile);

	//判断文件格式并读取数据
	ios::sync_with_stdio(false);
	stringstream judge(buffer);
	string name;
	judge >> name;
	if (name == "solid") {
		ReadASCII(buffer);
	}
	else {
		ReadBinary(buffer);
	}
	ios::sync_with_stdio(true);

	free(buffer);
	return TriSurfaces;
}

void DataForm::ReadASCII(const char* buffer) {
	numTriangles = 0;
	float x, y, z;
	int  i;
	int number = 0;
	string name, useless;
	stringstream ss(buffer);
	TriSurface surface = TriSurface();

	ss >> name >> name;
	ss.get();
	do {
		ss >> useless;
		if (useless != "facet")
			break;
		//在第一行中获取法向量
		ss >> useless >> x >> y >> z;
		surface.normal.x = x;
		surface.normal.y = y;
		surface.normal.z = z;

		//删去第二行
		getline(ss, useless);
		for (i = 0; i < 3; i++)
		{
			ss >> useless >> x >> y >> z;
			surface.vertex[i].x = x;
			surface.vertex[i].y = y;
			surface.vertex[i].z = z;
		}
		numTriangles++;
		surface.number = ++number;
		//计算三角形面积
		myVector3 AB(surface.vertex[2].x - surface.vertex[0].x, surface.vertex[2].y - surface.vertex[0].y, surface.vertex[2].z - surface.vertex[0].z);
		myVector3 AC(surface.vertex[1].x - surface.vertex[0].x, surface.vertex[1].y - surface.vertex[0].y, surface.vertex[1].z - surface.vertex[0].z);
		surface.area = (AB.crossProduct(AC)).length() / 2;

		//面片中心提取
		surface.center.x = (surface.vertex[0].x + surface.vertex[1].x + surface.vertex[2].x) / 3;
		surface.center.y = (surface.vertex[0].y + surface.vertex[1].y + surface.vertex[2].y) / 3;
		surface.center.z = (surface.vertex[0].z + surface.vertex[1].z + surface.vertex[2].z) / 3;

		surface.flag = false;
		TriSurfaces.push_back(surface);

		getline(ss, useless);
		getline(ss, useless);
		//getline(ss, useless);
	} while (1);
	//return true;
}

void DataForm::ReadBinary(const char* buffer) {

	cout << "开始读取二进制模型" << endl;
	const char* p = buffer;
	char name[80];
	int i, j;
	
	memcpy(name, p, 80);
	p += 80;
	numTriangles = cpyint(p);  //三角面片数量
	int number = 0;  //三角面片序号
	for (i = 0; i < numTriangles; i++) {
		TriSurface surface = TriSurface();
		//存储面片法向量
		surface.normal.x = cpyfloat(p);
		surface.normal.y = cpyfloat(p);
		surface.normal.z = cpyfloat(p);	
		for (j = 0; j < 3; j++) {
			//存储面片
			myVector3 tmp;
			tmp.x = cpyfloat(p);
			tmp.y = cpyfloat(p);
			tmp.z = cpyfloat(p);
			surface.vertex.push_back(tmp);
			//cout << tmp.y << " ";
		}
		surface.number = ++number;
		//cout << number << endl;

		//计算三角形面积
		myVector3 AB(surface.vertex[2].x - surface.vertex[0].x, surface.vertex[2].y - surface.vertex[0].y, surface.vertex[2].z - surface.vertex[0].z);
		myVector3 AC(surface.vertex[1].x - surface.vertex[0].x, surface.vertex[1].y - surface.vertex[0].y, surface.vertex[1].z - surface.vertex[0].z);
		surface.area = (AB.crossProduct(AC)).length() / 2;

		//面片中心提取
		//cout << i << " " << surface.vertex[0].y << " " << surface.vertex[1].y << " " << surface.vertex[2].y << endl;
		surface.center.x = (surface.vertex[0].x + surface.vertex[1].x + surface.vertex[2].x) / 3;
		surface.center.y = (surface.vertex[0].y + surface.vertex[1].y + surface.vertex[2].y) / 3;
		surface.center.z = (surface.vertex[0].z + surface.vertex[1].z + surface.vertex[2].z) / 3;
		//cout << surface.center.y << " ";
		
		surface.flag = false;
		TriSurfaces.push_back(surface);

		p += 2; //跳过尾部标志
	}
	//cout << endl;
	cout << " 模型读取完成，共读取了"<<numTriangles<<"个面片"<< endl;
}

vector<vector<ViewPoint>> DataForm::CalcViewPoint(vector<TriSurface> TriSurfaces) {
	cout << endl;
	cout << "开始计算视点" << endl;

	int divide = 5;//扫描仪最佳测量角度间距
	double radius = 0.01;
	double divideRad = Angle2Rad(divide);
	int up = 0, down = 180;
	
	vector<myVector3> center = BuildVolume(TriSurfaces);  //每个分区的中心
	vector<DividePart> res = DivideModel(TriSurfaces);  //每个分区的面片

	myVector3 ModelCenter(0, 0, 0);
	//vector<myVector3> Direction(180 / divide + 1);
	//vector<myVector3> Position(180 / divide + 1);
	//vector<vector<myVector3>> Pose(180 / divide + 1); 
	//bool flag[180 / 5 + 1];
	//myVector3 table(0.7, 0, 0.7);  //怎么赋值？

	//计算模型中心
	for (int i = 0; i < res.size(); i++) {
		ModelCenter = ModelCenter + center[i];
	}
	ModelCenter = ModelCenter / (double)res.size();

	//for (int xita = 0; xita <= 180 / divide; xita++) {
	//	Direction[xita] = myVector3(radius*sin(divideRad*xita), 0, -radius * cos(divideRad*xita));
	//	Position[xita] = myVector3(Direction[xita].x + ModelCenter.x, 0, Direction[xita].z + ModelCenter.z);
	//	Pose[xita] = vector<myVector3>{ myVector3(0, 1, 0).crossProduct(Direction[xita].normalize()), myVector3(0, 1, 0), Direction[xita].normalize(), table };
		//判断机器人可达性，缺少功能包，尝试在ROS实现
		/*flag[xita] = JudgeRobot(Pose[xita]);
		if (xita >= 1)
		{
			if (flag[xita - 1] == false && flag[xita] == true)
			{
				up = xita * divide;
				down = 180 - up;
				break;
			}
		}*/
	//}
	//cout << "模型中心点坐标为(" << ModelCenter.x << ", " << ModelCenter.y << ", " << ModelCenter.z << ")mm，模型将被划分成" << res.size() << "个测量空间。" << endl;

	//给每个视点评分
	vector<vector<ViewPoint>> result;
	for (int i = 0; i < res.size(); i++) {
		//cout << res.size() << " " << res[i].Part.size() << endl;
		if (res[i].Part.size() != 0) {
			cout << endl;
			cout <<"对第"<< i+1 << "个分区的视点评分开始" << endl;
			vector<ViewPoint> temp;
			temp = ViewPointScore(TriSurfaces, res[i].Part, center[i]);
			result.push_back(temp);
		}
	}
	return result;
}

vector<myVector3> DataForm::BuildVolume(vector<TriSurface> TriSurfaces) {
	cout << "BuildVolume开始，面片数量"<< numTriangles << endl;

	double Xmax, Xmin, Ymax, Ymin, Zmax, Zmin, Xorg, Yorg, Zorg;
	double Xlength, Ylength, Zlength;  //单位mm
	int i = 0, j = 0, k = 0, q = 0, Xcount = 0, Ycount = 0, Zcount = 0;

	myVector3 BoxSize;

	Xmax = TriSurfaces[0].center.x;
	Xmin = Xmax;
	Ymax = TriSurfaces[0].center.y;
	Ymin = Ymax;
	Zmax = TriSurfaces[0].center.z;
	Zmin = Zmax;
	//cout << Xmax << " " << Ymax << " " << Zmax << endl;

	int flag[6] = { 0,0,0,0,0,0 };

	//判断包围盒的边界，并用flag依次记录位置
	for (i; i < numTriangles; i++) {
		if (Xmax < TriSurfaces[i].center.x)
		{
			Xmax = TriSurfaces[i].center.x;
			flag[0] = i;
		}
		if (Xmin > TriSurfaces[i].center.x)
		{
			Xmin = TriSurfaces[i].center.x;
			flag[1] = i;
		}
		if (Ymax < TriSurfaces[i].center.y)
		{
			Ymax = TriSurfaces[i].center.y;
			flag[2] = i;
		}
		if (Ymin > TriSurfaces[i].center.y)
		{
			Ymin = TriSurfaces[i].center.y;
			flag[3] = i;
		}
		if (Zmax < TriSurfaces[i].center.z)
		{
			Zmax = TriSurfaces[i].center.z;
			flag[4] = i;
		}
		if (Zmin > TriSurfaces[i].center.z)
		{
			Zmin = TriSurfaces[i].center.z;
			flag[5] = i;
		}
	}
	for (i = 0; i < 3; i++)
	{
		Xmax = Xmax > TriSurfaces[flag[0]].vertex[i].x ? Xmax : TriSurfaces[flag[0]].vertex[i].x;
		Xmin = Xmin < TriSurfaces[flag[1]].vertex[i].x ? Xmin : TriSurfaces[flag[1]].vertex[i].x;
		Ymax = Ymax > TriSurfaces[flag[2]].vertex[i].y ? Ymax : TriSurfaces[flag[2]].vertex[i].y;
		Ymin = Ymin < TriSurfaces[flag[3]].vertex[i].y ? Ymin : TriSurfaces[flag[3]].vertex[i].y;
		Zmax = Zmax > TriSurfaces[flag[4]].vertex[i].z ? Zmax : TriSurfaces[flag[4]].vertex[i].z;
		Zmin = Zmin < TriSurfaces[flag[5]].vertex[i].z ? Zmin : TriSurfaces[flag[5]].vertex[i].z;
	}
	Xmax++; Ymax++; Zmax++; Xmin--; Ymin--; Zmin--;  //将包围盒四周向外延伸，防止点在包围盒外边界
	Xlength = Xmax - Xmin;
	Ylength = Ymax - Ymin;
	Zlength = Zmax - Zmin;
	cout << " 大包围盒边界结果：X" << flag[0] << " X" << flag[1] << " 间距" << Xlength;
	cout << "；Y" << flag[2] << " Y" << flag[3] << " 间距" << Ylength;
	cout << "；Z" << flag[4] << " Z" << flag[5] << " 间距" << Zlength << endl;

	//划分包围盒
	Xcount = ceil(Xlength / FovX);
	Ycount = ceil(Ylength / FovY);
	Zcount = ceil(Zlength / FovZ);
	Xlength = Xlength / Xcount;
	Ylength = Ylength / Ycount;
	Zlength = Zlength / Zcount;
	BoxSize = myVector3(Xlength, Ylength, Zlength);
	cout << " 子包围盒划分结果：" << endl;
	cout << "  Xcount=" << Xcount << "; Ycount=" << Ycount << "; Zcount=" << Zcount << endl;
	cout << "  Xlength=" << Xlength << "; Ylength=" << Ylength << "; Zlength=" << Zlength << endl;

	//获取每个子包围盒中心
	int numbox = 0;  //记录子包围盒的数量
	vector<myVector3> PartCenter(numbox);
	for (j = 0; j < Xcount; j++)
	{
		Xorg = Xmin + j * Xlength;
		for (k = 0; k < Ycount; k++)
		{
			Yorg = Ymin + k * Ylength;
			for (q = 0; q < Zcount; q++)
			{
				Zorg = Zmin + q * Zlength;
				PartCenter.push_back(myVector3(Xorg + Xlength / 2, Yorg + Ylength / 2, Zorg + Zlength / 2));
				numbox++;
			}
		}
	}
	cout << "BuildVolume模型划分完成，共获得" << numbox << "个子包围盒" << endl;
	return PartCenter;
}

vector<DividePart> DataForm::DivideModel(vector<TriSurface> TriSurfaces) {
	cout << endl;
	cout << "DivideModel开始" << endl;

	double Xmax, Xmin, Ymax, Ymin, Zmax, Zmin, Xorg, Yorg, Zorg;
	double Xscale, Yscale, Zscale;
	double Xlength, Ylength, Zlength;
	int i = 0, j = 0, k = 0, q = 0, Xcount = 0, Ycount = 0, Zcount = 0;

	vector<DividePart> DivideParts;
	myVector3 BoxSize;

	Xmax = TriSurfaces[0].center.x;
	Xmin = Xmax;
	Ymax = TriSurfaces[0].center.y;
	Ymin = Ymax;
	Zmax = TriSurfaces[0].center.z;
	Zmin = Zmax;

	int flag[6] = { 0,0,0,0,0,0 };

	//判断包围盒的边界，并用flag依次记录位置
	for (i; i < numTriangles; i++) {
		if (Xmax < TriSurfaces[i].center.x)
		{
			Xmax = TriSurfaces[i].center.x;
			flag[0] = i;
		}
		if (Xmin > TriSurfaces[i].center.x)
		{
			Xmin = TriSurfaces[i].center.x;
			flag[1] = i;
		}
		if (Ymax < TriSurfaces[i].center.y)
		{
			Ymax = TriSurfaces[i].center.y;
			flag[2] = i;
		}
		if (Ymin > TriSurfaces[i].center.y)
		{
			Ymin = TriSurfaces[i].center.y;
			flag[3] = i;
		}
		if (Zmax < TriSurfaces[i].center.z)
		{
			Zmax = TriSurfaces[i].center.z;
			flag[4] = i;
		}
		if (Zmin > TriSurfaces[i].center.z)
		{
			Zmin = TriSurfaces[i].center.z;
			flag[5] = i;
		}
	}
	for (i = 0; i < 3; i++)
	{
		Xmax = Xmax > TriSurfaces[flag[0]].vertex[i].x ? Xmax : TriSurfaces[flag[0]].vertex[i].x;
		Xmin = Xmin < TriSurfaces[flag[1]].vertex[i].x ? Xmin : TriSurfaces[flag[1]].vertex[i].x;
		Ymax = Ymax > TriSurfaces[flag[2]].vertex[i].y ? Ymax : TriSurfaces[flag[2]].vertex[i].y;
		Ymin = Ymin < TriSurfaces[flag[3]].vertex[i].y ? Ymin : TriSurfaces[flag[3]].vertex[i].y;
		Zmax = Zmax > TriSurfaces[flag[4]].vertex[i].z ? Zmax : TriSurfaces[flag[4]].vertex[i].z;
		Zmin = Zmin < TriSurfaces[flag[5]].vertex[i].z ? Zmin : TriSurfaces[flag[5]].vertex[i].z;
	}
	Xmax++; Ymax++; Zmax++; Xmin--; Ymin--; Zmin--;  //将包围盒四周向外延伸，防止点在包围盒外边界
	Xlength = Xmax - Xmin;
	Ylength = Ymax - Ymin;
	Zlength = Zmax - Zmin;

	//划分包围盒
	Xcount = ceil(Xlength / FovX);
	Ycount = ceil(Ylength / FovY);
	Zcount = ceil(Zlength / FovZ);
	Xlength = Xlength / Xcount;
	Ylength = Ylength / Ycount;
	Zlength = Zlength / Zcount;
	BoxSize = myVector3(Xlength, Ylength, Zlength);

	//获取每个子包围盒面片	
	for (j = 0; j < Xcount; j++)
	{
		Xorg = Xmin + j * Xlength;
		Xscale = Xorg + Xlength;
		for (k = 0; k < Ycount; k++)
		{
			Yorg = Ymin + k * Ylength;
			Yscale = Yorg + Ylength;
			for (q = 0; q < Zcount; q++)
			{
				Zorg = Zmin + q * Zlength;
				Zscale = Zorg + Zlength;
				DividePart DivideArea = DividePart();
				for (i = 0; i < numTriangles; i++) {
					//cout << TriSurfaces[i].center.x << " " << TriSurfaces[i].center.y << " " << TriSurfaces[i].center.z << endl;
					if (TriSurfaces[i].center.x>=Xorg && TriSurfaces[i].center.x < Xscale &&
						TriSurfaces[i].center.y>=Yorg && TriSurfaces[i].center.y < Yscale &&
						TriSurfaces[i].center.z>=Zorg && TriSurfaces[i].center.z < Zscale) {
						DivideArea.Part.push_back(TriSurfaces[i]);
					}
				}
				DivideParts.push_back(DivideArea);
				cout << " 第" << DivideParts.size() << "个子包围盒中有" << DivideArea.Part.size() << "个面片" << endl;
			}
		}
	}
	cout << "DivideModel模型划分完成，共获得" << DivideParts.size() << "个分区的面片" << endl;
	return DivideParts;
}

vector<ViewPoint> DataForm::ViewPointScore(vector<TriSurface> TriSurfaces, vector<TriSurface> resPart, myVector3 center) {
	double weightv = 7, weightd = 3;
	//int number = resPart.size();
	//cout << number << endl;
	vector<TriSurface> Part0, Part1, Part2, Part3, Part4, Part5;
	vector<myVector3> weightNormal(6, myVector3(0,0,0));  //视点区域加权法线方向
	int numtri[6] = { 0,0,0,0,0,0 };  //记录每个方向上的视点
	//按照特征分区
	for (int i = 0; i < resPart.size(); i++) {
		myVector3 temp(resPart[i].normal.x, resPart[i].normal.y, resPart[i].normal.z);
		switch (DirectionDivide(temp))
		{
		case 0:
			Part0.push_back(resPart[i]);
			weightNormal[0] = weightNormal[0] + temp.normalize() * resPart[i].area;
			numtri[0]++;
			break;
		case 1:
			Part1.push_back(resPart[i]);
			weightNormal[1] = weightNormal[1] + temp.normalize() * resPart[i].area;
			numtri[1]++;
			break;
		case 2:
			Part2.push_back(resPart[i]);
			weightNormal[2] = weightNormal[2] + temp.normalize() * resPart[i].area;
			numtri[2]++;
			break;
		case 3:
			Part3.push_back(resPart[i]);
			weightNormal[3] = weightNormal[3] + temp.normalize() * resPart[i].area;
			numtri[3]++;
			break;
		case 4:
			Part4.push_back(resPart[i]);
			weightNormal[4] = weightNormal[4] + temp.normalize() * resPart[i].area;
			numtri[4]++;
			break;
		case 5:
			Part5.push_back(resPart[i]);
			weightNormal[5] = weightNormal[5] + temp.normalize() * resPart[i].area;
			numtri[5]++;
			break;
		default:
			break;
		}
	}
	//for (int i = 0; i < 6; i++) {
	//	cout << weightNormal[i].normalize().x << ", " << weightNormal[i].normalize().y << ", " << weightNormal[i].normalize().z << endl;
	//}
	vector<vector<TriSurface>> Part{ Part0,Part1,Part2,Part3,Part4,Part5 };

	//计算分区三角面片面积
	double allarea[6] = { 0,0,0,0,0,0 };  //每个方向的面积
	double allmodelarea = 0;  //总面积
	for (int p = 0; p < 6; p++) {
		for (int i = 0; i < numtri[p]; i++) {
			allarea[p] = allarea[p] + Part[p][i].area;
		}
		allmodelarea += allarea[p];
	}
	//除去面积过小的面片
	bool enoughArea[6];
	for (int p = 0; p < 6; p++) {
		if (allarea[p] < allmodelarea / 20 || allmodelarea == 0)
			enoughArea[p] = 0;
		else 
			enoughArea[p] = 1;
	}

	int divide = 30;
	double divideRad = Angle2Rad(divide);
	double maxZ = camRadius;
	double minZ = camRadius * cos(Angle2Rad(45));
	int numVP = (360 / divide)*(90 / divide) + 1;  //每个分区的侯选视点数量
	
	//计算每个分区的候选视点位置与方向,并对法线方向和可见度评分
	vector<ViewPoint> viewpoint0(numVP), viewpoint1(numVP), viewpoint2(numVP), viewpoint3(numVP), viewpoint4(numVP), viewpoint5(numVP);
	vector<vector<ViewPoint>> candidateVP{ viewpoint0, viewpoint1, viewpoint2, viewpoint3, viewpoint4, viewpoint5 };
	vector<vector<double>> normalScore(6, vector<double>(numVP, 0));  //法线评分
	vector<vector<double>> visiableArea(6, vector<double>(numVP, 0));  //可见面积
	vector<vector<double>> visiableScore(6, vector<double>(numVP, 0));  //可见度评分
	vector<vector<double>> viewpointScore(6, vector<double>(numVP, 0));  //法线+可见度加权评分
	vector<double> maxScore(6, 0);  //记录最高分
	vector<int> maxVP(6, 0);  //记录最高分视点的位置
	int colCount = 0;
	bool colCheck = 0;
	int n = 1;
	
	//计算6个方向上每个视点的方向与位置
	if (enoughArea[0] == 1) {
		candidateVP[0][0].direction = myVector3(0, 0, camRadius);
		candidateVP[0][0].position = myVector3(candidateVP[0][0].direction.x + center.x, candidateVP[0][0].direction.y + center.y, candidateVP[0][0].direction.z + center.z);
		for (int xita = 1; xita <= 90 / divide; xita++) {
			for (int fai = 0; fai < 360 / divide; fai++) {
				ViewPoint cvp;
				cvp.direction = myVector3(-camRadius * sin(divideRad * xita) * cos(divideRad * fai), camRadius * sin(divideRad * xita) * sin(divideRad * fai), camRadius * cos(divideRad * xita));
				cvp.position = myVector3(cvp.direction.x + center.x, cvp.direction.y + center.y, cvp.direction.z + center.z);
				candidateVP[0][n++]=cvp;
			}
		}
		n = 1;
	}
	if (enoughArea[1] == 1) {
		candidateVP[1][0].direction = myVector3(camRadius, 0, 0);
		candidateVP[1][0].position = myVector3(candidateVP[1][0].direction.x + center.x, candidateVP[1][0].direction.y + center.y, candidateVP[1][0].direction.z + center.z);
		for (int xita = 1; xita <= 90 / divide; xita++) {
			for (int fai = 0; fai < 360 / divide; fai++) {
				ViewPoint cvp;
				cvp.direction = myVector3(camRadius * cos(divideRad * xita), camRadius * sin(divideRad * xita) * cos(divideRad * fai), camRadius * sin(divideRad * xita) * sin(divideRad * fai));
				cvp.position = myVector3(cvp.direction.x + center.x, cvp.direction.y + center.y, cvp.direction.z + center.z);
				candidateVP[1][n++] = cvp;
			}
		}
		n = 1;
	}
	if (enoughArea[2] == 1) {
		candidateVP[2][0].direction = myVector3(0, camRadius, 0);
		candidateVP[2][0].position = myVector3(candidateVP[2][0].direction.x + center.x, candidateVP[2][0].direction.y + center.y, candidateVP[2][0].direction.z + center.z);
		for (int xita = 1; xita <= 90 / divide; xita++) {
			for (int fai = 0; fai < 360 / divide; fai++) {
				ViewPoint cvp;
				cvp.direction = myVector3(camRadius * sin(divideRad * xita) * cos(divideRad * fai), camRadius * cos(divideRad * xita), camRadius * sin(divideRad * xita) * sin(divideRad * fai));
				cvp.position = myVector3(cvp.direction.x + center.x, cvp.direction.y + center.y, cvp.direction.z + center.z);
				candidateVP[2][n++] = cvp;
			}
		}
		n = 1;
	}
	if (enoughArea[3] == 1) {
		candidateVP[3][0].direction = myVector3(-camRadius, 0, 0);
		candidateVP[3][0].position = myVector3(candidateVP[3][0].direction.x + center.x, candidateVP[3][0].direction.y + center.y, candidateVP[3][0].direction.z + center.z);
		for (int xita = 1; xita <= 90 / divide; xita++) {
			for (int fai = 0; fai < 360 / divide; fai++) {
				ViewPoint cvp;
				cvp.direction = myVector3(-camRadius * cos(divideRad * xita), camRadius * sin(divideRad * xita) * cos(divideRad * fai), camRadius * sin(divideRad * xita) * sin(divideRad * fai));
				cvp.position = myVector3(cvp.direction.x + center.x, cvp.direction.y + center.y, cvp.direction.z + center.z);
				candidateVP[3][n++] = cvp;
			}
		}
		n = 1;
	}
	if (enoughArea[4] == 1) {
		candidateVP[4][0].direction = myVector3(0, -camRadius, 0);
		candidateVP[4][0].position = myVector3(candidateVP[4][0].direction.x + center.x, candidateVP[4][0].direction.y + center.y, candidateVP[4][0].direction.z + center.z);
		for (int xita = 1; xita <= 90 / divide; xita++) {
			for (int fai = 0; fai < 360 / divide; fai++) {
				ViewPoint cvp;
				cvp.direction = myVector3(camRadius * sin(divideRad * xita) * cos(divideRad * fai), -camRadius * cos(divideRad * xita), camRadius * sin(divideRad * xita) * sin(divideRad * fai));
				cvp.position = myVector3(cvp.direction.x + center.x, cvp.direction.y + center.y, cvp.direction.z + center.z);
				candidateVP[4][n++] = cvp;
			}
		}
		n = 1;
	}
	if (enoughArea[5] == 1) {
		candidateVP[5][0].direction = myVector3(0, 0, -camRadius);
		candidateVP[5][0].position = myVector3(candidateVP[5][0].direction.x + center.x, candidateVP[5][0].direction.y + center.y, candidateVP[5][0].direction.z + center.z);
		for (int xita = 1; xita <= 90 / divide; xita++) {
			for (int fai = 0; fai < 360 / divide; fai++) {
				ViewPoint cvp;
				cvp.direction = myVector3(-camRadius * sin(divideRad * xita) * cos(divideRad * fai), camRadius * sin(divideRad * xita) * sin(divideRad * fai), -camRadius * cos(divideRad * xita));
				cvp.position = myVector3(cvp.direction.x + center.x, cvp.direction.y + center.y, cvp.direction.z + center.z);
				candidateVP[5][n++] = cvp;
			}
		}
		n = 1;
	}
	
	//每个视点方向的法线夹角与可见度评分
	vector<ViewPoint> bestViewPoint;
	for (int i = 0; i < 6; i++) {
		if (enoughArea[i] == 1) {
			for (int j = 0; j < numVP; j++) {
				if (enoughArea[i] == 1) {
					//法线夹角评分
					normalScore[i][j] = AngleDivideScore(candidateVP[i][j].direction, weightNormal[i].normalize());
					
					//可见度评分
					for (int k = 0; k < numtri[i]; k++) {
						for (int l = 0; l < numtri[i]; l++) {
							//cout <<"视点:"<<numVP<<"/"<<j<<"，面片:"<< numtri[i] << "/" << k << " " << l << endl;
							//colCheck = isVisible(Part[i][k].center, candidateVP[i][j].position, TriSurfaces[l]);  太慢
							colCheck = isVisible(Part[i][k].center, candidateVP[i][j].position, Part[i][l]);
							if (colCheck == 1)
								colCount++;
							if (colCount > 2)
								break;
						}
						if (colCount <= 2) {
							visiableArea[i][j] += Part[i][k].area;
						}
						colCount = 0;
					}
					visiableScore[i][j] = visiableArea[i][j] / allarea[i];

					//综合加权评分
					viewpointScore[i][j] = weightv * viewpointScore[i][j] + weightd * normalScore[i][j];

					//记录最佳视点的分数与位置
					if (viewpointScore[i][j] > maxScore[i]) {
						maxScore[i] = viewpointScore[i][j];
						maxVP[i] = j;
					}
				}
			}
			cout << "可见区域：" << visiableArea[i][maxVP[i]] << ", 总区域：" << allarea[i] << endl;
			//cout << "view:" << visiableScore[i][maxVP[i]] << ", normal:" << normalScore[i][maxVP[i]] << endl;
			ViewPoint bestviewpoint;
			bestviewpoint.position = candidateVP[i][maxVP[i]].position;
			bestviewpoint.direction = candidateVP[i][maxVP[i]].direction;
			bestViewPoint.push_back(bestviewpoint);
			
			cout << " 第" << i << "个方向上的最佳视点为" << "(" << bestviewpoint.position.x << "," << bestviewpoint.position.y << "," << bestviewpoint.position.z << "),";
			cout << "该视点的方向为" << "(" << bestviewpoint.direction.x << "," << bestviewpoint.direction.y << "," << bestviewpoint.direction.z << ")" << endl;
		}
	}
	return bestViewPoint;
}

int DataForm::DirectionDivide(myVector3 direction) {  //方向划分
	int part = 0;
	if (abs(direction.z) >= (sqrt(pow(direction.x, 2) + pow(direction.y, 2))) && direction.z > 0)
	{
		part = 0;
	}
	else if (abs(direction.z) < (sqrt(pow(direction.x, 2) + pow(direction.y, 2))) && (direction.x >= abs(direction.y)))
	{
		part = 1;
	}
	else if (abs(direction.z) < (sqrt(pow(direction.x, 2) + pow(direction.y, 2))) && (direction.y > abs(direction.x)))
	{
		part = 2;
	}
	else if (abs(direction.z) < (sqrt(pow(direction.x, 2) + pow(direction.y, 2))) && (-direction.x >= abs(direction.y)))
	{
		part = 3;
	}
	else if (abs(direction.z) < (sqrt(pow(direction.x, 2) + pow(direction.y, 2))) && (-direction.y > abs(direction.x)))
	{
		part = 4;
	}
	else
	{
		part = 5;
	}
	return part;
}

double DataForm::AngleDivideScore(myVector3 A, myVector3 B) {
	double score = 0;
	double angleRad = A.Angle(B);
	double angle = Rad2Angle(angleRad);
	if (0 <= angle && angle < 15)
	{
		score = 1.00;
	}
	else if (15 <= angle && angle < 30)
	{
		score = 0.75;
	}
	else if (30 <= angle && angle <= 45)
	{
		score = 0.50;
	}
	else
	{
		score = 0.10;
	}
	return score;
}

bool DataForm::isVisible(myVector3 orig, myVector3 position, TriSurface TriSurface) {
	vector<myVector3> edge(2);
	myVector3 tvec, pvec, qvec;
	myVector3 direction = position - orig;
	float det, inv_det;
	float t, u, v;

	edge[0] = TriSurface.vertex[1] - TriSurface.vertex[0];
	edge[1] = TriSurface.vertex[2] - TriSurface.vertex[0];

	pvec = direction.crossProduct(edge[1]);
	det = edge[0].dot(pvec);

	if (det > -EPSILON && det < EPSILON)
		return 0;
	inv_det = 1.0 / det;

	tvec = orig - TriSurface.vertex[0];
	u = tvec.dot(pvec)*inv_det;
	if (u < 0 || u > 1)
		return 0;

	qvec = tvec.crossProduct(edge[0]);
	v = direction.dot(qvec)*inv_det;
	if (v < 0 || v + u > 1)
		return 0;

	return 1;
}

int DataForm::cpyint(const char*& p)
{
	int cpy;
	memWriter = (char*)&cpy;
	memcpy(memWriter, p, 4);
	p += 4;
	return cpy;
}
float DataForm::cpyfloat(const char*& p)
{
	float cpy;
	memWriter = (char*)&cpy;
	memcpy(memWriter, p, 4);
	p += 4;
	return cpy;
}
double DataForm::Rad2Angle(double radian)
{
	return radian * 180 / PI;
}
double DataForm::Angle2Rad(double angle)
{
	return angle / 180 * PI;
}
double Round(double number, unsigned int bits)   //保留n位小数
{
	long long integerPart = number;
	number -= integerPart;
	for (unsigned int i = 0; i < bits; ++i)
		number *= 10;
	number = (long long)(number + 0.5);
	for (unsigned int i = 0; i < bits; ++i)
		number /= 10;
	return integerPart + number;
}
static double m3tomm3(double number)//立方米到立方毫米
{
	return number * pow(10, 9);
}