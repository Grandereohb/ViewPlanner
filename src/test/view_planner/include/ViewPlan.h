#ifndef VIEWPLAN_H
#define VIEWPLAN_H

#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cmath>
// #include "RKGA.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/kinematic_constraints/utils.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

using namespace tf2;
using namespace std;


// 存储STL模型表面三角面片信息
struct TriSurface {
	bool flag;
	int number;
	double area;
    Vector3 normal;
	Vector3 center;
	vector<Vector3> vertex;
};

// 存储视点位置与方向
class ViewPoint {
public:
	int num;
	double cost;
	Vector3 position;
	Vector3 direction;
	Eigen::Quaterniond quaternion;
	vector<double> joint_state;
	ViewPoint *next;
	ViewPoint(int num, double cost);
	ViewPoint();
};

// 创建图，存储视点
class Graph{
	public:
	ViewPoint *edges[1000];
	Graph();
	~Graph();
    void insertEdge(int, int, double, bool);
    void print();
};

class ViewPlan{
public:
    vector<ViewPoint> generateViewPoint(const char *cfilename,int sampleNum, double coverage_rate); // 读取STL+生成候选视点
	double getModelPositionX();
	double getModelPositionZ();
	double getTablePositionX();
	double getTablePositionZ();
	Graph *g = new Graph();
	vector<vector<int>> visibility_matrix;  // 可见性矩阵
	
private:
    // 相机参数
	const float PI =3.1415926;
	const int measure_dist = 540;  // 最佳测量距离(单位：mm)
    int minFOD = 340;  // 前景深(单位：mm) minFOD = measure_dist - 200
	int maxFOD = 940;  // 后景深(单位：mm) maxFOD = measure_dist + 400
    //int rangeFOD = maxFOD - minFOD;  // 测量范围
	double minFOV = 0.463;  // 视野(弧度) 水平视角30度，垂直视角25度  minFOV = 25/180*PI = 0.436332306
	double view_angle_range = 1.309;  // 测量视角范围 view_angle_range = 75/180*PI = 1.309
	// 仿真场景距离参数
	double model_position_x = 0.8;  // 待测模型在x轴上的位置（单位：m）  
	double model_position_z = 0.35;  // 待测模型在z轴上的位置（单位：m）  
	double table_position_x = model_position_x - 0.05;  // 平台在x轴上的位置（单位：m）  
	double table_position_z = (model_position_z - 0.07)/2;  // 平台在z轴上的位置（单位：m） z = (转向节中心高度 - 转向节下半部分高度0.07)/2 
    
	//int numTriangles = 0;  // 面片数量
	//int sampleNum;  // 视点采样数量

	vector<TriSurface> readFile(const char *cfilename);  // 读取STL文件
    vector<TriSurface> readASCII(const char *cfilename);  // 读取ASCII格式STL
	vector<TriSurface> readBinary(const char *cfilename);  // 读取二进制格式STL
	
    void sampleViewPoint(const vector<TriSurface> &model, int sampleNum, int already_sampled, vector<ViewPoint> &candidate_view_point);  // 生成候选视点
	bool sampleEnough(const vector<TriSurface> &model, int candidate_num, int sample_num, double coverage_rate);  // 判断是否已经生成足够视点 sampleEnough(模型, 已采集的视点, 每次采样数)
	bool getJointState(ViewPoint &viewpoint, robot_model_loader::RobotModelLoader robot_model_loader);  // 计算机器人在视点位置处的轴配置数据
    bool isCovered(Vector3 orig, Vector3 position, const TriSurface &TriSurface);  // 检测是否被遮挡
	int checkVisibility(const ViewPoint &view_point, const vector<TriSurface> &model, int i);  // 检测可见性，并绘制可见性矩阵
	bool checkCollision(const ViewPoint &view_point, robot_model_loader::RobotModelLoader robot_model_loader);  // 检测视点位置处碰撞
	void setGraph(vector<ViewPoint> candidate_view_point, ViewPoint candidate);  // 绘制图

	int cpyint(const char*& p);
	float cpyfloat(const char*& p);



};

#endif