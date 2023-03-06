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
typedef moveit::planning_interface::MoveGroupInterface MGI;

// 存储STL模型表面三角面片信息
struct TriSurface {
	bool flag;
	int number;
	double area;
    Vector3 normal;
	Vector3 center;
	vector<Vector3> vertex;
};

// 存储视点信息
class ViewPoint {
public:
	int num;                               // 视点序号
	Vector3 position;                      // 视点位置
	Vector3 direction;                     // 视点方向
	int vis_area;					       // 视点可见面积
	Vector3 robot_position;				   // 视点位置对应的机械臂末端位置
	Eigen::Quaterniond quaternion;         // 视点位置对应的机械臂四元数
	double cost;                           // 运动成本
	vector<double> joint_state;            // 视点位姿下六个轴的关节状态
	Eigen::Isometry3d end_effector_state;  // 末端状态
	// moveit_msgs::RobotTrajectory traj;     // 轨迹
	// ViewPoint *next;
	// ViewPoint(int num, double cost, moveit_msgs::RobotTrajectory traj);
	ViewPoint(int num, double cost);
	ViewPoint(int num, int vis_area, vector<double> joint_state);
	ViewPoint();
};

// 创建图，存储视点
class Graph{
	public:
	// ViewPoint *edges[1000];
	vector<vector<ViewPoint>> graph;
	Graph();
	~Graph();
    void insertEdge(int, int, double, moveit_msgs::RobotTrajectory, bool);
	void insertGraph(int, int, double, moveit_msgs::RobotTrajectory);
	void print();
};

class ViewPlan{
public:
	// 读取STL+生成候选视点
 vector<ViewPoint> generateViewPoint(const char* cfilename,
                                     int sampleNum,
                                     double coverage_rate,
                                     MGI& group);
 double getModelPositionX() const;
 double getModelPositionZ() const;
 double getTablePositionX() const;
 double getTablePositionZ() const;

 Graph g;
 vector<vector<int>> visibility_matrix;               // 可见性矩阵
 vector<vector<moveit_msgs::RobotTrajectory>> trajs;  // 轨迹
 vector<TriSurface> model;
	
private:
    // 相机参数
	const float PI                = 3.1415926;
	const double measure_dist     = 700.0;                 // 最佳测量距离(单位：mm)
    const double minFOD           = measure_dist - 200.0;  // 前景深(单位：mm) minFOD = measure_dist - 200
	const double maxFOD           = measure_dist + 150.0;  // 后景深(单位：mm) maxFOD = measure_dist + 400
    //int rangeFOD                = maxFOD - minFOD;       // 测量范围
	const double minFOV           = 0.52359877;            // 视野(弧度) 水平视角30度，垂直视角25度  minFOV = 30/180*PI = 0.52359877
	const double view_angle_range = 1.047;                 // 测量视角范围 view_angle_range = 60/180*PI = 1.047
	// 仿真场景距离参数
	const double model_position_x = 0.77;                  // 待测模型在x轴上的位置（单位：m）  
	const double model_position_z = 0.44;                  // 待测模型在z轴上的位置（单位：m）  
	const double table_position_x = model_position_x;      // 平台在x轴上的位置（单位：m）  
	const double table_position_z = model_position_z / 2;  // 平台在z轴上的位置（单位：m）
    
	//int numTriangles = 0;  // 面片数量
	//int sampleNum;  // 视点采样数量

	vector<TriSurface> readFile(const char *cfilename);    // 读取STL文件
    vector<TriSurface> readASCII(const char *cfilename);   // 读取ASCII格式STL
	vector<TriSurface> readBinary(const char *cfilename);  // 读取二进制格式STL
	
	// 生成候选视点
    void sampleViewPoint(const vector<TriSurface>& model,
                         int sampleNum,
                         int already_sampled,
                         vector<ViewPoint>& candidate_view_point,
                         const vector<pair<double, int>>& RK_index,
                         int& rand_sample_num,
                         MGI& group);
    
	// 给每个面片设置随机Key值
	void setRandomKey(int size, vector<pair<double, int>>& RK_index);  

	// 将Key从小到大排序
    void sortRK(vector<pair<double, int>>& RK_index);  

	// 判断是否已经生成足够视点 sampleEnough(模型, 已采集的视点, 每次采样数)
    bool sampleEnough(const vector<TriSurface>& model,
                      int candidate_num,
                      int sample_num,
                      double coverage_rate);

    // 计算机器人在视点位置处的轴配置数据
    bool getJointState(ViewPoint& viewpoint,
                       robot_model_loader::RobotModelLoader robot_model_loader);

    // 检测是否被遮挡          
    bool isCovered(Vector3 orig, Vector3 position, const TriSurface &TriSurface);  

	// 检测可见性，并绘制可见性矩阵
    int checkVisibility(const ViewPoint& view_point,
                        const vector<TriSurface>& model,
                        int i);

    // 检测视点位置处碰撞
    bool checkCollision(const ViewPoint& view_point,
            			robot_model_loader::RobotModelLoader robot_model_loader);

    // 绘制图
    void setGraph(vector<ViewPoint> candidate_view_point,
                  ViewPoint candidate,
                  MGI& group,
                  robot_model_loader::RobotModelLoader robot_model_loader);

    moveit_msgs::RobotTrajectory calcMotionCost(ViewPoint candidate_view_point,
        										ViewPoint candidate,
        										MGI& group,
       											planning_scene::PlanningScene& planning_scene);

    int cpyint(const char*& p);
    float cpyfloat(const char*& p);
};

#endif