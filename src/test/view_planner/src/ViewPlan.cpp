#include "ViewPlan.h"
#include <time.h>
#include <stdlib.h>

using namespace std;
using namespace tf2;

vector<ViewPoint> ViewPlan::generateViewPoint(const char *cfilename, int sampleNum, double coverage_rate, MGI &group){
    vector<TriSurface> model = readFile(cfilename); // 读取STL模型点云

    vector<ViewPoint> candidate_view_point;
    vector<ViewPoint> best_view_point;
	g = Graph();
	trajs = vector<vector<moveit_msgs::RobotTrajectory>>(100, vector<moveit_msgs::RobotTrajectory>(100));
	vector<pair<double, int>> RK_index;
	setRandomKey(model.size(), RK_index);
    sortRK(RK_index);
	int rand_sample_num = 0;

	while(!sampleEnough(model, candidate_view_point.size(), sampleNum, coverage_rate) && (rand_sample_num < model.size())){
		int already_sampled = candidate_view_point.size();
		sampleViewPoint(model, sampleNum, already_sampled, candidate_view_point, RK_index, rand_sample_num, group);  // 采样生成候选视点
	}
	g.graph.resize(candidate_view_point.size());
	trajs.resize(candidate_view_point.size());
	for (int i = 0; i < g.graph.size(); ++i){
		g.graph[i][i] = ViewPoint();
		g.graph[i].resize(candidate_view_point.size());

		trajs[i].resize(candidate_view_point.size());
	}

	return candidate_view_point;
}

vector<TriSurface> ViewPlan::readFile(const char *cfilename) {
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
	//vector<TriSurface> TriSurfaces;  //用于读取每个面片的表面信息
	if (name == "solid") {
		return readASCII(buffer);
	}
	else {
		return readBinary(buffer);
	}
	ios::sync_with_stdio(true);

	free(buffer);
	//return 0;
}
vector<TriSurface> ViewPlan::readASCII(const char* buffer) {
	//numTriangles = 0;
	vector<TriSurface> TriSurfaces;  //用于读取每个面片的表面信息
	float x, y, z;
	int  i;
	int number = 0;
	string name, useless;
	stringstream ss(buffer);
	TriSurface surface;
	int numTriangles = 0;

	ss >> name >> name;
	ss.get();
	do {
		ss >> useless;
		if (useless != "facet")
			break;
		//在第一行中获取法向量
		ss >> useless >> x >> y >> z;
		surface.normal.m_floats[0] = x;
		surface.normal.m_floats[1] = y;
		surface.normal.m_floats[2] = z;

		//删去第二行
		getline(ss, useless);
		for (i = 0; i < 3; i++)
		{
			ss >> useless >> x >> y >> z;
			surface.vertex[i].m_floats[0] = x;
			surface.vertex[i].m_floats[1] = y;
			surface.vertex[i].m_floats[2] = z;
		}
		numTriangles++;
		surface.number = ++number;
		//计算三角形面积
		Vector3 AB(surface.vertex[2].m_floats[0] - surface.vertex[0].m_floats[0], surface.vertex[2].m_floats[1] - surface.vertex[0].m_floats[1], surface.vertex[2].m_floats[2] - surface.vertex[0].m_floats[2]);
		Vector3 AC(surface.vertex[1].m_floats[0] - surface.vertex[0].m_floats[0], surface.vertex[1].m_floats[1] - surface.vertex[0].m_floats[1], surface.vertex[1].m_floats[2] - surface.vertex[0].m_floats[2]);
		surface.area = (AB.cross(AC)).length() / 2;

		//面片中心提取
		surface.center.m_floats[0] = (surface.vertex[0].m_floats[0] + surface.vertex[1].m_floats[0] + surface.vertex[2].m_floats[0]) / 3;
		surface.center.m_floats[1] = (surface.vertex[0].m_floats[1] + surface.vertex[1].m_floats[1] + surface.vertex[2].m_floats[1]) / 3;
		surface.center.m_floats[2] = (surface.vertex[0].m_floats[2] + surface.vertex[1].m_floats[2] + surface.vertex[2].m_floats[2]) / 3;

		surface.flag = false;
		// 删去模型底部无法测量的面片
		if(surface.center.m_floats[2] > 5){
			TriSurfaces.push_back(surface);
		}

		getline(ss, useless);
		getline(ss, useless);
		//getline(ss, useless);
	} while (1);
	cout << " 模型读取完成，共读取了" << TriSurfaces.size() << "个面片" << endl;
	return TriSurfaces;
}
vector<TriSurface> ViewPlan::readBinary(const char* buffer) {

	cout << "开始读取二进制模型" << endl;
	const char* p = buffer;
	char name[80];
	int i, j;
	vector<TriSurface> TriSurfaces;  //用于读取每个面片的表面信息
	int num = 0;

	memcpy(name, p, 80);
	p += 80;
	num = cpyint(p);  //三角面片数量
	int number = 0;  //三角面片序号
	for (i = 0; i < num; i++) {
		TriSurface surface;
		//存储面片法向量
		surface.normal.m_floats[0] = cpyfloat(p);
		surface.normal.m_floats[1] = cpyfloat(p);
		surface.normal.m_floats[2] = cpyfloat(p);	
		for (j = 0; j < 3; j++) {
			//存储面片
			Vector3 tmp;
			tmp.m_floats[0] = cpyfloat(p);
			tmp.m_floats[1] = cpyfloat(p);
			tmp.m_floats[2] = cpyfloat(p);
			surface.vertex.push_back(tmp);
			//cout << tmp.y << " ";
		}
		surface.number = ++number;
		//cout << number << endl;

		//计算三角形面积
        Vector3 AB(surface.vertex[2].m_floats[0] - surface.vertex[0].m_floats[0], surface.vertex[2].m_floats[1] - surface.vertex[0].m_floats[1], surface.vertex[2].m_floats[2] - surface.vertex[0].m_floats[2]);
		Vector3 AC(surface.vertex[1].m_floats[0] - surface.vertex[0].m_floats[0], surface.vertex[1].m_floats[1] - surface.vertex[0].m_floats[1], surface.vertex[1].m_floats[2] - surface.vertex[0].m_floats[2]);
		surface.area = (AB.cross(AC)).length() / 2;

		//面片中心提取
		//cout << i << " " << surface.vertex[0].y << " " << surface.vertex[1].y << " " << surface.vertex[2].y << endl;
		surface.center.m_floats[0] = (surface.vertex[0].m_floats[0] + surface.vertex[1].m_floats[0] + surface.vertex[2].m_floats[0]) / 3;
		// surface.center.m_floats[0] = 0.77;
		// surface.center.m_floats[1] = 0;
		surface.center.m_floats[1] = (surface.vertex[0].m_floats[1] + surface.vertex[1].m_floats[1] + surface.vertex[2].m_floats[1]) / 3;
		surface.center.m_floats[2] = (surface.vertex[0].m_floats[2] + surface.vertex[1].m_floats[2] + surface.vertex[2].m_floats[2]) / 3;
		//cout << surface.center.y << " ";
		
		surface.flag = false;
		if(surface.center.m_floats[2] > 5){
			TriSurfaces.push_back(surface);
		}

		p += 2; //跳过尾部标志
	}
	cout << " 模型读取完成，共读取了" << TriSurfaces.size() << "个面片" << endl;
	return TriSurfaces;
}

void ViewPlan::sampleViewPoint(const vector<TriSurface> &model, int sampleNum, int already_sampled, vector<ViewPoint> &candidate_view_point, const vector<pair<double, int>> &RK_index, int &rand_sample_num, MGI &group){
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");  // 原为getJointState第一行，但在循环中无法生成随机数，所以提前声明
	srand((unsigned)time(NULL));

    cout<<"开始生成候选视点："<<endl;
	int candidate_num = already_sampled;

	// // 模型中心计算
	double x_sum = 0;
	double y_sum = 0;
	double z_sum = 0;
	for (int i = 0; i < model.size(); i++) {
		// x_sum += model[i].center.m_floats[0];
		// y_sum += model[i].center.m_floats[1];
		z_sum += model[i].center.m_floats[2];
	}
	double center_x = model_position_x;
	double center_y = 0;
	double center_z = model_position_z + 0.04;
    
	while((candidate_view_point.size() - already_sampled) < sampleNum){
		ViewPoint candidate;
		vector<int> tempVisibility;
		if(rand_sample_num >= model.size()){
			cout << "随机采样数超过模型面片数！" <<model.size()<< endl;
			break;
		}
		int randNum = RK_index[rand_sample_num].second;
		++rand_sample_num;
		// 生成视点位置
		// yang
		// candidate.position = model[randNum].center + model[randNum].normal.normalized() * measure_dist; // 沿面片法线方向延伸最佳测量距离，生成候选视点

		// xcf
		float randNum_1 = (float)rand() / RAND_MAX * 0.4;
		float randNum_2 = (float)rand() / RAND_MAX * 0.6 + 0.4;
		float randNum_3 = (float)rand() / RAND_MAX;
		if(candidate_num < 0.4 * sampleNum) {
			candidate.position.m_floats[0] = center_x + measure_dist * cos(randNum_1 * PI/2) * cos(randNum_3 * 2*PI);
			candidate.position.m_floats[1] = center_y + measure_dist * cos(randNum_1 * PI/2) * sin(randNum_3 * 2*PI);
			candidate.position.m_floats[2] = center_z + measure_dist * sin(randNum_1 * PI/2);
		}
		else {
			candidate.position.m_floats[0] = center_x + measure_dist * cos(randNum_2 * PI/2) * cos(randNum_3 * 2*PI);
			candidate.position.m_floats[1] = center_y + measure_dist * cos(randNum_2 * PI/2) * sin(randNum_3 * 2*PI);
			candidate.position.m_floats[2] = center_z + measure_dist * sin(randNum_2 * PI/2);
		}

		// 生成视点的z过低，重新生成
		if(candidate.position.m_floats[2] < -300){
			cout << "生成视点的z过低，重新生成!" << endl;
			continue;
		}
        
		// 在所选面片的rangeFOD/2范围内寻找夹角小于90度的邻面片，势场法计算视点方向  
		// yang   
		// candidate.direction = Vector3(0,0,0);
        // for(int j = 0; j < model.size(); ++j){
        //     double dist = (model[j].center - model[randNum].center).length();
        //     double theta = model[randNum].normal.angle(model[j].normal);
		// 	if (dist <= (maxFOD - minFOD) / 2 && theta <= PI / 2){
		// 		candidate.direction += model[j].area * (model[j].center - candidate.position) / (model[j].center - candidate.position).length();  // 视点方向 = sum（邻面片方向*面积/距视点的距离）
		// 	}
		// }

		// xcf
		// 加权求和生成视点方向
		Vector3 model_center(center_x,center_y,center_z);
        // for(int j = 0; j < model.size(); ++j){
        //     double dist = (model[j].center - candidate.position).length();
        //     double theta = (candidate.position - model_center).angle(model[j].normal);
		// 	if (dist <= maxFOD && dist >=minFOD && theta <= PI / 3){
		// 		candidate.direction += model[j].area * (model[j].center - candidate.position) / (model[j].center - candidate.position).length();  // 视点方向 = sum（邻面片方向*面积/距视点的距离）
		// 	}
		// }
		// 以模型中心为视点方向
		candidate.direction = (model_center - candidate.position).normalized();

		// 计算视点的机器人轴配置参数和碰撞检测，舍弃无法求解IK或发生碰撞的候选视点
		if(!getJointState(candidate, robot_model_loader) || checkCollision(candidate, robot_model_loader)){
			cout << "运动学不可解！" << endl;
			continue;
		}

		// 计算视点的可见性矩阵
		for(int j = 0; j < model.size(); ++j){
			int n = checkVisibility(candidate, model, j);
			candidate.vis_area += n;
			tempVisibility.push_back(n);
		}

		// 生成视点图
		candidate.num = candidate_num++;
		if(candidate.num != 0){
			setGraph(candidate_view_point, candidate, group, robot_model_loader);
		}

        candidate.direction = candidate.direction.normalize();
		candidate_view_point.push_back(candidate);
		visibility_matrix.push_back(tempVisibility);

		cout << "生成第" << candidate.num << "个视点, 随机数为" << randNum << endl;
	}
}
void ViewPlan::setRandomKey(int size, vector<pair<double, int>> &RK_index){
    for (int i = 0; i < size; i++){
        pair<double, int> temp;  // <key， 序号>
        temp.first = (double)rand() / RAND_MAX;
        while (temp.first == 1.0) {
            temp.first = ((double) rand() / RAND_MAX);
        }
        temp.second = i;
        RK_index.push_back(temp);
        //cout << "104 setRK: " << RK_index[i].first << ", " << RK_index[i].second << endl;
    }
}
void ViewPlan::sortRK(vector<pair<double, int>> &RK_index){
    sort(RK_index.begin(), RK_index.end(), less<pair<double, int>>());
}
bool ViewPlan::sampleEnough(const vector<TriSurface> &model, int candidate_num, int sample_num, double coverage_rate){
	// 未开始采样，可见性矩阵仍为空，进行采样
	if(visibility_matrix.size() == 0){
		cout<<"可见性矩阵容量为0"<<endl;
		return 0;
	}

	// 判断模型表面覆盖率是否达到要求
	double visible_num = 0;
	for(int j = 0; j < model.size(); j++){
		double visible_tmp = 0;
		for(int i = 0; i < candidate_num; i++){
			visible_tmp += visibility_matrix[i][j];
			if(visible_tmp == 2)  // 建议设为2
				break;
		}
		visible_num += visible_tmp / 2;
	}
	cout<<"表面覆盖率为： "<< visible_num / model.size() <<endl;
	if(visible_num >= coverage_rate * model.size()){
		return 1;
	}

	// 采样数超过限额，停止采样
	if(candidate_num >= (sample_num * 2)){
		cout<<"采样数超过限额，停止采样"<<endl;
		return 1;
	}
	
	return 0;
}
bool ViewPlan::getJointState(ViewPoint &viewpoint, robot_model_loader::RobotModelLoader robot_model_loader){
    // 实例化一个RobotModelLoader对象，该对象将在ROS参数服务器上查找机器人描述并构建一个 RobotModel供我们使用。
    //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    // Using the RobotModel, we can construct a RobotState that maintains the configuration of the robot. 
    // We will set all joints in the state to their default values. We can then get a JointModelGroup, 
    // which represents the robot model for a particular group, e.g. the “arm” of the abb robot.
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
    const vector<std::string>& joint_names = joint_model_group->getVariableNames();
    vector<double> joint_values;
	
	// 初始化机器人末端关节状态
	Eigen::Isometry3d end_effector_state = Eigen::Isometry3d::Identity();    
	// 将视点的位置与方向转换为平移和旋转矩阵
	Eigen::Matrix3d rotMatrix;
	Eigen::Vector3d vectorBefore(1, 0, 0);  // joint_6的初始方向
	Eigen::Vector3d vectorAfter(viewpoint.direction.m_floats[0], viewpoint.direction.m_floats[1], viewpoint.direction.m_floats[2]);

	viewpoint.quaternion = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter);
	rotMatrix = viewpoint.quaternion.toRotationMatrix();
	Eigen::Vector3d translation(viewpoint.position.m_floats[0]/1000 + model_position_x, viewpoint.position.m_floats[1]/1000, viewpoint.position.m_floats[2]/1000 + model_position_z);  // 平移矩阵(单位：m)
	// 机器人基座到视点位置的变换矩阵T1
	end_effector_state.rotate(rotMatrix);
	end_effector_state.pretranslate(translation);

	// 手眼标定结果 X1
	Eigen::Matrix4d rob_cam_calibration;
	// 右相机到末端（从末端移动到右相机） 第一次标定（x为光轴）
	rob_cam_calibration << 0.206326, 0.001152, 0.978508, 0.067869619,
					      -0.000008, 0.999980, 0.001183,-0.139219215,
					      -0.978482, 0.000183, 0.206310, 0.102563320,
					       0,        0,        0,        1;
	// 左相机到末端（从末端移动到左相机）手眼标定结果（z为光轴）
	// rob_cam_calibration << -0.231825,  0.002388,  0.972779, 0.060808487,
    //                        -0.972672,  0.011158, -0.231820, 0.140728791,
    //                        -0.011499, -0.999932, -0.000279, 0.103992375,
    //                         0.000000,  0.000000,  0.000000, 1.000000;
	// 左相机到末端（从末端移动到左相机）手眼标定结果（x为光轴）
	// rob_cam_calibration << -0.000279, 0.011499,  0.999932, 0.060808487,
    //                         -0.972779, -0.231825,  0.002388, 0.140728791,
    //                         0.231820, -0.972672,  0.011158, 0.103992375,
    //                         0.000000,  0.000000,  0.000000, 1.000000;

	// 相机到投影仪标定结果 X2
	Eigen::Matrix4d cam_pro_calibration;
	// 左相机到投影仪标定结果
	// 原始左相机坐标系下标定结果（z为光轴）
	// cam_pro_calibration << 0.9726830895886041,  -0.002346875884211844, 0.2321251804564764,  -0.1409574458184522,
	// 				       0.003367426665242761, 0.9999863284976849,  -0.004000406901106243,-0.0001083651746352479,
	// 				      -0.2321126184980511,   0.004672792666523019, 0.9726776944819254,  -0.02790086249003743,
	// 				       0,                    0,                    0,                    1;
	// 旋转至测头坐标系下的标定结果（x为光轴）
	cam_pro_calibration <<  0.9726776944819254,  -0.2321126184980511,   0.004672792666523019,  -0.02790086249003743,
					        0.2321251804564764,   0.9726830895886041,  -0.002346875884211844,  -0.1409574458184522,
					       -0.004000406901106243, 0.003367426665242761, 0.9999863284976849,    -0.0001083651746352479,
					        0,                    0,                    0,                      1;
	// 右相机到投影仪标定结果
	// 原始右相机坐标系下标定结果（z为光轴）
	// cam_pro_calibration << 0.976166286897133,    0.005452384782800655, -0.2169554143727337,    0.1410143582539718,
	// 				      -0.005658054933568069, 0.9999839396727248,   -0.0003268194990640644,-0.0009730115521494622,
	// 				       0.2169501480521114,   0.001546575829542898,  0.9761814592397104,   -0.02639814021943194,
	// 				       0,                    0,                     0,                     1;
	// 旋转至测头坐标系下的标定结果（x为光轴）
	// cam_pro_calibration <<  0.9761814592397104,     0.2169501480521114,   0.001546575829542898, -0.02639814021943194,
	// 				       -0.2169554143727337,     0.976166286897133,    0.005452384782800655,  0.1410143582539718,
	// 				       -0.0003268194990640644, -0.005658054933568069, 0.9999839396727248,   -0.0009730115521494622,
	// 				        0,                      0,                    0,                      1;

	// 投影仪到末端的变换矩阵（手动估计）
	Eigen::Matrix4d R;
	R << 0,  0,  1, 0.09,
		 0,  1,  0, 0,
		-1,  0,  0, 0.073,
		 0,  0,  0, 1;
	// 视点矫正矩阵o1到o2
	// Eigen::Matrix4d T;
	// T << 0.88420963, -0.15563251,  0.44041094,  -0.21563197,
	// 	 0.15284006,  0.98735571,  0.042056009, -0.28902084,
	// 	-0.4413875,   0.030126061, 0.89681613,   0.32228275,
	// 	 0,           0,           0,            1;
	// 根据手眼标定关系校正后的机器人位姿变换矩阵T2      T1 = T2 * X1 * X2
	// end_effector_state = end_effector_state * cam_pro_calibration.inverse() * rob_cam_calibration.inverse();
	// end_effector_state = end_effector_state * (R * cam_pro_calibration) * rob_cam_calibration.inverse();
	// end_effector_state = end_effector_state * rob_cam_calibration.inverse();
	// 根据手眼标定关系校正后的机器人位姿变换矩阵T2      T1 = T2 * R
	end_effector_state = end_effector_state * R.inverse();

	// 我们现在可以为机器人求解逆运动学 (IK)。要解决 IK，我们需要以下内容：
	// 1.机械臂末端的所需姿势（默认情况下，这是“arm”链中的最后一个链接）：我们在上述步骤中计算的 end_effector_state。
	// 2.超时时间：0.1 秒
	double timeout = 0.1;
	int n = 0;
	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);
	// 如果没找到IK解，则绕光轴旋转一定角度重新求解
	Eigen::AngleAxisd v(PI / 90, Eigen::Vector3d(0,0,1));        // 旋转角度v
	Eigen::Matrix3d rotationMatrix = v.matrix();                 // 绕光轴旋转20度的旋转矩阵
	Eigen::Vector3d translation2(0, 0, 0);                       // 平移矩阵，0
	Eigen::Isometry3d rotation = Eigen::Isometry3d::Identity();  // 变换矩阵
	rotation.rotate(rotationMatrix);
	rotation.pretranslate(translation2);
	while(!found_ik){
		end_effector_state = end_effector_state * R * rotation * R.inverse();
		found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);
		// 旋转一周为止
		if(++n >= 180)
			break;
	}
	// 有IK解则记录该位置的轴位置 / 视点位姿 / 机器人末端位姿
	if (found_ik){
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for (std::size_t i = 0; i < joint_names.size(); ++i){
			viewpoint.joint_state.push_back(joint_values[i]);
		}
		viewpoint.end_effector_state = end_effector_state;
		rotMatrix = end_effector_state.rotation();
		translation = end_effector_state.translation();
		for (int i = 0; i < 3; i++){
			viewpoint.robot_position.m_floats[i] = translation[i];
			// cout << viewpoint.robot_position.m_floats[i] << " ";
		}
		viewpoint.quaternion = rotMatrix;
		return 1;
	}
	else{
		cout<<"视点的IK解求取失败,";
		//ROS_INFO("Did not find IK solution");
		return 0;
	}	
}
bool ViewPlan::isCovered(Vector3 orig, Vector3 position, const TriSurface &TriSurface){
	vector<Vector3> edge(2);
	Vector3 tvec, pvec, qvec;
	Vector3 direction = position - orig;
	float det, inv_det;
	float t, u, v;
	const float EPSILON = 0.000001;

	edge[0] = TriSurface.vertex[1] - TriSurface.vertex[0];
	edge[1] = TriSurface.vertex[2] - TriSurface.vertex[0];

	pvec = direction.cross(edge[1]);
	det = edge[0].dot(pvec);

	if (det > -EPSILON && det < EPSILON)
		return 0;
	inv_det = 1.0 / det;

	tvec = orig - TriSurface.vertex[0];
	u = tvec.dot(pvec)*inv_det;
	if (u < 0 || u > 1)
		return 0;

	qvec = tvec.cross(edge[0]);
	v = direction.dot(qvec)*inv_det;
	if (v < 0 || v + u > 1)
		return 0;

	return 1;
}
int ViewPlan::checkVisibility(const ViewPoint &view_point, const vector<TriSurface> &model, int i){
	// 可见性准则：
	// 1.表面片的所有顶点都在视场FOV内 ———— view_point.direction.angle(model[i] - view_point) < rangeFOV
	// 2.表面片的所有顶点都在景深FOD内 ———— minFOD < distance < maxFOD
	// 3.视角必须在特定角度之内 ———— model[i].normal.angle(view_point - model[i])
	// 4.没有遮挡 ———— isCovered()
	// 1&2.判断表面片顶点是否都在FOV和FOD中
	for (int j = 0; j < 3; j++){
		double dist = (model[i].vertex[j] - view_point.position).dot(view_point.direction.normalized());
		if (dist <= minFOD || dist >= maxFOD ||
			abs(view_point.direction.angle(model[i].vertex[j] - view_point.position)) >= minFOV / 2){
			return 0;
		}
	}

	// 3.判断视角是否在规定范围中
	if(model[i].normal.angle(view_point.position - model[i].center) >= view_angle_range){
		return 0;
	}

	// 4.判断遮挡
	int colCount = 0;  // 遮挡面片数
	bool colCheck = 0;  // 是否被遮挡
	for(int j=0;j<model.size();j++){
		colCheck = isCovered(model[i].center, view_point.position, model[j]);
		if(colCheck == 1)
			colCount++;
		if(colCount > 2){
			return 0;
		}
	}
	return 1;
}
bool ViewPlan::checkCollision(const ViewPoint &view_point, robot_model_loader::RobotModelLoader robot_model_loader){
    // 加载机器人的运动学模型到情景实例中
	const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
	planning_scene::PlanningScene planning_scene(kinematic_model);

	// 创建一个碰撞检测的请求对象和响应对象
	collision_detection::CollisionRequest c_req;
	collision_detection::CollisionResult c_res;

	// 修改机器人状态
	robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
	const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("arm");
	current_state.setJointGroupPositions(joint_model_group, view_point.joint_state);
	
	planning_scene.checkCollision(c_req, c_res, current_state);
	//cout<<"Full collision Test: " << (c_res.collision ? "in" : "not in") << " collision" << endl;
	return c_res.collision;
}
void ViewPlan::setGraph(vector<ViewPoint> candidate_view_point, ViewPoint candidate, MGI &group, robot_model_loader::RobotModelLoader robot_model_loader){
	// 该函数负责计算新视点与已有视点之间的运动成本
	const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
	planning_scene::PlanningScene planning_scene(kinematic_model);
	robot_state::RobotState &start_state = planning_scene.getCurrentStateNonConst();
	const robot_model::JointModelGroup* joint_model_group = start_state.getJointModelGroup("arm");

	// 以运动时间为运动成本
	for(int i = 0; i < candidate_view_point.size(); ++i){
		double cost = 0.0;

		// 以candidate_view_point[i]为起点，candidate为终点规划轨迹，以视点位置的机器人轴关节角度为起始点与目标点状态
		start_state.setJointGroupPositions(joint_model_group, candidate_view_point[i].joint_state);
		group.setStartState(start_state);
        group.setJointValueTarget(candidate.joint_state);

		// 规划视点之间的运动轨迹
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    	moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

		// 若存在轨迹，则以轨迹运动时间为两视点之间的运动成本；反之则设置运动成本为无限大
		if(success){
			int size = my_plan.trajectory_.joint_trajectory.points.size();
			cost = my_plan.trajectory_.joint_trajectory.points[size - 1].time_from_start.toSec();
		}
		else{
			cost = DBL_MAX;
		}

		moveit_msgs::RobotTrajectory traj = my_plan.trajectory_;

		// 将新节点插入图中，并绘制边界
		// g->insertEdge(candidate_view_point[i].num, candidate.num, cost, traj, true);
		g.insertGraph(candidate_view_point[i].num, candidate.num, cost, traj);
		trajs[candidate_view_point[i].num][candidate.num] = traj;

		// -----------------------------------------------------------------------------------------
		// 以candidate为起点，candidate_view_point[i]为终点规划轨迹
		start_state.setJointGroupPositions(joint_model_group, candidate.joint_state);
		group.setStartState(start_state);
        group.setJointValueTarget(candidate_view_point[i].joint_state);

    	success = group.plan(my_plan);

		if(success){
			int size = my_plan.trajectory_.joint_trajectory.points.size();
			cost = my_plan.trajectory_.joint_trajectory.points[size - 1].time_from_start.toSec();
		}
		else{
			cost = DBL_MAX;
		}

		traj = my_plan.trajectory_;

		// g->insertEdge(candidate.num, candidate_view_point[i].num, cost, traj, true);
		g.insertGraph(candidate.num, candidate_view_point[i].num, cost, traj);
		trajs[candidate.num][candidate_view_point[i].num] = traj;
	}
}
moveit_msgs::RobotTrajectory calcMotionCost(ViewPoint candidate_view_point, ViewPoint candidate, MGI &group, planning_scene::PlanningScene &planning_scene){
	// robot_state::RobotState &start_state = planning_scene.getCurrentStateNonConst();
	// const robot_model::JointModelGroup* joint_model_group = start_state.getJointModelGroup("arm");
	// start_state.setJointGroupPositions(joint_model_group, candidate.joint_state);
		
	// group.setStartState(start_state);
    // group.setPoseTarget(candidate_view_point.end_effector_state);

	// moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

	// if(success){
	// 	return my_plan.trajectory_;
	// 	}
	// else{
	// 	return;
	// }
}

int ViewPlan::cpyint(const char*& p)
{
	int cpy;
	char* memWriter;
	memWriter = (char*)&cpy;
	memcpy(memWriter, p, 4);
	p += 4;
	return cpy;
}
float ViewPlan::cpyfloat(const char*& p){
	float cpy;
	char* memWriter;
	memWriter = (char*)&cpy;
	memcpy(memWriter, p, 4);
	p += 4;
	return cpy;
}
double ViewPlan::getModelPositionX() const { return model_position_x; }
double ViewPlan::getModelPositionZ() const { return model_position_z; }
double ViewPlan::getTablePositionX() const { return table_position_x; }
double ViewPlan::getTablePositionZ() const { return table_position_z; }

ViewPoint::ViewPoint(int num, double cost){
	this->num = num;
	this->cost = cost;
}
ViewPoint::ViewPoint(int num_, int vis_area_, vector<double> joint_state_): num(num_), vis_area(vis_area_), joint_state(joint_state_){}
ViewPoint::ViewPoint() : vis_area(0) {}

Graph::Graph(){
	// for (int i = 0; i < 1000; ++i)
	// 	this->edges[i] = NULL;
	graph = vector<vector<ViewPoint>>(100, vector<ViewPoint>(100));
}
Graph::~Graph(){ }
void Graph::insertEdge(int x, int y, double cost, moveit_msgs::RobotTrajectory traj, bool directed){
	// ViewPoint *edge = new ViewPoint(y, cost, traj);
	// edge->next = this->edges[x];
	// this->edges[x] = edge;
	// //cout<<"已添加视点 "<<x<<" 的邻节点 "<<this->edges[x]->num<<endl;
	// if(!directed){
	// 	insertEdge(y, x, cost, traj, true);
	// }
}
void Graph::insertGraph(int from, int to, double cost, moveit_msgs::RobotTrajectory traj){
	ViewPoint tmp(to, cost);
	graph[from][to] = tmp;
}
void Graph::print(){
	// for(int v = 0; v < 1000; ++v){
    //     if(this->edges[v] != NULL){
    //         cout << "视点 " << v << " 有以下邻节点: " << endl;
    //         ViewPoint *curr = this->edges[v];
    //         while(curr != NULL){
    //             cout << "num: " << curr->num << ", cost: " << curr->cost << endl;
    //             curr = curr->next;
    //         }
    //     }
    // }
}
