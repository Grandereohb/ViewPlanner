#include <ros/ros.h>
#include "ViewPlan.h"

using namespace std;
using namespace tf2;
const double PI = 3.1415926;
const double interval = PI / 36;
const double model_position_x = 0.77;
const double model_position_z = 0.44;

bool getJointState(ViewPoint &viewpoint, robot_model_loader::RobotModelLoader robot_model_loader){
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

	// 投影仪到末端的变换矩阵（手动估计）
	Eigen::Matrix4d R;
	R << 0,  0,  1, 0.09,
		 0,  1,  0, 0,
		-1,  0,  0, 0.073,
		 0,  0,  0, 1;

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


int main(int argc, char **argv){
    ros::init(argc, argv, "ViewAngleTest");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    moveit::planning_interface::MoveGroupInterface group("arm");
    const moveit::core::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup("arm");
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");  

    group.allowReplanning(true);            //当运动规划失败后，允许重新规划
    group.setGoalJointTolerance(0.001);
    group.setGoalPositionTolerance(0.001);  //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.setGoalOrientationTolerance(0.01);

    Vector3 model_center(0, 0, 0);
    double measure_dist = 700;
    Vector3 view_position, view_direction;
    geometry_msgs::Pose view_pose;

    for (int i = 0; i < 10; i++){
        ViewPoint cur;
        cur.position.m_floats[0] = -measure_dist * cos(PI / 2 - interval * i);
        cur.position.m_floats[1] = 0;
        cur.position.m_floats[2] = measure_dist * sin(PI / 2 - interval * i);
		cout << PI / 2 - interval * i << endl;
		cout << cur.position.m_floats[0] << " " << cur.position.m_floats[1] << " " << cur.position.m_floats[2] << endl;
		cur.direction = -cur.position.normalized();
		cout << cur.direction.m_floats[0] << " " << cur.direction.m_floats[1] << " " << cur.direction.m_floats[2] << endl;

		getJointState(cur, robot_model_loader);
        group.setStartStateToCurrentState();    // 将机器人的初始状态设置为当前状态
        group.setJointValueTarget(cur.joint_state);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    	moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
        if(success)
            group.execute(my_plan);
        int n;
        cin >> n;
    }

    return 0;
}
 