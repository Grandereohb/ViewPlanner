#include "ViewPlan.h"
#include "RKGA.h"
#include "MCST.h"
#include <ros/ros.h>
#include <cstring>
#include <vector>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

// 在RViz环境中发布实验环境/视点位置/机器人运动轨迹
ros::Publisher env_vis_pub, view_point_vis_pub, traj_vis_pub;
void visEnv(const char *file_path_small, const ViewPlan &vp);
void visCandidateViewPoint(const vector<ViewPoint> cand_view_point, const ViewPlan &vp);
void visBestViewPoint(const vector<ViewPoint> best_view_point, const ViewPlan &vp);

int main(int argc, char **argv)
{
    // 使用前请根据需求修改以下参数 
    const char* file_path       = "/home/ros/abb_ws/src/test/view_planner/model/model1123_5k.stl";  // 用于视点生成的模型文件路径
    const char* file_path_small = "package://view_planner/model/model1123_5k_small.stl";            // 用于构建场景的模型文件路径

    int sampleNum        = 40;    // 采样候选视点个数
    double coverage_rate = 0.98;   // 要求的采样覆盖率

    // RKGA参数
    int maxGen        = 200;  // 最大进化代数
    int pop           = 200;  // 每代个体样本数
    double pop_elite  = 0.1;  // 每代种群中的精英个体比例
    double pop_mutant = 0.3;  // 每代种群中变异的个体比例
    double rhoe       = 70;   // probability that an offspring inherits the allele of its elite parent

    // ROS初始化
    ros::init(argc, argv, "planner");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 定义添加环境与视点模型的消息发布端，定义轨迹可视化接口
    env_vis_pub        = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    view_point_vis_pub = node_handle.advertise<visualization_msgs::Marker>("vis_view_point", 1);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link", "vis_traj");
    visual_tools.deleteAllMarkers();

    // 定义moveit规划接口group，关节模型指针和视点规划类vp
    moveit::planning_interface::MoveGroupInterface group("arm");
    const moveit::core::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup("arm");
    group.allowReplanning(true);            //当运动规划失败后，允许重新规划
    group.setGoalJointTolerance(0.001);
    group.setGoalPositionTolerance(0.001);  //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    group.setGoalOrientationTolerance(0.01);

    ViewPlan vp;

    // 测量环境可视化
    visEnv(file_path_small, vp);
    
    // 计算候选视点
    vector<ViewPoint> cand_view_point = vp.generateViewPoint(file_path, sampleNum, coverage_rate, group);   // 候选视点
    visCandidateViewPoint(cand_view_point, vp);  // 候选视点可视化
    
    // 筛选最优视点并进行测量路径规划
    // 随机密钥遗传算法求解
    // RKGA scp_solver(pop, pop_elite, pop_mutant, rhoe, coverage_rate, cand_view_point, vp.g, vp.visibility_matrix);
    // vector<ViewPoint> best_view_point = scp_solver.solveRKGA(maxGen); // 最优视点

    // 马尔科夫决策过程+蒙特卡洛法求解
    MCST mcst_solver(coverage_rate, cand_view_point, vp.g, vp.visibility_matrix); 
    vector<ViewPoint> best_view_point = mcst_solver.solveMCST();      // 最优视点

    visBestViewPoint(best_view_point, vp);                            // 最佳视点与运动路径可视化

    // 规划机器人运动轨迹并控制机器人运动
    cout << "共获得" << best_view_point.size() << "个最佳视点，开始规划机器人运动轨迹：" << endl;
    group.setStartStateToCurrentState();    // 将机器人的初始状态设置为当前状态

    for (int i = 0; i < best_view_point.size(); i++){
        // 从初始位置到第一个视点的轨迹单独规划
        if(i == 0){
            group.setJointValueTarget(best_view_point[i].joint_state);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
            ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
            visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            visual_tools.trigger();

            //让机械臂按照规划的轨迹开始运动。
            if (success)
                group.execute(my_plan.trajectory_);
            continue;
        }

        cout << i << " (" << best_view_point[i].robot_position.m_floats[0] << "," 
                          << best_view_point[i].robot_position.m_floats[1] << "," 
                          << best_view_point[i].robot_position.m_floats[2] << ")" 
                          << best_view_point[i].num << endl;

        // 以当前视点为运动目标
        // group.setJointValueTarget(best_view_point[i].joint_state);

        // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

        // 从视点图中搜索获取预先计算好的前一个视点到当前视点的运动轨迹，直接调用
        moveit_msgs::RobotTrajectory traj;
        if (i != 0){
            ViewPoint *curr = vp.g->edges[best_view_point[i-1].num];
            while (curr->num != best_view_point[i].num){
                curr = curr->next;
            }
            traj = curr->traj;
        }

        // 机器人运动轨迹可视化
        // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group, rvt::BLUE);
        visual_tools.publishTrajectoryLine(traj, joint_model_group);
        visual_tools.trigger();

        // 轨迹运动时间
        cout << "第" << i << "条轨迹时间：";
        int size = traj.joint_trajectory.points.size();
        cout << traj.joint_trajectory.points[size - 1].time_from_start.toSec() << ", " << size << endl;
        // int size1 = my_plan.trajectory_.joint_trajectory.points.size();
        // cout << my_plan.trajectory_.joint_trajectory.points[size1 - 1].time_from_start.toSec() << ", " << size1 << endl;

        //让机械臂按照规划的轨迹开始运动。
        // group.execute(my_plan.trajectory_);
        group.execute(traj);
    }
    ros::shutdown(); 
    return 0;
}

void visEnv(const char *file_path_small, const ViewPlan &vp){
    // 在Rviz中添加待测物体与平台模型
    cout << "开始添加场景模型：" << endl;
    moveit_msgs::CollisionObject obj;
    obj.header.frame_id = "base_link";
    obj.id = "test_turning_bin_07";
    // 平台
    moveit_msgs::CollisionObject table;
    table.header.frame_id = "base_link";
    table.id = "table";

    // 导入待测物体STL网格模型
    shape_msgs::Mesh model_mesh;
    shapes::Mesh* mesh_ptr=shapes::createMeshFromResource(file_path_small);
    if(mesh_ptr == NULL){
        cout << "模型导入错误！" << endl;
    }
    shapes::ShapeMsg model_mesh_msg;
    shapes::constructMsgFromShape(mesh_ptr, model_mesh_msg);
    model_mesh = boost::get<shape_msgs::Mesh>(model_mesh_msg);

    // 设置平台的外形属性
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.42;  // x
    primitive.dimensions[1] = 0.37;  // y
    primitive.dimensions[2] = vp.getModelPositionZ();  // z
    // primitive.dimensions[2] = vp.getModelPositionZ() - 0.02;  //z = 转向节中心高度 - 转向节下半部分高度0.02

    // 设置待测物体和平台位置
    geometry_msgs::Pose model_pose;
    model_pose.orientation.w = 0;
    model_pose.orientation.x = 0;
    model_pose.orientation.y = 0;
    model_pose.orientation.z = 0;
    model_pose.position.x = vp.getModelPositionX();
    model_pose.position.y = 0;
    model_pose.position.z = vp.getModelPositionZ();  

    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 0;
    table_pose.orientation.x = 0;
    table_pose.orientation.y = 0;
    table_pose.orientation.z = 0;
    table_pose.position.x = vp.getTablePositionX();
    table_pose.position.y = 0;
    table_pose.position.z = primitive.dimensions[2] / 2;

    //将物体添加到场景并发布
    obj.meshes.push_back(model_mesh);
    obj.mesh_poses.push_back(model_pose);
    obj.operation = obj.ADD;

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;

    cout<<"成功添加物体"<<endl;

    //发布消息
    // ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.3);
    while (env_vis_pub.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(obj);
    planning_scene.world.collision_objects.push_back(table);
    planning_scene.is_diff = true;
    env_vis_pub.publish(planning_scene);
}

void visCandidateViewPoint(const vector<ViewPoint> cand_view_point, const ViewPlan &vp){
    visualization_msgs::Marker points;
    int id = 0;
    points.header.frame_id    = "base_link";
    points.header.stamp       = ros::Time::now();
    points.ns                 = "candidate_vp";
    points.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.pose.orientation.x = 0.0;
    points.pose.orientation.y = 0.0;
    points.pose.orientation.z = 0.0;
    points.id                 = id;
    points.type               = visualization_msgs::Marker::SPHERE_LIST;
    
    points.scale.x = 0.02;
    points.scale.y = 0.02;
    points.scale.z = 0.02;
    points.color.a = 0.9;
    points.color.r = 0.0;
    points.color.g = 0.5;
    points.color.b = 1.0;

    for (int i = 0; i < cand_view_point.size(); ++i){
        geometry_msgs::Point p;
        p.x = cand_view_point[i].position.m_floats[0] / 1000 + vp.getModelPositionX();
        p.y = cand_view_point[i].position.m_floats[1] / 1000;
        p.z = cand_view_point[i].position.m_floats[2] / 1000 + vp.getModelPositionZ();

        points.points.push_back(p);
    }

    view_point_vis_pub.publish(points);
}

void visBestViewPoint(const vector<ViewPoint> best_view_point, const ViewPlan &vp){
    visualization_msgs::Marker points, line_list;
    int id = 1;
    points.header.frame_id    = line_list.header.frame_id    = "base_link";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "best_vp";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;
    points.id                 = line_list.id                 = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;
    
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.scale.z = 0.05;
    points.color.a = 1.0;
    points.color.r = 0.3;
    points.color.g = 0.0;
    points.color.b = 1.0;

    line_list.scale.x = 0.03;
    line_list.scale.y = 0.03;
    line_list.scale.z = 0.03;
    line_list.color.a = 0.5;
    line_list.color.r = 0.3;
    line_list.color.g = 0.0;
    line_list.color.b = 0.8;

    line_list.points.clear();

    for (int i = 0; i < best_view_point.size(); ++i){
        geometry_msgs::Point p;
        p.x = best_view_point[i].position.m_floats[0] / 1000 + vp.getModelPositionX();
        p.y = best_view_point[i].position.m_floats[1] / 1000;
        p.z = best_view_point[i].position.m_floats[2] / 1000 + vp.getModelPositionZ();

        points.points.push_back(p);

        if(i < (best_view_point.size() - 1)){
            geometry_msgs::Point p_line;
            p_line = p;
            line_list.points.push_back(p_line);
            p_line.x = best_view_point[i + 1].position.m_floats[0] / 1000 + vp.getModelPositionX();
            p_line.y = best_view_point[i + 1].position.m_floats[1] / 1000;
            p_line.z = best_view_point[i + 1].position.m_floats[2] / 1000 + vp.getModelPositionZ();
            line_list.points.push_back(p_line);
        }
    }
    view_point_vis_pub.publish(points);
    sleep(0.5);
    view_point_vis_pub.publish(line_list);
    // sleep(0.5);

    visualization_msgs::Marker dir;
    id = 2;
    dir.header.frame_id    = "base_link";
    dir.header.stamp       = ros::Time::now();
    dir.ns                 = "best_vp_dir";
    dir.action             = visualization_msgs::Marker::ADD;
    dir.pose.orientation.w = 1.0;
    dir.id                 = id;
    dir.type               = visualization_msgs::Marker::ARROW;
    
    dir.color.a = 0.9;
    dir.color.r = 0.3;
    dir.color.g = 0.0;
    dir.color.b = 1.0;

    for (int i = 0; i < best_view_point.size(); ++i){
        dir.scale.x = 0.01;
        dir.scale.y = 0.02;
        dir.scale.z = 0;
        dir.points.clear();
        geometry_msgs::Point p2;
        p2.x = 0.0;
        p2.y = 0.0;
        p2.z = 0.0;
        dir.points.push_back(p2);
        p2.x = 0.5 * best_view_point[i].direction.m_floats[0];
        p2.y = 0.5 * best_view_point[i].direction.m_floats[1];
        p2.z = 0.5 * best_view_point[i].direction.m_floats[2];
        dir.points.push_back(p2);

        dir.pose.position.x = best_view_point[i].position.m_floats[0] / 1000 + vp.getModelPositionX();
        dir.pose.position.y = best_view_point[i].position.m_floats[1] / 1000;
        dir.pose.position.z = best_view_point[i].position.m_floats[2] / 1000 + vp.getModelPositionZ();
        
        view_point_vis_pub.publish(dir);
        dir.id++;
        sleep(0.5);
    }
}
