#include "ViewPlan.h"
#include "RKGA.h"
#include "MCST.h"

#include <ros/ros.h>
#include <cstring>
#include <vector>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

// 在RViz环境中发布实验环境/视点位置/机器人运动轨迹/表面片/每个视点的可见区域
ros::Publisher env_vis_pub, view_point_vis_pub, patch_vis_pub, view_area_vis_pub;
void visEnv(const char *file_path_small, const ViewPlan &vp);
void visCandidateViewPoint(const vector<ViewPoint> &cand_view_point, const ViewPlan &vp);
void visBestViewPoint(const vector<ViewPoint> &best_view_point, const ViewPlan &vp);
void visPatch(const vector<int>& uncovered_patch, const ViewPlan& vp);
void visViewArea(const string vis_file_path, const ViewPlan& vp);
// 读写视点/可见性矩阵/轨迹/表面片信息
void writeData(const char* viewpoint_file,
               const vector<ViewPoint>& cand_view_point,
               const vector<vector<ViewPoint>>& graph,
               const vector<vector<int>>& visibility_matrix);
void writeData(const char* viewpoint_file,
               const vector<vector<moveit_msgs::RobotTrajectory>> trajs);
void writeData(const char *viewpoint_file, const vector<ViewPoint> &best_view_point);
void writeData(const char *patch_file, const vector<TriSurface> &model);
void writeData(const char* patch_file, const vector<int>& uncovered_patch);
void writeVisModel(string vis_file,
                   const vector<ViewPoint>& best_view_point,
                   const vector<vector<int>>& visibility_matrix,
                   const vector<TriSurface>& model);
void readData(const char* viewpoint_file,
              vector<ViewPoint>& cand_view_point,
              vector<vector<ViewPoint>>& graph,
              vector<vector<int>>& visibility_matrix);
void readData(const char* viewpoint_file,
              vector<vector<moveit_msgs::RobotTrajectory>>& trajs);
void readData(const char *viewpoint_file, vector<ViewPoint> &best_view_point);
void readData(const char *patch_file, vector<TriSurface> &model);
void readData(const char *patch_file, vector<int>& uncovered_patch);

int main(int argc, char **argv){
    // ROS初始化
    ros::init(argc, argv, "planner");
    ros::NodeHandle node_handle("~"); 
    ros::NodeHandle nh; 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化参数
    // 输入模型与输出视点文件
    string _file_path, _file_path_small, _vis_file_path, _viewpoint_file, _trajectory_file, _bestvp_file;
    // 模型表面片文件
    string _model_file, _uncovered_file;
    node_handle.getParam("file_path", _file_path);
    node_handle.getParam("file_path_small", _file_path_small);
    node_handle.getParam("vis_file_path", _vis_file_path);
    node_handle.getParam("viewpoint_file", _viewpoint_file);
    node_handle.getParam("trajectory_file", _trajectory_file);
    node_handle.getParam("bestvp_file", _bestvp_file);
    node_handle.getParam("model_file", _model_file);
    node_handle.getParam("uncovered_file", _uncovered_file);

    const char* file_path       = _file_path.data();        // 用于视点生成的模型文件路径
    const char* file_path_small = _file_path_small.data();  // 用于构建场景的模型文件路径
    const char* viewpoint_file  = _viewpoint_file.data();   // 存储候选视点信息的文件名
    const char* trajectory_file = _trajectory_file.data();  // 存储轨迹信息的文件名
    const char* bestvp_file     = _bestvp_file.data();      // 存储最优视点信息的文件名
    const char* model_file      = _model_file.data();       // 存储模型表面片坐标信息的文件名
    const char* uncovered_file  = _uncovered_file.data();   // 存储未覆盖表面片信息的文件名

    // 采样参数(default)
    int sampleNum        = 20;     // 采样候选视点个数
    double coverage_rate = 0.6;    // 要求的采样覆盖率
    node_handle.getParam("sample_num", sampleNum);
    node_handle.getParam("coverage_rate", coverage_rate);

    // RKGA参数(default)
    int maxGen        = 400;  // 最大进化代数
    int pop           = 200;  // 每代个体样本数
    double pop_elite  = 0.1;  // 每代种群中的精英个体比例
    double pop_mutant = 0.3;  // 每代种群中变异的个体比例
    double rhoe       = 70;   // probability that an offspring inherits the allele of its elite parent
    node_handle.getParam("max_gen", maxGen);
    node_handle.getParam("pop", pop);
    node_handle.getParam("pop_elite", pop_elite);
    node_handle.getParam("pop_mutant", pop_mutant);
    node_handle.getParam("rhoe", rhoe);
    
    // 重用与优化方法选择
    bool reuse_best;  // 是否重用最佳视点
    bool reuse;       // 是否重用候选视点
    int optm_type;    // 选择优化方法
    node_handle.getParam("reuse_best", reuse_best);
    node_handle.getParam("reuse", reuse);
    node_handle.getParam("optm_type", optm_type);

    // 定义添加环境与视点模型的消息发布端，定义轨迹可视化接口
    env_vis_pub        = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    view_point_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_view_point", 1);
    patch_vis_pub      = nh.advertise<visualization_msgs::Marker>("vis_patch", 1);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link", "vis_traj");
    visual_tools.deleteAllMarkers();

    // 定义moveit规划接口group，关节模型指针和视点规划类vp
    moveit::planning_interface::MoveGroupInterface group("arm");
    const moveit::core::JointModelGroup* joint_model_group =
        group.getCurrentState()->getJointModelGroup("arm");
    group.allowReplanning(true);  //当运动规划失败后，允许重新规划
    group.setGoalJointTolerance(0.001);
    group.setGoalPositionTolerance(0.001);  //设置位置(单位:米)和姿态（单位:弧度）的允许误差
    group.setGoalOrientationTolerance(0.01);

    ViewPlan vp;

    // 测量环境可视化
    visEnv(file_path_small, vp);
    
    // 计算候选视点
    // reuse = true: 直接读取已存储的候选视点和轨迹文件
    // reuse = false: 调用函数生成新的候选视点与轨迹，写入并覆盖文件
    vector<ViewPoint> cand_view_point;
    if (reuse_best || reuse){
        readData(viewpoint_file, cand_view_point, vp.g.graph, vp.visibility_matrix);
        readData(trajectory_file, vp.trajs);
        readData(model_file, vp.model);
    }
    else{
        cand_view_point = vp.generateViewPoint(file_path, sampleNum, coverage_rate, group); // 候选视点
        writeData(viewpoint_file, cand_view_point, vp.g.graph, vp.visibility_matrix);
        writeData(trajectory_file, vp.trajs);
        writeData(model_file, vp.model);
    }
    visCandidateViewPoint(cand_view_point, vp);  // 候选视点可视化
    
    // 筛选最优视点并进行测量路径规划
    vector<ViewPoint> best_view_point;
    vector<int> uncovered_patch;
    if (reuse_best) {
        // 复用存储的最优视点
        readData(bestvp_file, best_view_point);
        readData(uncovered_file, uncovered_patch);
    } else {
        // optm_type = 0: RKGA
        // optm_type = 1: MCTS
        if (optm_type == 0){
            // 随机密钥遗传算法求解
            RKGA scp_solver(pop, pop_elite, pop_mutant, rhoe, coverage_rate,
                            cand_view_point, vp.g, vp.visibility_matrix);
            best_view_point = scp_solver.solveRKGA(maxGen);  // 最优视点
        }
        else{
            // 马尔科夫决策过程 + 蒙特卡洛法求解
            MCST mcst_solver(coverage_rate, cand_view_point, vp.g.graph,
                             vp.visibility_matrix, node_handle);
            best_view_point = mcst_solver.solveMCST();
            mcst_solver.getUncoveredPatch(uncovered_patch, best_view_point);
        }
        writeData(bestvp_file, best_view_point);
        writeData(uncovered_file, uncovered_patch);
    }
    visBestViewPoint(best_view_point, vp);  // 最佳视点与运动路径可视化
    visPatch(uncovered_patch, vp);
    writeVisModel(_vis_file_path, best_view_point, vp.visibility_matrix, vp.model);

    // 规划机器人运动轨迹并控制机器人运动
    cout << "共获得" << best_view_point.size()
         << "个最佳视点，开始规划机器人运动轨迹: " << endl;
    group.setStartStateToCurrentState();  // 将机器人的初始状态设置为当前状态

    ros::Time start_time;  // 机器人实际运动计时器
    start_time = ros::Time::now();
    double total_time;     // 规划每段轨迹的时间和

    for (int i = 0; i < best_view_point.size(); i++){
        // 从初始位置到第一个视点的轨迹单独规划
        if(i == 0){
            group.setJointValueTarget(best_view_point[i].joint_state);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
            ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
            visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            visual_tools.trigger();
            visViewArea(string("package://view_planner/model/vis") + "/" + to_string(i) + ".stl", vp);

            //让机械臂按照规划的轨迹开始运动。
            if (success)
                group.execute(my_plan.trajectory_);
            continue;
        }

        // 以当前视点为运动目标
        // group.setJointValueTarget(best_view_point[i].joint_state);

        // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
        // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

        // 从视点图中搜索获取预先计算好的前一个视点到当前视点的运动轨迹，直接调用
        moveit_msgs::RobotTrajectory traj;
        traj = vp.trajs[best_view_point[i - 1].num][best_view_point[i].num];

        // int tmp;
        // cin >> tmp;

        // 机器人运动轨迹可视化
        // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group, rvt::BLUE);
        visual_tools.publishTrajectoryLine(traj, joint_model_group);
        visual_tools.trigger();
        visViewArea(string("package://view_planner/model/vis") + "/" + to_string(i) + ".stl", vp);

        // 轨迹运动时间
        cout << "第" << i << "条轨迹时间: ";
        int size = traj.joint_trajectory.points.size();
        cout << traj.joint_trajectory.points[size - 1].time_from_start.toSec()
             << ", " << size << endl;
        total_time +=
            traj.joint_trajectory.points[size - 1].time_from_start.toSec();
        // int size1 = my_plan.trajectory_.joint_trajectory.points.size();
        // cout << my_plan.trajectory_.joint_trajectory.points[size1 - 1].time_from_start.toSec() << ", " << size1 << endl;

        //让机械臂按照规划的轨迹开始运动。
        // group.execute(my_plan.trajectory_);
        group.execute(traj);
    }
    ros::Time end_time = ros::Time::now();
    cout << "机器人运动 + 拍摄总时间为: " << (end_time - start_time).toSec();
    cout << ", 规划得到的所有轨迹的运动时间总和: " << total_time << endl;
    ros::shutdown();
    return 0;
}

void visEnv(const char *file_path_small, const ViewPlan &vp){
    // 在Rviz中添加待测物体与平台模型
    cout << "开始添加场景模型: " << endl;
    moveit_msgs::CollisionObject obj;
    obj.header.frame_id = "base_link";
    obj.id = "sheet";
    // 平台
    moveit_msgs::CollisionObject table;
    table.header.frame_id = "base_link";
    table.id = "table";
    // 墙面
    moveit_msgs::CollisionObject wall_1;
    wall_1.header.frame_id = "base_link";
    wall_1.id = "wall_1";
    

    moveit_msgs::CollisionObject wall_2;
    wall_2.header.frame_id = "base_link";
    wall_2.id = "wall_2";

    // 导入待测物体STL网格模型
    shape_msgs::Mesh model_mesh;
    shapes::Mesh* mesh_ptr=shapes::createMeshFromResource(file_path_small);
    if(mesh_ptr == NULL){
        cout << "模型导入错误!" << endl;
    }
    shapes::ShapeMsg model_mesh_msg;
    shapes::constructMsgFromShape(mesh_ptr, model_mesh_msg);
    model_mesh = boost::get<shape_msgs::Mesh>(model_mesh_msg);

    // 设置平台的外形属性
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.4;  // x
    primitive.dimensions[1] = 0.3;  // y
    primitive.dimensions[2] = vp.getModelPositionZ();  // z
    // primitive.dimensions[2] = vp.getModelPositionZ() - 0.02;  //z = 转向节中心高度 - 转向节下半部分高度0.02

    // 设置墙面的外形属性
    shape_msgs::SolidPrimitive primitive_wall_1;
    primitive_wall_1.type = primitive_wall_1.BOX;
    primitive_wall_1.dimensions.resize(3);
    primitive_wall_1.dimensions[0] = 0.1;
    primitive_wall_1.dimensions[1] = 2.0;  
    primitive_wall_1.dimensions[2] = 2.0;

    shape_msgs::SolidPrimitive primitive_wall_2;
    primitive_wall_2.type = primitive_wall_2.BOX;
    primitive_wall_2.dimensions.resize(3);
    primitive_wall_2.dimensions[0] = 2.0;
    primitive_wall_2.dimensions[1] = 0.4;  
    primitive_wall_2.dimensions[2] = 0.8;

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

    geometry_msgs::Pose wall_pose;
    wall_pose.orientation.w = 0;
    wall_pose.orientation.x = 0;
    wall_pose.orientation.y = 0;
    wall_pose.orientation.z = 0;
    wall_pose.position.x = -1.0;
    wall_pose.position.y = 0;
    wall_pose.position.z = primitive_wall_1.dimensions[2] / 2;

    geometry_msgs::Pose wall_pose_2;
    wall_pose_2.orientation.w = 0;
    wall_pose_2.orientation.x = 0;
    wall_pose_2.orientation.y = 0;
    wall_pose_2.orientation.z = 0;
    wall_pose_2.position.x = 0;
    wall_pose_2.position.y = -1.0;
    wall_pose_2.position.z = primitive_wall_2.dimensions[2] / 2;

    // 设置颜色
    vector<moveit_msgs::ObjectColor> colors;
    moveit_msgs::ObjectColor color_obj;
    color_obj.color.a = 0.5;
    color_obj.color.r = 0.6;
    color_obj.color.g = 0.6;
    color_obj.color.b = 0.6;
    color_obj.id = "sheet";

    moveit_msgs::ObjectColor color_table;
    color_table.color.a = 1.0;
    color_table.color.r = 0.588;
    color_table.color.g = 0.3;
    color_table.color.b = 0.0;
    color_table.id = "table";

    moveit_msgs::ObjectColor color_0;
    color_0.color.a = 0.6;
    color_0.color.r = 0.5;
    color_0.color.g = 0.5;
    color_0.color.b = 0.5;
    color_0.id = "wall_1";

    moveit_msgs::ObjectColor color_1;
    color_1.color.a = 0.6;
    color_1.color.r = 0.5;
    color_1.color.g = 0.5;
    color_1.color.b = 0.5;
    color_1.id = "wall_2";
    colors.push_back(color_1);

    //将物体添加到场景并发布
    obj.meshes.push_back(model_mesh);
    obj.mesh_poses.push_back(model_pose);
    obj.operation = obj.ADD;

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;

    wall_1.primitives.push_back(primitive_wall_1);
    wall_1.primitive_poses.push_back(wall_pose);
    wall_1.operation = wall_1.ADD;

    wall_2.primitives.push_back(primitive_wall_2);
    wall_2.primitive_poses.push_back(wall_pose_2);
    wall_2.operation = wall_2.ADD;

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
    planning_scene.object_colors.push_back(color_obj);
    planning_scene.world.collision_objects.push_back(table);
    planning_scene.object_colors.push_back(color_table);
    planning_scene.world.collision_objects.push_back(wall_1);
    planning_scene.object_colors.push_back(color_0);
    planning_scene.world.collision_objects.push_back(wall_2);
    planning_scene.object_colors.push_back(color_1);

    planning_scene.is_diff = true;
    env_vis_pub.publish(planning_scene);
}

void visCandidateViewPoint(const vector<ViewPoint> &cand_view_point, const ViewPlan &vp){
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

void visBestViewPoint(const vector<ViewPoint> &best_view_point, const ViewPlan &vp){
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

    // 视点方向可视化
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
        id++;
        sleep(0.5);
    }

    // 视点测量区域可视化
    visualization_msgs::Marker vis_area;
    vis_area.header.frame_id    = "base_link";
    vis_area.header.stamp       = ros::Time::now();
    vis_area.ns                 = "best_vp_vis_area";
    vis_area.action             = visualization_msgs::Marker::ADD;
    vis_area.pose.orientation.w = 1.0;
    vis_area.id                 = id;
    vis_area.type               = visualization_msgs::Marker::CYLINDER;
    
    vis_area.color.a = 0.3;
    vis_area.color.r = 0.3;
    vis_area.color.g = 0.0;
    vis_area.color.b = 1.0;

    for (int i = 0; i < best_view_point.size(); ++i){
        vis_area.scale.x = 0.31;
        vis_area.scale.y = 0.31;
        vis_area.scale.z = 0.7;
        vis_area.points.clear();

        vis_area.pose.position.x = best_view_point[i].position.m_floats[0] / 1000 +
                                   vp.getModelPositionX() +
                                   best_view_point[i].direction.m_floats[0] * 0.35;
        vis_area.pose.position.y = best_view_point[i].position.m_floats[1] / 1000 +
                                   best_view_point[i].direction.m_floats[1] * 0.35;
        vis_area.pose.position.z = best_view_point[i].position.m_floats[2] / 1000 +
                                   vp.getModelPositionZ() +
                                   best_view_point[i].direction.m_floats[2] * 0.35;

        view_point_vis_pub.publish(vis_area);
        vis_area.id++;
        sleep(0.5);
    }
}

void visPatch(const vector<int>& uncovered_patch, const ViewPlan& vp) {
    cout << "开始进行表面片可视化:" << endl;
    visualization_msgs::Marker model_points, uncovered_points;
    int id = 50;
    model_points.header.frame_id    = uncovered_points.header.frame_id    = "base_link";
    model_points.header.stamp       = uncovered_points.header.stamp       = ros::Time::now();
    model_points.ns                 = "model_patch";
    uncovered_points.ns             = "uncovered_patch";
    model_points.action             = uncovered_points.action             = visualization_msgs::Marker::ADD;
    model_points.pose.orientation.w = uncovered_points.pose.orientation.w = 1.0;
    model_points.pose.orientation.x = uncovered_points.pose.orientation.x = 0.0;
    model_points.pose.orientation.y = uncovered_points.pose.orientation.y = 0.0;
    model_points.pose.orientation.z = uncovered_points.pose.orientation.z = 0.0;
    model_points.id                 = uncovered_points.id                 = id;
    model_points.type               = uncovered_points.type               = visualization_msgs::Marker::SPHERE_LIST;
    
    model_points.scale.x = 0.002;
    model_points.scale.y = 0.002;
    model_points.scale.z = 0.002;
    uncovered_points.scale.x = 0.004;
    uncovered_points.scale.y = 0.004;
    uncovered_points.scale.z = 0.004;
    model_points.color.a = 0.9;
    model_points.color.r = 0.0;
    model_points.color.g = 0.0;
    model_points.color.b = 1.0;
    uncovered_points.color.a = 1.0;
    uncovered_points.color.r = 1.0;
    uncovered_points.color.g = 0.0;
    uncovered_points.color.b = 0.0;

    for (int i = 0; i < vp.model.size(); ++i){
        geometry_msgs::Point p;
        p.x = vp.model[i].center.m_floats[0] / 1000 + vp.getModelPositionX();
        p.y = vp.model[i].center.m_floats[1] / 1000;
        p.z = vp.model[i].center.m_floats[2] / 1000 + vp.getModelPositionZ();

        model_points.points.push_back(p);
    }
    for (int i = 0; i < uncovered_patch.size(); ++i){
        geometry_msgs::Point p;
        p.x = vp.model[uncovered_patch[i]].center.m_floats[0] / 1000 + vp.getModelPositionX();
        p.y = vp.model[uncovered_patch[i]].center.m_floats[1] / 1000;
        p.z = vp.model[uncovered_patch[i]].center.m_floats[2] / 1000 + vp.getModelPositionZ();

        uncovered_points.points.push_back(p);
    }

    patch_vis_pub.publish(uncovered_points);
    sleep(0.5);
    patch_vis_pub.publish(model_points);

    cout << "共获得" << vp.model.size() << "个表面片, 其中未覆盖表面数为: ";
    cout << uncovered_patch.size() << endl;
    cout << "表面片可视化完成!" << endl;
}

void visViewArea(const string vis_file_path, const ViewPlan& vp){
    // 在Rviz中添加每个视点的可见区域
    moveit_msgs::CollisionObject obj;
    obj.header.frame_id = "base_link";
    obj.id = "vis_area";

    // 导入待测物体STL网格模型
    shape_msgs::Mesh model_mesh;
    shapes::Mesh* mesh_ptr=shapes::createMeshFromResource(vis_file_path);
    if(mesh_ptr == NULL){
        cout << "可见区域导入错误!" << endl;
    }
    shapes::ShapeMsg model_mesh_msg;
    shapes::constructMsgFromShape(mesh_ptr, model_mesh_msg);
    model_mesh = boost::get<shape_msgs::Mesh>(model_mesh_msg);

    // 设置可见区域位置
    geometry_msgs::Pose model_pose;
    model_pose.orientation.w = 0;
    model_pose.orientation.x = 0;
    model_pose.orientation.y = 0;
    model_pose.orientation.z = 0;
    model_pose.position.x = vp.getModelPositionX();
    model_pose.position.y = 0;
    model_pose.position.z = vp.getModelPositionZ(); 

    // 设置颜色
    vector<moveit_msgs::ObjectColor> colors;
    moveit_msgs::ObjectColor color_obj;
    color_obj.color.a = 1.0;
    color_obj.color.r = 1.0;
    color_obj.color.g = 1.0;
    color_obj.color.b = 0.0;
    color_obj.id = "vis_area";

    //将物体添加到场景并发布
    obj.meshes.push_back(model_mesh);
    obj.mesh_poses.push_back(model_pose);
    obj.operation = obj.ADD;

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(obj);
    planning_scene.object_colors.push_back(color_obj);

    planning_scene.is_diff = true;
    env_vis_pub.publish(planning_scene);
}

void writeData(const char *viewpoint_file, const vector<ViewPoint> &cand_view_point,
               const vector<vector<ViewPoint>> &graph, const vector<vector<int>> &visibility_matrix){
    // 用于存储视点生成步骤生成的候选视点集，视点图和可见性矩阵
    // 文件内容格式为:int 候选视点数， 候选视点， int 视点图行， int 视点图列，
    // 视点图， int 可见性矩阵行， int 可见性矩阵列， 可见性矩阵

    ofstream fout(viewpoint_file, ios_base::binary);
    // ofstream fout(viewpoint_file, ios_base::out | ios_base::app | ios_base::binary);
    if(!fout.is_open()){
        cerr << "open error:\n";
        exit(EXIT_FAILURE);
    }
    int m, n;
    // 写入候选视点数据
    n = cand_view_point.size();
    fout.write((char *)&n, sizeof(n));
    for(int i = 0; i < n; ++i){
        fout.write((char *)(&cand_view_point[i].num), sizeof(int));
        fout.write((char *)(&cand_view_point[i].vis_area), sizeof(int));
        for(int j = 0; j < 6; ++j)
            fout.write((char *)(&cand_view_point[i].joint_state[j]), sizeof(double));
        for (int j = 0; j < 3; ++j){
            fout.write((char *)(&cand_view_point[i].position.m_floats[j]), sizeof(double));
            fout.write((char *)(&cand_view_point[i].direction.m_floats[j]), sizeof(double));
        }
    }

    // 写入视点图数据
    m = graph.size(), n = graph[0].size();
    fout.write((char *)&m, sizeof(m));
    fout.write((char *)&n, sizeof(n));  
    for (int i = 0; i < m; ++i){
        for(int j = 0; j < n; ++j)
            fout.write((char *)(&graph[i][j]), sizeof(graph[i][j]));
    }
    int a = sizeof(ViewPoint), b = sizeof(graph[0][1]);

    // 写入可见性矩阵数据
    m = visibility_matrix.size(), n = visibility_matrix[0].size();
    fout.write((char *)&m, sizeof(m));
    fout.write((char *)&n, sizeof(n));
    for (int i = 0; i < m; ++i){
        for(int j = 0; j < n; ++j)
            fout.write((char *)(&visibility_matrix[i][j]), sizeof(visibility_matrix[i][j]));
    }

    fout.close();
}
void writeData(const char *viewpoint_file, const vector<vector<moveit_msgs::RobotTrajectory>> trajs){
    ofstream fout(viewpoint_file, ios_base::binary);
    if(!fout.is_open()){
        cerr << "open error:\n";
        exit(EXIT_FAILURE);
    }
    int m = trajs.size(), n = trajs[0].size();
    fout.write((char *)&m, sizeof(m));
    fout.write((char *)&n, sizeof(n));

    for (int i = 0; i < m; ++i){
        for(int j = 0; j < n; ++j){
            int num_p = trajs[i][j].joint_trajectory.points.size();
            fout.write((char *)&num_p, sizeof(int));
            for (int ip = 0; ip < num_p; ++ip){
                for (int k = 0; k < 6; ++k){
                    double p = trajs[i][j].joint_trajectory.points[ip].positions[k];
                    double v = trajs[i][j].joint_trajectory.points[ip].velocities[k];
                    double a = trajs[i][j].joint_trajectory.points[ip].accelerations[k];
                    fout.write((char *)&p, sizeof(double));
                    fout.write((char *)&v, sizeof(double));
                    fout.write((char *)&a, sizeof(double));
                }
                double time = trajs[i][j].joint_trajectory.points[ip].time_from_start.toSec();
                fout.write((char *)&time, sizeof(double));
            }
        }
    }
    fout.close();
}
void writeData(const char *viewpoint_file, const vector<ViewPoint> &best_view_point){
    // 用于存储最佳视点

    ofstream fout(viewpoint_file, ios_base::binary);
    // ofstream fout(viewpoint_file, ios_base::out | ios_base::app | ios_base::binary);
    if(!fout.is_open()){
        cerr << "open error:\n";
        exit(EXIT_FAILURE);
    }
    int m, n;
    // 写入候选视点数据
    n = best_view_point.size();
    fout.write((char *)&n, sizeof(n));
    for(int i = 0; i < n; ++i){
        fout.write((char *)(&best_view_point[i].num), sizeof(int));
        fout.write((char *)(&best_view_point[i].vis_area), sizeof(int));
        for(int j = 0; j < 6; ++j)
            fout.write((char *)(&best_view_point[i].joint_state[j]), sizeof(double));
        for (int j = 0; j < 3; ++j){
            fout.write((char *)(&best_view_point[i].position.m_floats[j]), sizeof(double));
            fout.write((char *)(&best_view_point[i].direction.m_floats[j]), sizeof(double));
        }
    }

    fout.close();
}
void writeData(const char *patch_file, const vector<TriSurface> &model){
    // 用于存储待测模型 全部 表面片
    ofstream fout(patch_file, ios_base::binary);
    // ofstream fout(viewpoint_file, ios_base::out | ios_base::app | ios_base::binary);
    if(!fout.is_open()){
        cerr << "open error:\n";
        exit(EXIT_FAILURE);
    }
    int n;
    // 写入候选视点数据
    n = model.size();
    fout.write((char *)&n, sizeof(n));
    for(int i = 0; i < n; ++i){
        auto num = sizeof(model[i].center.m_floats[0]);
        for (int j = 0; j < 3; ++j)
            fout.write((char *)(&model[i].center.m_floats[j]), sizeof(double));
        for (int j = 0; j < 3; ++j)
            fout.write((char *)(&model[i].normal.m_floats[j]), sizeof(double));
        for (int j = 0; j < 3; ++j){
            fout.write((char *)(&model[i].vertex[j].m_floats[0]), sizeof(double));
            fout.write((char *)(&model[i].vertex[j].m_floats[1]), sizeof(double));
            fout.write((char *)(&model[i].vertex[j].m_floats[2]), sizeof(double));
        }
    }

    fout.close();
}
void writeData(const char* patch_file, const vector<int>& uncovered_patch){
    // 用于存储待测模型 未被覆盖的 表面片序号
    ofstream fout(patch_file, ios_base::binary);
    // ofstream fout(viewpoint_file, ios_base::out | ios_base::app | ios_base::binary);
    if(!fout.is_open()){
        cerr << "open error:\n";
        exit(EXIT_FAILURE);
    }
    int n;
    // 写入候选视点数据
    n = uncovered_patch.size();
    fout.write((char *)&n, sizeof(n));
    for(int i = 0; i < n; ++i)
        fout.write((char *)(&uncovered_patch[i]), sizeof(int));    

    fout.close();
}
void writeVisModel(string vis_file,
                   const vector<ViewPoint>& best_view_point,
                   const vector<vector<int>>& visibility_matrix,
                   const vector<TriSurface>& model) {
    for (int i = 0; i < best_view_point.size(); ++i){
        vector<TriSurface> vis_patch;
        for (int j = 0; j < model.size(); ++j){
            if(visibility_matrix[best_view_point[i].num][j] == 1)
                vis_patch.push_back(model[j]);
        }
        string filename = vis_file + "/" + to_string(i) + ".stl";
        // Binary
        // ofstream file(filename.c_str(), ios_base::out | ios::binary);
        // if (!file){
        //     cerr << "Error: Unable to open file " << filename << endl;
        //     return;
        // }
        // char header[80] = "STL File";
        // file.write(header, 80);
        // int size = vis_patch.size();
        // file.write((char*)&size, 4);
        // for (int j = 0; j < size; ++j){
        //     file.write((char*)&vis_patch[j].normal.m_floats[0], 4);
        //     file.write((char*)&vis_patch[j].normal.m_floats[1], 4);
        //     file.write((char*)&vis_patch[j].normal.m_floats[2], 4);
        //     for (int k = 0; k < 3; ++k){
        //         file.write((char*)&vis_patch[j].vertex[k].m_floats[0], 4);
        //         file.write((char*)&vis_patch[j].vertex[k].m_floats[1], 4);
        //         file.write((char*)&vis_patch[j].vertex[k].m_floats[2], 4);
        //     }
        //     short val = 0;
        //     file.write((char*)&val, 2);
        // }
        // file.close();

        // ASCII
        ofstream output(filename.c_str(), ios_base::out);
        output << "solid" << filename << endl;
        for (int j = 0; j < vis_patch.size(); ++j){
            output << "   "
                   << "facet normal " << vis_patch[j].normal.m_floats[0] << " "
                   << vis_patch[j].normal.m_floats[1] << " " << vis_patch[j].normal.m_floats[2]
                   << endl;
            output << "      "
                   << "outer loop" << endl;
            for (int k = 0; k < 3; ++k){
                output << "         "
                       << "vertex"
                       << " " << vis_patch[j].vertex[k].m_floats[0] / 1000 << " "
                       << vis_patch[j].vertex[k].m_floats[1] / 1000 << " "
                       << vis_patch[j].vertex[k].m_floats[2] / 1000 << endl;
            }
            output << "      " << "endloop" << endl;
            output << "   " << "endfacet" << endl;
        }
        output << "endsolid" << endl;
        output.close();
    }
}

void readData(const char *viewpoint_file, vector<ViewPoint> &cand_view_point,
              vector<vector<ViewPoint>> &graph, vector<vector<int>> &visibility_matrix){
    ifstream fin;
    fin.open(viewpoint_file, ios_base::in | ios_base::binary);
    if(!fin){
		cout << "读取文件失败" <<endl;
		return;
	}
    if(fin.is_open()){
        int m, n;
        // 读取候选视点
        fin.read((char *)&n, sizeof(int));
        cand_view_point.resize(n);
        for(int i = 0; i < n; ++i){
            fin.read((char *)(&cand_view_point[i].num), sizeof(int));
            fin.read((char *)(&cand_view_point[i].vis_area), sizeof(int));
            cand_view_point[i].joint_state.resize(6);
            for (int j = 0; j < 6; ++j)
                fin.read((char *)(&cand_view_point[i].joint_state[j]), sizeof(double));
            for (int j = 0; j < 3; ++j){
                fin.read((char *)(&cand_view_point[i].position.m_floats[j]), sizeof(double));
                fin.read((char *)(&cand_view_point[i].direction.m_floats[j]), sizeof(double));
            }
        }

        // 读取视点图
        fin.read((char *)&m, sizeof(int));
        fin.read((char *)&n, sizeof(int));
        graph.resize(m);
        for (int i = 0; i < m; ++i){
            graph[i].resize(n);
            for (int j = 0; j < n; ++j)
                fin.read((char *)(&graph[i][j]), sizeof(ViewPoint));
        }

        // 读取可见性矩阵
        fin.read((char *)&m, sizeof(int));
        fin.read((char *)&n, sizeof(int));
        visibility_matrix.resize(m);
        for (int i = 0; i < m; ++i){
            visibility_matrix[i].resize(n);
            for (int j = 0; j < n; ++j)
                fin.read((char *)(&visibility_matrix[i][j]), sizeof(int));
        }

        fin.close();
    }
}
void readData(const char *viewpoint_file, vector<vector<moveit_msgs::RobotTrajectory>> &trajs){
    ifstream fin;
    fin.open(viewpoint_file, ios_base::in | ios_base::binary);
    if(!fin){
		cout << "读取文件失败" <<endl;
		return;
	}
    if(fin.is_open()){
        int m, n;
        fin.read((char *)&m, sizeof(int));
        fin.read((char *)&n, sizeof(int));

        trajs.resize(m);
        for (int i = 0; i < m; ++i){
            trajs[i].resize(n);
            for (int j = 0; j < n; ++j){
                trajs[i][j].joint_trajectory.header.frame_id = "base_link";
                trajs[i][j].joint_trajectory.joint_names = {
                    "joint_1", "joint_2", "joint_3",
                    "joint_4", "joint_5", "joint_6"};

                int num_p = 0;
                fin.read((char *)&num_p, sizeof(int));
                trajs[i][j].joint_trajectory.points.resize(num_p);
                for (int ip = 0; ip < num_p; ++ip){
                    trajs[i][j].joint_trajectory.points[ip].positions.resize(6);
                    trajs[i][j].joint_trajectory.points[ip].velocities.resize(6);
                    trajs[i][j].joint_trajectory.points[ip].accelerations.resize(6);
                    for (int k = 0; k < 6; ++k){
                        double p, v, a;
                        fin.read((char *)&p, sizeof(double));
                        fin.read((char *)&v, sizeof(double));
                        fin.read((char *)&a, sizeof(double));
                        trajs[i][j].joint_trajectory.points[ip].positions[k] = p;
                        trajs[i][j].joint_trajectory.points[ip].velocities[k] = v;
                        trajs[i][j].joint_trajectory.points[ip].accelerations[k] = a;
                    }
                    double time;
                    fin.read((char *)&time, sizeof(double));
                    trajs[i][j].joint_trajectory.points[ip].time_from_start = ros::Duration().fromSec(time);
                }
            }
        }

        fin.close();
    }
}
void readData(const char *viewpoint_file, vector<ViewPoint> &best_view_point){
    ifstream fin;
    fin.open(viewpoint_file, ios_base::in | ios_base::binary);
    if(!fin){
		cout << "读取文件失败" <<endl;
		return;
	}
    if(fin.is_open()){
        int m, n;
        // 读取候选视点
        fin.read((char *)&n, sizeof(int));
        best_view_point.resize(n);
        for(int i = 0; i < n; ++i){
            fin.read((char *)(&best_view_point[i].num), sizeof(int));
            fin.read((char *)(&best_view_point[i].vis_area), sizeof(int));
            best_view_point[i].joint_state.resize(6);
            for (int j = 0; j < 6; ++j)
                fin.read((char *)(&best_view_point[i].joint_state[j]), sizeof(double));
            for (int j = 0; j < 3; ++j){
                fin.read((char *)(&best_view_point[i].position.m_floats[j]), sizeof(double));
                fin.read((char *)(&best_view_point[i].direction.m_floats[j]), sizeof(double));
            }
        }

        fin.close();
    }
}
void readData(const char *patch_file, vector<TriSurface> &model){
    // 读取待测模型全部表面片坐标
    ifstream fin;
    fin.open(patch_file, ios_base::in | ios_base::binary);
    if(!fin){
		cout << "读取文件失败" <<endl;
		return;
	}
    if(fin.is_open()){
        int n;
        // 读取候选视点
        fin.read((char *)&n, sizeof(int));
        model.resize(n);
        for(int i = 0; i < n; ++i){
            for (int j = 0; j < 3; ++j)
                fin.read((char *)(&model[i].center.m_floats[j]), sizeof(double));
            for (int j = 0; j < 3; ++j)
                fin.read((char *)(&model[i].normal.m_floats[j]), sizeof(double));
            model[i].vertex.resize(3);
            for (int j = 0; j < 3; ++j){
                fin.read((char *)(&model[i].vertex[j].m_floats[0]), sizeof(double));
                fin.read((char *)(&model[i].vertex[j].m_floats[1]), sizeof(double));
                fin.read((char *)(&model[i].vertex[j].m_floats[2]), sizeof(double));
            }
        }
        fin.close();
    }
}
void readData(const char *patch_file, vector<int>& uncovered_patch){
    // 读取待测模型 未覆盖的 表面片序号
    ifstream fin;
    fin.open(patch_file, ios_base::in | ios_base::binary);
    if(!fin){
		cout << "读取文件失败" <<endl;
		return;
	}
    if(fin.is_open()){
        int n;
        // 读取候选视点
        fin.read((char *)&n, sizeof(int));
        uncovered_patch.resize(n);
        for(int i = 0; i < n; ++i){
            fin.read((char *)(&uncovered_patch[i]), sizeof(int));
        }
        fin.close();
    }
}
