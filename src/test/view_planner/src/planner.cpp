#include "ViewPlan.h"
#include "RKGA.h"
#include <ros/ros.h>
#include <cstring>
#include <vector>

int main(int argc, char **argv)
{
    // 使用前请根据需求修改以下参数 
    const char* file_path = "/home/ros/abb_ws/src/test/view_planner/model/test_model.stl";  // 用于视点生成的模型文件路径
    const char* file_path_small = "package://view_planner/model/test_model_small.stl";  // 用于构建场景的模型文件路径
    int sampleNum = 20;  // 采样次数;
    double coverage_rate = 0.6;  // 采样覆盖率
    // RKGA参数
    int maxGen = 100; // 最大进化代数
    int pop = 50;       // 每代个体样本数
    double pop_elite = 0.1;    // 每代种群中的精英个体比例
    double pop_mate = 0.8;   // 每代种群中交叉的个体比例
    double rhoe = 80;  // probability that an offspring inherits the allele of its elite parent

    ros::init(argc, argv, "planner");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group("arm");
    ViewPlan vp;

    {
    // 在Rviz中添加待测物体与平台模型
	cout<<"开始添加场景模型："<<endl;
    moveit_msgs::CollisionObject obj;
    obj.header.frame_id="base_link";
    obj.id="test_turning_bin_07";
    // 平台
    moveit_msgs::CollisionObject table;
    table.header.frame_id="base_link";
    table.id="table";
    
    // 导入待测物体STL网格模型
    shape_msgs::Mesh model_mesh;
    shapes::Mesh* mesh_ptr=shapes::createMeshFromResource(file_path_small);
    if(mesh_ptr==NULL){
        cout<<"模型导入错误！"<<endl;
    }
    shapes::ShapeMsg model_mesh_msg;
    shapes::constructMsgFromShape(mesh_ptr,model_mesh_msg);
    model_mesh=boost::get<shape_msgs::Mesh>(model_mesh_msg); 

    // 设置平台的外形属性
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;  //x
    primitive.dimensions[1] = 0.2;  //y
    // primitive.dimensions[2] = vp.getModelPositionZ() - 0.02;  //z = 转向节中心高度 - 转向节下半部分高度0.02
    primitive.dimensions[2] = vp.getModelPositionZ();  //z = 转向节中心高度 - 转向节下半部分高度0.02

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
    table_pose.position.x = vp.getModelPositionX();
    table_pose.position.y = 0;
    table_pose.position.z = primitive.dimensions[2] / 2;  // 高度=0.2/2

    //将物体添加到场景并发布
    obj.meshes.push_back(model_mesh);
    obj.mesh_poses.push_back(model_pose);
    obj.operation = obj.ADD;

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;

    cout<<"成功添加物体"<<endl;

    //发布消息
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(obj);
    planning_scene.world.collision_objects.push_back(table);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    }

    // 计算视点，规划并执行机器人轨迹
    vector<ViewPoint> candViewPoint = vp.generateViewPoint(file_path, sampleNum, coverage_rate);   // 候选视点
    
    RKGA scp_solver(pop, pop_elite, pop_mate, rhoe, coverage_rate, candViewPoint, vp.g, vp.visibility_matrix);
    vector<ViewPoint> best_view_point = scp_solver.solveRKGA(maxGen); // 最优视点

    cout<<"共获得"<<best_view_point.size()<<"个最佳视点，开始规划机器人运动轨迹："<<endl;

    for (int i = 0; i < best_view_point.size(); i++)
    {
        geometry_msgs::Pose target_pose1;
        target_pose1.orientation.w = best_view_point[i].quaternion.w(); 
        target_pose1.orientation.x = best_view_point[i].quaternion.x();
        target_pose1.orientation.y = best_view_point[i].quaternion.y();
        target_pose1.orientation.z = best_view_point[i].quaternion.z();

        target_pose1.position.x = vp.getModelPositionX() + best_view_point[i].position.m_floats[0] / 1000;
        target_pose1.position.y = best_view_point[i].position.m_floats[1] / 1000;
        target_pose1.position.z = vp.getModelPositionZ() + best_view_point[i].position.m_floats[2] / 1000;
        group.setPoseTarget(target_pose1);

        cout << i << " (" << target_pose1.position.x << ", " << target_pose1.position.y << ", " << target_pose1.position.z << ")" << endl;

        // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

        ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        //让机械臂按照规划的轨迹开始运动。
        if (success)
            group.execute(my_plan);
    }
    ros::shutdown(); 
    return 0;
}