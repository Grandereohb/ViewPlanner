#include "ViewPlan.h"
#include "RKGA.h"
#include <ros/ros.h>
#include <cstring>
#include <vector>

int main(int argc, char **argv)
{
    // 使用前请根据需求修改以下参数 
    const char* file_path = "/home/ros/abb_ws/src/test/view_planner/model/test_model.stl";  // 模型文件路径
    int sampleNum = 20;  // 采样次数;
    double coverage_rate = 0.6;  // 采样覆盖率
    // RKGA参数
    int maxGen = 200; // 最大进化代数
    int p = 50;       // 每代个体样本数
    double pe = 5;    // 每代种群中的精英个体数
    double pm = 40;   // 每代种群中交叉的个体数
    double rhoe = 5;  // probability that an offspring inherits the allele of its elite parent
    int K = 50;       // number of independent parallel populations
    int MAX = 50;     // number of threads for parallel decoding

    ros::init(argc, argv, "planner");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group("arm");
    ViewPlan vp;

    // 计算视点，规划并执行机器人轨迹
    vector<ViewPoint> candViewPoint = vp.generateViewPoint(file_path, sampleNum, coverage_rate);   // 候选视点
    
    RKGA scp_solver(p, pe, pm, rhoe, MAX, coverage_rate, candViewPoint, vp.g, vp.visibility_matrix);
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