// 包含miveit的API头文件
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <cstring>
#include <vector>
#include "DataForm.h"

int main(int argc, char **argv)
{
    const char* FilePath = "/home/ros/test_turning_bin_07.stl";

    ros::init(argc, argv, "ViewPlanning_demo");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group("arm");
    //-----------------------------------------------------------------------
    // 添加待测物体和平台模型
    // 待测物体
    moveit_msgs::CollisionObject obj;
    obj.header.frame_id="base_link";
    obj.id="test_turning_bin_07";
    // 平台
    moveit_msgs::CollisionObject table;
    table.header.frame_id="base_link";
    table.id="table";
    
    // 导入待测物体STL网格模型
    shape_msgs::Mesh turning_mesh;
    shapes::Mesh* mesh_ptr=shapes::createMeshFromResource("package://test_planning/model/test_turning_bin_07_smallvol3.stl");
    if(mesh_ptr==NULL){
        cout<<"模型导入错误！"<<endl;
    }
    shapes::ShapeMsg turning_mesh_msg;
    shapes::constructMsgFromShape(mesh_ptr,turning_mesh_msg);
    turning_mesh=boost::get<shape_msgs::Mesh>(turning_mesh_msg); 

    // 设置平台的外形属性
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;  //x
    primitive.dimensions[1] = 0.2;  //y
    primitive.dimensions[2] = 0.3;  //z

    // 设置待测物体和平台位置
    geometry_msgs::Pose turning_pose;
    turning_pose.orientation.w = 0;
    turning_pose.orientation.x = 0;
    turning_pose.orientation.y = 0;
    turning_pose.orientation.z = 0;
    turning_pose.position.x = 0.85;
    turning_pose.position.y = 0;
    turning_pose.position.z = primitive.dimensions[2] + 0.07;  //平台高度0.2 + 转向节下半部分高度0.07

    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 0;
    table_pose.orientation.x = 0;
    table_pose.orientation.y = 0;
    table_pose.orientation.z = 0;
    table_pose.position.x = turning_pose.position.x - 0.05;
    table_pose.position.y = 0;
    table_pose.position.z = primitive.dimensions[2] / 2;  //高度0.2/2

    //将物体添加到场景并发布
    obj.meshes.push_back(turning_mesh);
    obj.mesh_poses.push_back(turning_pose);
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

    //-----------------------------------------------------------------------
    // 计算视点，规划并执行机器人轨迹
    DataForm vp;
    vector<vector<ViewPoint>> resBestViewPoint;  //最优视点

    vp.points = vp.ReadFile(FilePath);
	resBestViewPoint = vp.CalcViewPoint(vp.points); 
    cout<<"共获得"<<resBestViewPoint.size()<<"个最佳视点分区，开始规划机器人运动轨迹："<<endl;
    
    for(int i=0;i<resBestViewPoint.size();i++){
        for(int j=0;j<resBestViewPoint[i].size();j++){
            geometry_msgs::Pose target_pose1;
            target_pose1.orientation.w = 0; //待解决难题：机器人该方向上自由度未确定
            target_pose1.orientation.x = resBestViewPoint[i][j].direction.x;
            target_pose1.orientation.y = resBestViewPoint[i][j].direction.y;
            target_pose1.orientation.z = resBestViewPoint[i][j].direction.z;

            target_pose1.position.x = turning_pose.position.x + resBestViewPoint[i][j].position.x / 1000;
            target_pose1.position.y = resBestViewPoint[i][j].position.y / 1000;
            target_pose1.position.z = turning_pose.position.z + resBestViewPoint[i][j].position.z / 1000;
            group.setPoseTarget(target_pose1);
            
            cout<<i<<"  "<<j<<" ("<<target_pose1.position.x <<", "<<target_pose1.position.y<<", "<<target_pose1.position.z<<")"<<endl;

            // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

            ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   

            //让机械臂按照规划的轨迹开始运动。
            if(success)
                group.execute(my_plan);
        }
    }
    ros::shutdown(); 
    return 0;
}
