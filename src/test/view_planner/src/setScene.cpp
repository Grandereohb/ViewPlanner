#include "ViewPlan.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "setScene");
    ros::NodeHandle node_handle; 
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    ViewPlan vp;
    const char* file_path = "package://view_planner/model/test_model_small.stl";  // 模型文件路径

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
    shapes::Mesh* mesh_ptr=shapes::createMeshFromResource(file_path);
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

    ros::spin();

    return 0;
}
