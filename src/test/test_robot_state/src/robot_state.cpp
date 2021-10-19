#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_state");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    //kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;

    //const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link_6");
    Eigen::Isometry3d end_effector_state = Eigen::Isometry3d::Identity();

  /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    Eigen::Matrix3d rotMatrix;
    Eigen::Vector3d vectorBefore(1, 0, 0);
    Eigen::Vector3d vectorAfter(2, 3, 4);
    rotMatrix = Eigen::Quaterniond::FromTwoVectors(vectorBefore, vectorAfter).toRotationMatrix();

    Eigen::Vector3d translation(1,-0.7677,0.06234);
    end_effector_state.rotate(rotMatrix);
    end_effector_state.pretranslate(translation);
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    double timeout = 0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);
    if (found_ik){
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }
    else{
        ROS_INFO("Did not find IK solution");
    }

    return 0;
}
