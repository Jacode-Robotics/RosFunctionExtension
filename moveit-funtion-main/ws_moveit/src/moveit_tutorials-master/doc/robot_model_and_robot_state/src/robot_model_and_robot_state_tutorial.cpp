#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 开始
  // 设置以开始使用 RobotModel 类非常简单。通常，您会发现大多数高级组件都会返回指向 RobotModel 的共享指针。
  // 您应该尽可能使用它。在此示例中，我们将从这样的共享指针开始，并仅讨论基本 API。
  // 您可以查看这些类的实际代码API，以获取有关如何使用这些类提供的更多功能的更多信息。
  //
  //我们将首先实例化一个 RobotModelLoader 对象，它将在 ROS 参数服务器上查找机器人描述并构建一个 RobotModel供我们使用。
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // 使用RobotModel，我们可以构造一个 RobotState来维护机器人的配置。
  // 我们将状态中的所有关节设置为其默认值。
  // 然后我们可以得到一个 JointModelGroup，它代表特定组的机器人模型，例如 Panda 机器人的“panda_arm”。
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // 获取关节值
  // 我们可以检索Panda arm状态中存储的当前关节值
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // 关节限制
  // setJointGroupPositions() 本身不会执行关节限制, 但是调用 enforceBounds() 可以
  /* 将熊猫手臂的一个关节设置在关节极限之外 */
  joint_values[0] = 5.57;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  /* 检查是否有关节超过关节限制 */
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* 执行此状态的关节限制，然后再次检查*/
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* 正向运动学 */
  // 现在，我们可以计算一组随机关节值的正向运动学。请注意，我们想要找到“panda_link8”的姿势，它是机器人“panda_arm”组中最远端的连杆
  kinematic_state->setToRandomPositions(joint_model_group);    // 将关节组设置为随机位置
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");

  /* 打印末端效应器姿势。记住，这是在模型框架中 */
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  /* 逆运动学 */
  // 现在我们可以求解 Panda 机器人的逆运动学 (IK)。要求解 IK，我们需要以下内容：
  //
  //  * 末端执行器所需的姿势（默认情况下，这是“panda_arm”链中的最后一个连杆）：我们在上面的步骤中计算的 end_effector_state。
  //  * 超时：0.1 秒
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // 现在，我们可以打印出 IK 解决方案（如果找到的话）：
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  /* 获取雅可比矩阵 */
  // 我们还可以从RobotState中获取雅可比矩阵。
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
