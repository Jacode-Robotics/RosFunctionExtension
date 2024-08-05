#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

const double tau = 2 * M_PI;          // 定义常量2pi

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);           // ROS旋转必须运行，以便MoveGroupInterface获取有关机器人状态的信息。One way to do this is to start an AsyncSpinner beforehand.
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";                                // MoveIt对关节组"planning groups"进行操作，并将它们存储在`JointModelGroup`对象中。// 在MoveIt中，术语"planning group"和"joint model group"可以互换使用

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);  // 只需使用您要控制和规划的规划组的名称，就可以轻松设置MoveGroupInterface类。// MoveGroupInterface: 客户端类，方便使用move_group节点提供的ROS接口。
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;          // PlanningSceneInterface类可以在虚拟环境中添加和删除碰撞对象

  const moveit::core::JointModelGroup* joint_model_group =                              // 原始指针经常用于引用规划组以提高性能。
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  /* 可视化 */ 
  namespace rvt = rviz_visual_tools;           // MoveItVisualTools包提供了许多在RViz中可视化对象、机器人和轨迹的功能，以及调试工具（例如脚本的逐步自省）。
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();             // 删除所有标记

  visual_tools.loadRemoteControl();            // 远程控制是一种自省工具，允许用户通过 RViz 中的按钮和键盘快捷键逐步执行高级脚本

  // RViz 提供了多种类型的标记，在本演示中我们将使用文本、圆柱体和球体
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();    // 批量发布用于减少发送到 RViz 进行大型可视化的消息数量

  /* 获取基本信息 */ 
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());  // 打印机器人的参考坐标系的名称
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());  // 打印该组的末端执行器连杆名称

  // 获取机器人中所有组的列表
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  /* 开始demo */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  /* 规划到目标姿势 */
  // 我们可以为该组规划一个运动，以使末端执行器达到所需的姿势。
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group_interface.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;    // 现在，我们调用规划器来计算计划并将其可视化。请注意，我们只是在规划，而不是要求 move_group_interface 实际移动机器人。

  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // 可视化规划
  // 我们还可以将该规划可视化为 RViz 中带有标记的线
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 最后，要执行存储在 my_plan 中的轨迹，您可以使用以下方法调用：请注意，如果机器人同时移动，这可能会导致问题。
  // move_group_interface.execute(my_plan);

  /* 移动到目标姿势 */
  // 如果您不想检查规划的轨迹，以下是上面显示的规划+执行模式的更稳健的组合，应该是首选。请注意，我们之前设置的姿势目标仍然有效，因此机器人将尝试移动到该目标。
  // move_group_interface.move();

  /* 规划到目标关节空间 */
  // 让我们设定一个目标关节空间并朝它移动。这将取代我们上面设置的目标姿势。

  // 首先，我们将创建一个引用当前机器人状态的指针。RobotState 是包含所有当前位置/速度/加速度数据的对象。
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  // 然后，获取关节的值
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // 现在，让我们修改其中一个关节，规划新的目标关节空间，并将该规划可视化。
  joint_group_positions[0] = -tau / 6;  // -1/6 turn in radians
  move_group_interface.setJointValueTarget(joint_group_positions);

  // 我们将允许的最大速度和加速度降低到其最大值的5%。默认值为 10% (0.1)。如果您需要机器人移动得更快，请在机器人的 moveit_config 的 joint_limits.yaml 文件中设置您喜欢的默认值，或者在代码中设置明确的因素。
  move_group_interface.setMaxVelocityScalingFactor(0.05);
  move_group_interface.setMaxAccelerationScalingFactor(0.05);

  success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // 在RViz中可视化该规划
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  /* 带路径约束的规划 */
  // 可以轻松为机器人上的链接指定路径约束。让我们为我们的团队指定路径约束和姿势目标。首先定义路径约束
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // 现在，将其设置为组的路径约束
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group_interface.setPathConstraints(test_constraints);

  /* 在关节空间中执行规划 */
  // 根据规划问题，moveit选择关节空间和笛卡尔空间来表示问题
  // 设置ompl_planning.yaml中的组参数`enforce_joint_model_state_space:true`强制所有规划使用关节空间
  //
  // 默认情况下，带方向路径约束的规划请求会在笛卡尔空间中进行采样以便调用IK服务作为生成采样器
  //
  // 通过强制关节空间，规划过程将使用拒绝抽样来查找有效请求。这可能会大大增加规划时间
  //
  // 我们将重新使用我们原有的旧目标，并重新对其进行规划
  // 请注意，这只有在当前状态已经满足路径约束时才会工作。所以我们需要将起始状态设置为新姿势。
  moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group_interface.setStartState(start_state);

  // 现在我们将从刚刚创建的新起始状态规划到之前的姿势目标
  move_group_interface.setPoseTarget(target_pose1);

  // 带约束的规划可能会很慢，因为每个样本都必须调用逆运动学解算器。让我们将规划时间从默认的5秒增加到10秒，以确保规划器有足够的时间取得成功。
  move_group_interface.setPlanningTime(10.0);

  success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // 在RViz中可视化该规划
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // 完成路径约束后请务必清除它。
  move_group_interface.clearPathConstraints();

  // 笛卡尔路径
  // 您可以通过指定末端执行器要经过的航点列表来直接规划笛卡尔路径。请注意，我们从上面的新起始状态开始。初始姿势（起始状态）不需要添加到航点列表中，但添加它可以帮助实现可视化
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  geometry_msgs::Pose target_pose3 = start_pose2;

  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  // 我们希望以 1 厘米的分辨率对笛卡尔路径进行插值，这就是为什么我们将指定 0.01 作为笛卡尔平移中的最大步长。我们将指定跳跃阈值为 0.0，从而有效地禁用它。警告 - 在操作真实硬件时禁用跳跃阈值可能会导致冗余关节发生大量不可预测的运动，并可能造成安全问题
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // 在RViz中可视化该规划
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 笛卡尔运动通常应该很慢，例如在接近物体时,笛卡尔计划的速度目前无法通过 maxVelocityScalingFactor 设置，而需要您手动计时轨迹
  //
  // 您可以执行一条轨迹，像这样：
  // move_group_interface.execute(trajectory);

  // 向环境中添加对象
  // 首先，让我们规划另一个简单的目标，没有任何障碍物。
  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.x = 1.0;
  another_pose.position.x = 0.7;
  another_pose.position.y = 0.0;
  another_pose.position.z = 0.59;
  move_group_interface.setPoseTarget(another_pose);

  success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (with no obstacles) %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Clear Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // 现在让我们定义一个机器人需要避免的碰撞对象 ROS 信息
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();

  // 对象的 id 用于识别它
  collision_object.id = "box1";

  // 定义一个box添加到世界中。
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.1;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.5;

  // 为盒子定义一个姿势（相对于frame_id指定）
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // 现在，让我们将碰撞对象添加到世界中（使用可以包含其他对象的向量）
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // 在状态的RViz中显示文本并等待MoveGroup接收和处理碰撞对象消息
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // 现在，当我们规划轨迹时，它将避开障碍物
  success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal move around cuboid) %s", success ? "" : "FAILED");
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

  // 将物体附着到机器人上
  // 您可以将物体附着到机器人上，这样它就可以随着机器人的几何形状移动。这模拟了拾取物体以便对其进行操作。运动规划也应避免两个物体之间发生碰撞。
  moveit_msgs::CollisionObject object_to_attach;
  object_to_attach.id = "cylinder1";

  shape_msgs::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = primitive.CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
  cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

  // 我们定义该圆柱体的坐标系/姿势，以便它出现在夹持器中
  object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
  geometry_msgs::Pose grab_pose;
  grab_pose.orientation.w = 1.0;
  grab_pose.position.z = 0.2;

  // 首先，我们将对象添加到世界中（不使用vector）
  object_to_attach.primitives.push_back(cylinder_primitive);
  object_to_attach.primitive_poses.push_back(grab_pose);
  object_to_attach.operation = object_to_attach.ADD;
  planning_scene_interface.applyCollisionObject(object_to_attach);

  // 然后，我们将对象“固定”到给定链接处的机器人上，并允许对象与列出的链接之间发生碰撞。您也可以使用applyAttachedCollisionObject将对象直接附加到机器人上。
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group_interface.attachObject(object_to_attach.id, "panda_hand", { "panda_leftfinger", "panda_rightfinger" });

  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* 等待MoveGroup接收并处理固定的碰撞对象信息 */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

  // 重新规划，但现在有目标放在手上。
  move_group_interface.setStartStateToCurrentState();
  success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

  // 分离和移除对象
  // 现在，让我们将圆柱体从机器人的夹钳上拆下来.
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group_interface.detachObject(object_to_attach.id);

  // 在 RViz 中显示状态文本
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* 等待MoveGroup接收并处理附加的碰撞对象消息 */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

  // 现在，让我们从世界中移除物体。
  ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  object_ids.push_back(object_to_attach.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // 在 RViz 中显示状态文本
  visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to receive and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
