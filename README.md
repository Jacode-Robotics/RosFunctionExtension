# RosFunctionExtension
This repository contains ROS function related codes


Simulation:
1. The model function package stores the model files and simulation functions of the robotic arm, and the control physical. launch is the main switch for the simulation function
2. The configuration related to the model is modified in the my-arm.xacro file, including the naming of joints and linkages, the number of robotic arms, etc. The configuration related to the controller is modified in the joints.yaml file. If the parameters such as the model name and number change, the simulation_follow.cpp file also needs to be modified synchronously


Prototype:
1. The follow_control function package is the following control function package of the prototype, and the controll_simulation.launch is the main switch for starting the robotic arm. It executes all commands in one step and can directly run the prototype run.sh file
2. Modify all configuration parameters in the dxl_config.yaml file. The dynamic parameters related to gripper control are set in the dynamic.param.cfg file. For debugging convenience, you can open rqt and make changes inside it
3. The model file and the launch file of rviz are stored in the model feature pack
4. This feature pack supports controlling any number of robotic arms at the same time. If you want multiple pairs of robotic arms to work simultaneously, you can run a few more node_physics follow.cpp nodes in controll_simulation.launch, and set the node and args parameters to different values. You can also copy the configuration content in dxl_config.yaml a few more times. It should be noted that the arm_pairx in dxl_config.yaml needs to be consistent with the args in controll_simulation.launch. If multiple robotic arms are connected to the same bus, they need to be set to different IDs. If necessary, To display in rviz, it is necessary to increase the number of robotic arms in the my_arm.xacro file of the model feature package, where the "arm_number" in my-arm.xacro should be the same as in dxl_config.yaml
5. It is not recommended to modify all other files



仿真：
1.model功能包存储了机械臂的模型文件和仿真功能，control_physical.launch为仿真功能的总开关
2.其中模型相关的配置在my_arm.xacro文件中修改，其中包括关节和连杆的命名，机械臂的数量等，控制器相关的配置在joints.yaml文件中修改，如果模型名称和数量等参数发生改变则simulation_follow.cpp文件也需要同步修改


样机：
1.follow_control功能包为样机的跟随控制功能包，control_simulation.launch为启动机械臂的总开关，为一步执行所有命令，可以直接运行prototype_run.sh文件
2.在dxl_config.yaml文件中修改所有的配置参数，有关夹爪控制的动态参数在dynamic_param.cfg文件中有设置，为方便调试可以打开rqt并在里面进行更改
3.模型文件和rviz的launch文件存储在model功能包中
4.本功能包支持同时控制任意数量的机械臂，如果希望多对机械臂同时工作则在control_simulation.launch中多运行几个node_physics_follow.cpp节点，并将其中的node和args参数设置为不同的值，并将dxl_config.yaml中的配置内容多复制几份即可，需要注意的是dxl_config.yaml中的arm_pairx需要与control_simulation.launch中的args保持一致，如果多条机械臂接在同一个总线则需要设置为不同的ID，如果需要再rviz中显示则需要再model功能包的my_arm.xacro文件中增加机械臂的数量,其中my_arm.xacro与dxl_config.yaml中的“arm_number”应该相同
5.其他所有文件不建议修改
