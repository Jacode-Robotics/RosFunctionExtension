# RosFunctionExtension
This repository contains ROS function related codes


Simulation:
1. The model function package stores the model files and simulation functions of the robotic arm, and the controll_physical.launch is the final launch file for the simulation function
2. The configuration related to the model is defined in the my_arm.xacro file, which includes the naming of joints and linkages, the number of robotic arms, etc. The configuration related to the controller is defined in the joints.yaml file. If the parameters such as the model name and number change, the simulation_follow.cpp and joints.yaml files also need to be modified synchronously


Prototype:
1. follow_control is the following control function package of the prototype, and controll_simulation.aunch is the final launch file for starting the robotic arm
2. The model file and the launch file of rviz are stored in the model feature package. Similarly, if there are modifications to the joint or link name or quantity in my_arm.xacro, it is necessary to synchronously modify the content of the joints_state_publish function in the physics_follow.cpp file
3.The dynamicparam.cfg file contains settings for all dynamic parameters. To facilitate debugging, you can open rqt and make changes to it. After obtaining the final result, fill the value into the dynamicparam.cfg file
4. This feature pack supports controlling any number of robotic arms at the same time. If you want multiple pairs of robotic arms to work simultaneously, add a few more node_physics_follow.cpp nodes in controll_simulation.launch, and set the node and args parameters to different values (this value represents the name of the pair of robotic arms). Copy the configuration content in dxl_config.yaml a few more times. It should be noted that the arm_pairx in dxl_config.yaml needs to be consistent with the args in controll_simulation. launch. If multiple robotic arms are connected to the same device, Each bus needs to be set to a different ID. If it needs to be displayed in rviz, the number of robotic arms needs to be added to the my-arm.xacro file in the model function package. The number of robotic arms in my_arm.xacro should be the same as in dxl_config.yaml, which is the number of robotic arms



仿真：
1.model功能包存储了机械臂的模型文件和仿真功能，control_physical.launch为仿真功能的最终launch文件
2.其中模型相关的配置在my_arm.xacro文件中定义，其中包括关节和连杆的命名，机械臂的数量等，控制器相关的配置在joints.yaml文件中定义，如果模型名称和数量等参数发生改变则simulation_follow.cpp和joints.yaml文件也需要同步修改


样机：
1.follow_control为样机的跟随控制功能包，control_simulation.launch为启动机械臂的最终launch文件
2.模型文件和rviz的launch文件存储在model功能包中，同理，如果在my_arm.xacro中对关节或连杆名称或数量有修改，则需要同步修改physics_follow.cpp文件中joints_state_publish函数内的内容
3.dynamic_param.cfg文件中包含所有动态参数的设置，为方便调试可以打开rqt并在里面进行更改调试，得到最终结果后将值填入dynamic_param.cfg文件中
4.本功能包支持同时控制任意数量的机械臂，如果希望多对机械臂同时工作则在control_simulation.launch中多添加几个node_physics_follow.cpp节点，并将其中的node和args参数设置为不同的值（这个值代表了该对机械臂的名称），并将dxl_config.yaml中的配置内容多复制几份即可，需要注意的是dxl_config.yaml中的arm_pairx需要与control_simulation.launch中的args保持一致，如果多条机械臂接在同一个总线则需要设置为不同的ID，如果需要再rviz中显示则需要再model功能包的my_arm.xacro文件中增加机械臂的数量,其中my_arm.xacro与dxl_config.yaml中的“arm_number”应该相同，这是机械臂的编号
