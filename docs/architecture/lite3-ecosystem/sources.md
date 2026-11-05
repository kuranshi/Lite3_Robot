# Lite3 跨仓库架构图：源码索引

本目录 evidence 是本次读取的源文件汇总，不是完整可运行检出。图中的本地 snapshot 提交固定了这些文本；以下链接回到五个原始仓库的固定提交。

## 仓库版本

- [kuranshi/deep_robotics_model @ 52352f5](https://github.com/kuranshi/deep_robotics_model/tree/52352f5bbf2a30b5d02ece91900415188e7abc74)
- [kuranshi/rl_training @ 9c0e710](https://github.com/kuranshi/rl_training/tree/9c0e710421b165a444b2c73e5a9d65ee3fafa299)
- [kuranshi/sdk_deploy @ 2027907](https://github.com/kuranshi/sdk_deploy/tree/20279073bc43522aa06b0abf611bb2daaa53d87d)
- [kuranshi/Lite3_rl_deploy @ 1c4a8f1](https://github.com/kuranshi/Lite3_rl_deploy/tree/1c4a8f14c9c531903fa5a26783e3f9b99d1039d9)
- [kuranshi/Lite3_MotionSDK @ b30a3ec](https://github.com/kuranshi/Lite3_MotionSDK/tree/b30a3ec09619e1dd0f3cfa6c49a50eb59670f141)

## 节点证据

### deep_robotics_model

- [deep_robotics_model/README.md:1–7](https://github.com/kuranshi/deep_robotics_model/blob/52352f5bbf2a30b5d02ece91900415188e7abc74/README.md#L1-L7)
- [rl_training/.gitmodules:1–4](https://github.com/kuranshi/rl_training/blob/9c0e710421b165a444b2c73e5a9d65ee3fafa299/.gitmodules#L1-L4)

### rl_training

- [rl_training/source/rl_training/rl_training/assets/deeprobotics.py:14–38](https://github.com/kuranshi/rl_training/blob/9c0e710421b165a444b2c73e5a9d65ee3fafa299/source/rl_training/rl_training/assets/deeprobotics.py#L14-L38)
- [rl_training/scripts/reinforcement_learning/rsl_rl/train.py:152–202](https://github.com/kuranshi/rl_training/blob/9c0e710421b165a444b2c73e5a9d65ee3fafa299/scripts/reinforcement_learning/rsl_rl/train.py#L152-L202)
- [rl_training/scripts/tools/export_onnx_fast.py:190–237](https://github.com/kuranshi/rl_training/blob/9c0e710421b165a444b2c73e5a9d65ee3fafa299/scripts/tools/export_onnx_fast.py#L190-L237)

### ONNX 策略文件

- [rl_training/scripts/tools/export_onnx_fast.py:211–237](https://github.com/kuranshi/rl_training/blob/9c0e710421b165a444b2c73e5a9d65ee3fafa299/scripts/tools/export_onnx_fast.py#L211-L237)
- [sdk_deploy/src/Lite3_sdk_deploy/run_policy/lite3_policy_runner.hpp:32–79](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/Lite3_sdk_deploy/run_policy/lite3_policy_runner.hpp#L32-L79)
- [Lite3_rl_deploy/run_policy/lite3_test_policy_runner_onnx.hpp:40–93](https://github.com/kuranshi/Lite3_rl_deploy/blob/1c4a8f14c9c531903fa5a26783e3f9b99d1039d9/run_policy/lite3_test_policy_runner_onnx.hpp#L40-L93)

### Lite3_rl_deploy

- [Lite3_rl_deploy/state_machine/state_machine.hpp:121–156](https://github.com/kuranshi/Lite3_rl_deploy/blob/1c4a8f14c9c531903fa5a26783e3f9b99d1039d9/state_machine/state_machine.hpp#L121-L156)
- [Lite3_rl_deploy/run_policy/lite3_test_policy_runner_onnx.hpp:40–104](https://github.com/kuranshi/Lite3_rl_deploy/blob/1c4a8f14c9c531903fa5a26783e3f9b99d1039d9/run_policy/lite3_test_policy_runner_onnx.hpp#L40-L104)
- [Lite3_rl_deploy/interface/robot/hardware/hardware_interface.hpp:21–42](https://github.com/kuranshi/Lite3_rl_deploy/blob/1c4a8f14c9c531903fa5a26783e3f9b99d1039d9/interface/robot/hardware/hardware_interface.hpp#L21-L42)

### Lite3 · ROS2 接入

- [Lite3_MotionSDK/README.md:45–55](https://github.com/kuranshi/Lite3_MotionSDK/blob/b30a3ec09619e1dd0f3cfa6c49a50eb59670f141/README.md#L45-L55)
- [sdk_deploy/src/lite3_transfer/interface/robot/udp_interface.hpp:139–148](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/lite3_transfer/interface/robot/udp_interface.hpp#L139-L148)

### lite3_transfer

- [sdk_deploy/src/lite3_transfer/interface/robot/udp_interface.hpp:139–210](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/lite3_transfer/interface/robot/udp_interface.hpp#L139-L210)
- [sdk_deploy/src/lite3_transfer/interface/robot/udp_interface.hpp:214–242](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/lite3_transfer/interface/robot/udp_interface.hpp#L214-L242)
- [sdk_deploy/src/lite3_transfer/interface/robot/udp_interface.hpp:288–357](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/lite3_transfer/interface/robot/udp_interface.hpp#L288-L357)

### Lite3_sdk_deploy

- [sdk_deploy/src/Lite3_sdk_deploy/state_machine/quadruped/rl_control_state.hpp:41–105](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/Lite3_sdk_deploy/state_machine/quadruped/rl_control_state.hpp#L41-L105)
- [sdk_deploy/src/Lite3_sdk_deploy/interface/robot/hardware/lite3_interface.hpp:65–84](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/Lite3_sdk_deploy/interface/robot/hardware/lite3_interface.hpp#L65-L84)
- [sdk_deploy/src/Lite3_sdk_deploy/run_policy/lite3_policy_runner.hpp:32–114](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/Lite3_sdk_deploy/run_policy/lite3_policy_runner.hpp#L32-L114)

### Lite3_MotionSDK

- [Lite3_MotionSDK/include/sender.h:19–57](https://github.com/kuranshi/Lite3_MotionSDK/blob/b30a3ec09619e1dd0f3cfa6c49a50eb59670f141/include/sender.h#L19-L57)
- [Lite3_rl_deploy/CMakeLists.txt:80–85](https://github.com/kuranshi/Lite3_rl_deploy/blob/1c4a8f14c9c531903fa5a26783e3f9b99d1039d9/CMakeLists.txt#L80-L85)
- [Lite3_rl_deploy/CMakeLists.txt:184–196](https://github.com/kuranshi/Lite3_rl_deploy/blob/1c4a8f14c9c531903fa5a26783e3f9b99d1039d9/CMakeLists.txt#L184-L196)

### lite3_sdk_service

- [sdk_deploy/src/lite3_sdk_service/main.cpp:26–49](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/lite3_sdk_service/main.cpp#L26-L49)
- [sdk_deploy/src/lite3_sdk_service/service/lite3_sdk_service.hpp:186–224](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/lite3_sdk_service/service/lite3_sdk_service.hpp#L186-L224)
- [sdk_deploy/src/lite3_sdk_service/service/lite3_sdk_service.hpp:520–555](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/lite3_sdk_service/service/lite3_sdk_service.hpp#L520-L555)

### MuJoCo 仿真后端

- [sdk_deploy/src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py:34–34](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py#L34-L34)
- [sdk_deploy/src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py:53–99](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py#L53-L99)
- [sdk_deploy/src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py:207–276](https://github.com/kuranshi/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py#L207-L276)

### Lite3 · UDP 直连

- [Lite3_rl_deploy/interface/robot/hardware/hardware_interface.hpp:21–42](https://github.com/kuranshi/Lite3_rl_deploy/blob/1c4a8f14c9c531903fa5a26783e3f9b99d1039d9/interface/robot/hardware/hardware_interface.hpp#L21-L42)
- [Lite3_rl_deploy/interface/robot/hardware/hardware_interface.hpp:49–102](https://github.com/kuranshi/Lite3_rl_deploy/blob/1c4a8f14c9c531903fa5a26783e3f9b99d1039d9/interface/robot/hardware/hardware_interface.hpp#L49-L102)
- [Lite3_MotionSDK/README.md:45–55](https://github.com/kuranshi/Lite3_MotionSDK/blob/b30a3ec09619e1dd0f3cfa6c49a50eb59670f141/README.md#L45-L55)

## 额外核对

- 默认姿态差异：训练资产 `assets/deeprobotics.py:31–38` 与快速导出 `export_onnx_fast.py:54–55`；ROS2 推理器 `lite3_policy_runner.hpp:93–100`。
- 本图的两种接入方式为替代方案；没有将五个 GitHub 仓库描绘为同时运行的五个服务。
- 图中的物理控制器为外部系统；其内部驱动实现未包含在所读开源源码中。

