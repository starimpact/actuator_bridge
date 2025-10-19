# actuator_bridge

统一的执行器（电机）CAN 桥接包：提供统一命令/反馈消息格式与驱动抽象（当前支持 dm 与 robstride），作为“库”在节点中直接使用，命令写入与反馈读取均为内存访问（不经额外话题中转）。

## 配置
使用私有参数 `~buses` 声明多条总线、厂商类型、收发主题与执行器映射；支持按模型或逐电机设置控制范围。

示例见 `launch/demo.launch` 与 `config/example.yaml`。

## 快速尝试
1) 启动 socketcan 桥与本包节点（需要 socketcan_bridge）：见 `launch/demo.launch`。
2) 在你的节点里直接持有并使用 ActuatorBridge 实例：
	- 发送命令：`bridge.sendCommand(cmd)`（推荐使用 cmd.name 进行路由；未提供 name 时可用 cmd.id 按 id 路由）。
	- 读取反馈：
	  - 回调：`bridge.setFeedbackCallback([](const ActuatorFeedback& fb){ ... });`
	  - 快照：`auto map = bridge.getFeedbackSnapshot();` 或 `bridge.getLastFeedback(name, out)`。

## 扩展新的驱动
实现 `include/actuator_bridge/driver_base.h` 的 DriverBase 接口，并在 `src/bridge.cpp` 的 `getOrCreateBus` 中注册（按 vendor 字符串选择）即可。
