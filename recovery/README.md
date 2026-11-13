# Lite3 独立摔倒恢复工具

提供离线报文预览、机器人状态观察、终端人工确认恢复，以及稳定倒地持续5秒后的自动左恢复。电脑端只需要 Python 3.8+ 和 OpenSSH；机器人端需要 Linux、Python 3.8+、ROS 2 Foxy 或兼容环境及 `rclpy`、`sensor_msgs`、`diagnostic_msgs`、`ament_python`、`colcon`。

该工具使用原厂控制器的恢复流程，不提供低层关节控制。固件指纹不匹配时拒绝请求。原实验中确认过左恢复；本次公开版本的配置化和独立接收器只做离线测试，不代表任意固件、左右方向或倒地姿态均已通过实机验收。不是厂商官方SDK。

## 1. 私有配置

在仓库根目录执行：

```bash
cp recovery/config/recovery.example.json recovery/config/robot.local.json
```

编辑 `robot.local.json`：

| 配置项 | 含义 |
| --- | --- |
| `ssh_host` | 使用者的SSH主机别名或连接目标；示例 `robot` 只是别名 |
| `ssh_key` | 可选的电脑端私钥路径；`null` 使用OpenSSH配置／agent，不把密钥复制进仓库 |
| `ros_setup` | 机器人ROS环境脚本，默认 `/opt/ros/foxy/setup.bash` |
| `robot_workspace` | 机器人只读接收器工作区，默认 `~/lite3_recovery_ws` |
| `factory_root` | 原厂控制程序目录，默认 `~/jy_exe` |
| `factory_ip` | 从原厂 `conf/network.toml` 核实的机器人本机UDP地址 |
| `factory_port` | 原厂控制端口；仍须与 `network.toml` 一致 |
| `telemetry_port` | 从原厂配置核实的状态接收端口，必须与控制端口不同 |
| `ros_domain_id` | 接收器与恢复观察器相同的ROS域，默认83 |
| `dds_profile` | 可选的机器人DDS配置文件；`null` 不依赖任何个人网络配置 |

示例的 `192.0.2.10` 是文档保留地址，必须替换，不能直接用来控制机器人。SSH地址与机器人本机UDP目标是两件事，不可互换。`~` 在对应机器展开：`ssh_key` 属于电脑，其余目录属于机器人。`--host`、`--key` 可以覆盖配置。未知字段和无效配置会报错。

所有 `*.local.json`、日志和审计文件均被忽略。不要将自己的配置、密钥或运行记录加入Git。首次连接先用 `ssh robot` 核对主机指纹，工具始终启用严格主机密钥检查，不保存密码。

## 2. 在机器人安装只读接收器

将本仓库及自己的配置复制到机器人。在机器人仓库根目录执行：

```bash
python3 recovery/scripts/receiver.py build --config recovery/config/robot.local.json
python3 recovery/scripts/receiver.py start --config recovery/config/robot.local.json
```

`start` 在前台运行，保持该终端打开。在机器人另一个终端可检查：

```bash
python3 recovery/scripts/receiver.py status --config recovery/config/robot.local.json
```

接收器只绑定状态端口，发布 `/lite3/imu` 和 `/lite3/native/diagnostics`，不创建速度网关或发送原厂运动指令。必须同时匹配已知固件SHA-256和原厂接口地址。它不共享已占用的UDP端口；若已有兼容接收器运行，应复用它，不启动第二个。恢复工具还要求两种遥测各恰有一个发布者。

接收器不会被安装为开机服务；机器人重启或关闭其终端后，需要重新启动。它不修改原厂程序、模式或网络配置。

## 3. 电脑端只读验证

所有命令均在仓库根目录执行，以下均不发送运动报文：

```bash
# 完全离线，无需配置或SSH
python3 recovery/lite3_recovery.py plan --direction left

# 先验证能收到机器人姿态与原厂诊断
python3 recovery/lite3_recovery.py inspect --config recovery/config/robot.local.json --duration 6

# 检测候选倒地，只报警
python3 recovery/lite3_recovery.py watch --config recovery/config/robot.local.json --duration 30

# 验证5秒自动判定，不发动作；条件满足后结束本次演练
python3 recovery/lite3_recovery.py auto --config recovery/config/robot.local.json --duration 300
```

应看到 `valid_imu_callbacks` 持续增加，`tilt_deg` 有数值且诊断有效。若回调为0、姿态为空，不是在证明没有摔倒，而是没有收到数据。自动模式启动5秒仍无有效IMU或诊断会明确报错退出。

观察器和接收器均运行在机器人本机：直接核验源时间与本机数据年龄；兼容已有接收器附加的电脑时钟偏移，并将其不确定性计入200ms上限。不需要启动相机、导航或电脑时钟进程，也不发布伪造时钟校正。

## 4. 显式启用真实恢复

先启动工具，再进行新的倒地测试。历史倒地事件不会重新授权动作。每次使用全新的本地审计文件，已有文件不会覆盖：

```bash
mkdir -p recovery/reports

# 手动：条件满足后，必须在同一终端输入当次显示的完整RECOVER／RESUME口令
python3 recovery/lite3_recovery.py manual --config recovery/config/robot.local.json \
  --direction left --duration 300 --execute --operator-ready --clearance-confirmed \
  --audit "recovery/reports/manual-$(date +%Y%m%dT%H%M%S%N).jsonl"

# 自动：无需逐次口令；满足条件后即请求一次左恢复
python3 recovery/lite3_recovery.py auto --config recovery/config/robot.local.json \
  --direction left --fall-policy any --duration 300 --execute \
  --audit "recovery/reports/auto-$(date +%Y%m%dT%H%M%S%N).jsonl"
```

默认不带 `--execute` 的入口不创建运动UDP发送器。手动真实恢复要求现场确认参数和真实终端；自动模式要求显式启用和审计文件，不检查相机、超声或周围环境。

`any` 不区分倒地原因，包含已有软急停后倒地。**软急停后若重新满足条件，仍可能自动请求恢复。停止自动流程需输入 `CANCEL` 加回车或按 Ctrl+C。** 工具退出、SSH断开或停止观察，不保证已经被原厂接受的动作停止。

`--fall-policy natural-only` 保留为只观察策略：尚无已核实的自然／人工来源信号，不会根据“没有急停日志”猜测自然摔倒，搭配 `--execute` 会在连接前拒绝。

## 恢复条件与单次请求

- 候选倾角至少65°，窗口内相对参考姿态变化不超过5°。手动候选观察为1秒，自动为完整5秒，不额外叠加1秒。姿态变化、源数据／诊断失效或新软急停会重计时。
- IMU与诊断均须持续更新，数据时效不超过200ms，电量大于20%，固件和原厂进程匹配，不存在冲突控制器或已观察到的原厂错误。
- 必须观察到新的原厂“等待恢复”事件，且30秒内没有后续状态撤销。已观察到正常站立状态码6时不请求恢复；其他状态值不是允许恢复的充分依据。
- 自动入口固定左恢复；手动可显式选择左右。先设置方向，再约100ms后复核并发送一次起立切换请求。起立请求不是幂等命令，UDP没有确认不能成为重发理由。
- 发送前共用 `/tmp/lite3-native-commission.lock`，在机器人 `~/.lite3-recovery-attempts/` 排他写入并持久化本次等待事件。部分发送失败也保留记录，不自动删除或重试。
- 自动恢复观察最多10秒；需要新的开始、左方向、成功事件及正常站立状态与不超过20°的姿态持续2秒。随后正常站立再稳定2秒才允许下一次倒地。失败、超时或结果不明确会停止自动流程。
- 单次会话最多300秒；结束后不发起新请求，已请求的动作最多额外观察10秒。恢复动作本身另需时间，稳定倒地等待5秒不等于5秒内完成站起。

不会发送速度、心跳、模式接管、独立解除急停或退出时的隐藏起立命令。故障和原厂就绪条件不会因 `any` 策略被取消。

## 离线测试与许可证

```bash
python3 -m pip install -r recovery/requirements-dev.txt
python3 -m pytest -q
python3 recovery/scripts/check_export.py
```

测试使用合成日志、模拟ROS／UDP和本地管道，不需要连接机器人。配置、私有记录和构建缓存不属于发布内容。已知固件指纹只作版本校验，不分发原厂库。许可证与提取代码说明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
