## 物料复用循环模式

两个入口 [launch/je_arm_state_machine.launch.py](launch/je_arm_state_machine.launch.py)
和 [launch/je_arm_pcb_inspection_sm.launch.py](launch/je_arm_pcb_inspection_sm.launch.py)
均支持 `loop_mode`（默认 `false`，保持原来的取料、检测、放料流程）。

- 启用：启动时追加 `loop_mode:=true`；恢复普通生产模式：`loop_mode:=false`。
- 此参数在启动时读取，不支持运行中切换。
- 仍需按 `s` 启动，并提供首轮有效 PCB / 空闲放料位消息；调试时可在
      `WAIT_RESOURCES` 用 `w` / `h` 注入配置中的目标，不会自动启动硬件。

循环模式流程：

```text
PICK → INSPECT → PLACE 下探（保持夹紧，不执行 place_open/place_close）
             → PLACE 抬起 → 经 pick_p2 等待位 → 原 PICK 预抓取位
             → 原 PICK 下探位 → place_open 松爪 → 按 pick_offset 抬起
             → place_close 闭空爪 → BACK（末端直线回 pick_p2）→ WAIT_RESOURCES → 下一轮 PICK
```

这里的“初始位置”沿用原 `BACK` 的等待起始位
[config/move_group_client/joint_states/dual_pick_p2.yaml](config/move_group_client/joint_states/dual_pick_p2.yaml)，
不是退回 `IDLE` 的 home 位。新增搬运段也先经过该等待位，再接近取料夹具。
最后放回物料后的 `BACK` 使用笛卡尔直线运动：由 `pick_p2` 关节配置通过正运动学
求出末端目标位姿，再从当前抬起位置直线返回。末端目标不变，但关节终值可能因
逆运动学解而不同。直线规划/执行失败时进入暂停，不回退到关节运动；暂停后恢复
仍使用直线。普通模式和放回物料之前经 `pick_p2` 的搬运段保持原运动方式。

### 位姿、恢复和资源约束

- 首次进入 `PICK` 时保存实际选中的取料位姿（位置、姿态和参考坐标系）。
      后续检测消息不会覆盖循环用的取料位姿；回放和下一轮都使用该位姿。
      循环期间工件/夹具不能移动；需要更换目标时先暂停，确认现场和物料状态后
      用 `b` 放弃旧循环并重新等待检测，或重新启动程序。
- 回放预抓取/抬起复用
      [config/move_group_client/cartesian_offsets/pick_offset.yaml](config/move_group_client/cartesian_offsets/pick_offset.yaml)，
      松爪/闭空爪复用
      [config/gripper_commands.yaml](config/gripper_commands.yaml) 的 `place_open` / `place_close`。
- 只有回放、松爪、抬起和闭空爪均成功后，才标记物料可供下一轮取料。
      暂停、动作失败、延时退出不会续环；`p` 暂停，`r` 从当前阶段恢复。
      循环模式的 `u` 会暂停，不再仅撤销视觉目标；运动/夹爪的 `n` 跳步禁用，
      检测等待时的手动确认仍保留。
- 暂停后按 `b`：放弃本轮并返回等待位，清除锁定的取料位姿和已有 PCB 检测缓存。
      后续收到有效 PCB 检测，且夹爪闭合确认、空闲放料位条件满足后，从新的 `PICK`
      开始并重新锁定位姿。不会直接把旧物料标记为“已放回”，也不会开爪放料；
      按 `b` 前必须确认夹爪无料、物料已在可取位置，避免新检测触发后误取料。
      `r` 仅断点续跑，不清缓存；`f` 故障返回不解除未完成循环的锁定，仍需人工处置。
- 检测仍需原来的检测完成信号/服务应答，空闲放料位仍需有效。
      此功能只省去重复加料，不会伪造检测成功或绕过放料位资源检查。
      `skip_inspect_handover` 行为不变；完整交接流程最终仍须把物料交回左臂。

### 上机前检查

先在仿真中验证多轮及各阶段暂停恢复，再在低速、有人监护条件下确认
携料经过放料点和返回路径不会碰撞、工件确实落入原夹具、松爪宽度足够。
既有规划场景不等于真实工件/治具碰撞验证；夹爪动作成功也不等于检测到了
实际落料。启用循环模式本身不代表系统已具备无人值守的安全条件。

Launch 参数的无硬件检查位于
[test/test_loop_mode_launch.py](test/test_loop_mode_launch.py)，只构造描述、不启动节点。

 <h2>Build Instructions</h2>

First, source your ros2 installation.
```
source /opt/ros/humble/setup.bash
```

Before you build, make sure you've installed all the dependencies...

```
rosdep install --ignore-src --from-paths src -y -r
```

Then build with colcon build...

```
colcon build
```
  <h2>Operating Instructions</h2>

Then source the proper workspace...

```
source ~/workspace/humble_ws/install/setup.sh
```

And then run the launch file...

```
ros2 launch sm_panda_cl_moveit2z_cb_inventory sm_panda_cl_moveit2z_cb_inventory.launch.py
```


 <h2>Viewer Instructions</h2>
If you have the SMACC2 Runtime Analyzer installed then type...

```
ros2 run smacc2_rta smacc2_rta
```

If you don't have the SMACC2 Runtime Analyzer click <a href="https://robosoft.ai/product-category/smacc2-runtime-analyzer/">here</a>

处理外部事件队列 → 执行状态转移 → 对当前激活状态调用 update()（如果该状态实现了）→ 再睡眠到下一周期

je_arm_pcb_inspection_sm_node.cpp
      └─ smacc2::run<SmJeArmPcbInspection>()
            │
            └─ SignalDetector 线程  ← 这就是"主循环"
                    │
                    以 20Hz (config.yaml 里 signal_detector_loop_freq) 轮询：
                    │  ① rclcpp::spin_some()      ← 处理 ROS 话题/服务
                    │  ② 检查 pending events 队列  ← CB success/failure/键盘等
                    │  ③ 触发 Boost.Statechart 状态转移
                    │  ④ 调 update() (如果实现了)

SM 进入 StLPregrasp
    │
    ├─ staticConfigure()         [编译期/初始化期，声明 CB]
    ├─ StLPregrasp::onEntry()    [同步: 写黑板、打日志]
    └─ CbMoveKnownState::onEntry() [异步线程: 读yaml → plan → execute]
                │
                │  ...轨迹执行中...
                │
                └─ 完成 → postEvent<EvCbSuccess>
                                │
                          SignalDetector 主循环消费
                                │
                          Transition → StGripperOpen