# Office Stage 3 Feature Test Summary

## 更新目标

本次更新实现三个新功能，补齐 Office 场景的智能化能力：

1. **动态避障**：基于 `/scan` 实时检测前方障碍物，触发减速停车和 RMF 重规划
2. **任务调度器**：多任务路线分析与最优调度，支持 nearest\_neighbor 和 greedy\_tsp 两种策略
3. **LLM 真实接入**：通过 `config.toml` 配置化接入 OpenAI API，API Key 为空时自动使用 mock 模式

## 新增文件

### 1. 全局配置文件

- `config.toml`

包含三个配置节：

```toml
[llm]                    # LLM 配置（provider, api_key, model, base_url, timeout）
[llm.mock]               # Mock 模式默认值（default_task, default_confidence）
[scheduler]              # 调度器配置（strategy, robot_name, fleet_name）
[obstacle_avoidance]     # 避障配置（threshold, confirm_count, slowdown_factor, cooldown）
```

### 2. 动态避障节点

- `office/scripts/office_dynamic_obstacle_avoidance.py`（278 行）

功能：

- 订阅 `/scan` 话题，检测前方 60 度扇形区域内的动态障碍物
- 障碍物距离低于阈值（默认 1.5m）且连续检测 N 次（默认 3 次）后触发避障
- 避障动作：发送零速 Twist 消息减速/停车 + 发布 ModeRequest 请求 RMF 重规划
- 支持命令行参数覆盖：`--robot-name`、`--threshold`、`--cooldown`
- 从 `config.toml [obstacle_avoidance]` 读取配置

### 3. 任务调度器

- `office/scripts/office_task_scheduler.py`（635 行）

功能：

- 接受多个任务（delivery/patrol/go\_to\_place），基于 Office 拓扑地图分析路线
- 两种调度策略：`nearest_neighbor`（贪心最近邻）和 `greedy_tsp`（2-opt 改进 TSP）
- 按 Euclidean 距离计算最优任务执行顺序，最小化总行程
- 支持 `--dry-run` 模式查看调度计划而不实际派发（无需 ROS 环境）
- 通过 RMF ApiRequest 按优化顺序依次派发任务
- 内置 13 个 Office 语义位置的坐标数据
- 从 `config.toml [scheduler]` 读取配置

任务规格格式：

- `delivery:取货点:送货点`
- `patrol:航点1,航点2,航点3,...`
- `go_to_place:目标位置`

### 4. LLM 命令解析（重构）

- `office/scripts/office_llm_command.py`（336 行，重构）

功能：

- 三种解析模式：`openai`（真实 API）、`mock`（确定性测试）、`keyword_fallback`（本地关键词）
- 配置优先级：环境变量 > config.toml > 默认值
- API Key 为空时自动切换 mock 模式
- 支持 `OPENAI_BASE_URL` 环境变量（兼容 OpenAI 兼容 API）
- 支持 `--force-mock`、`--force-fallback` 强制指定模式
- 支持 `--save-json` 保存解析和执行证据
- 内置轻量 TOML 解析器（无第三方依赖）

### 5. 测试脚本

- `test_stage3_features.py`

覆盖 6 个测试类别、28 个测试用例的自动化测试套件。

## 修改的现有文件

### `office/CMakeLists.txt`

新增安装入口：

- `scripts/office_dynamic_obstacle_avoidance.py`
- `scripts/office_task_scheduler.py`

### `office/package.xml`

- 版本从 `0.2.0` 更新为 `0.3.0`
- 描述更新为包含动态避障和任务调度
- 新增依赖：`rmf_task_msgs`

### `office/scripts/office_llm_command.py`

- 重构：支持 `config.toml` 配置读取
- 新增：mock 解析模式（API Key 为空时自动启用）
- 新增：`--force-mock` 参数
- 新增：`--save-json` 参数
- 新增：环境变量覆盖支持（`OPENAI_BASE_URL`、`OPENAI_PROVIDER`）
- 修复：TOML 解析器正确处理行内注释

## 测试结果

### 测试概况

| 指标   | 值          |
| ---- | ---------- |
| 总测试数 | 28         |
| 通过   | 28         |
| 失败   | 0          |
| 通过率  | **100.0%** |

### 按类别统计

| 类别                | 通过 | 失败 | 总计 |
| ----------------- | -- | -- | -- |
| 静态验证              | 3  | 0  | 3  |
| 配置文件              | 4  | 0  | 4  |
| Feature 3: LLM 集成 | 6  | 0  | 6  |
| Feature 2: 任务调度器  | 7  | 0  | 7  |
| Feature 1: 动态避障   | 6  | 0  | 6  |
| 构建配置              | 2  | 0  | 2  |

### 详细测试结果

#### 静态验证

| 测试                                           | 结果   | 耗时     | 详情              |
| -------------------------------------------- | ---- | ------ | --------------- |
| AST: office\_llm\_command.py                 | PASS | 1.64ms | 336 行, AST 解析通过 |
| AST: office\_dynamic\_obstacle\_avoidance.py | PASS | 0.75ms | 278 行, AST 解析通过 |
| AST: office\_task\_scheduler.py              | PASS | 2.30ms | 635 行, AST 解析通过 |

#### 配置文件

| 测试                      | 结果   | 耗时      | 详情                                                                                |
| ----------------------- | ---- | ------- | --------------------------------------------------------------------------------- |
| config.toml 文件存在        | PASS | 0.01ms  | 路径已验证                                                                             |
| config.toml 解析正确        | PASS | 42.72ms | sections=\[llm, scheduler, obstacle\_avoidance], provider=mock, model=gpt-4o-mini |
| API Key 为空时自动切换 mock 模式 | PASS | 42.02ms | provider=mock when api\_key is empty                                              |
| 环境变量覆盖 config.toml      | PASS | 39.87ms | OPENAI\_API\_KEY/OPENAI\_MODEL 正确覆盖                                               |

#### Feature 3: LLM 集成

| 测试                             | 结果   | 耗时      | 详情                                    |
| ------------------------------ | ---- | ------- | ------------------------------------- |
| LLM mock 模式 - 巡检命令解析为 patrol   | PASS | 41.23ms | task=patrol, parser=mock              |
| LLM mock 模式 - 配送命令解析为 delivery | PASS | 39.69ms | task=delivery, parser=mock            |
| LLM 关键词回退模式 - patrol           | PASS | 38.36ms | task=patrol, parser=keyword\_fallback |
| LLM 默认模式（无 API Key => mock）    | PASS | 40.75ms | parse\_mode\_used=mock                |
| LLM --save-json 保存结果           | PASS | 38.59ms | 文件存在=True, 内容有效=True                  |
| LLM 模糊输入仍返回有效任务                | PASS | 36.45ms | task=patrol（默认回退）                     |

#### Feature 2: 任务调度器

| 测试                                     | 结果   | 耗时      | 详情                                           |
| -------------------------------------- | ---- | ------- | -------------------------------------------- |
| 调度器 dry-run 基本功能（3 个任务）                | PASS | 45.56ms | 任务数=3, 总距离=72.51, 策略=nearest\_neighbor       |
| 调度器 nearest\_neighbor 策略               | PASS | 45.93ms | 策略=nearest\_neighbor, 任务数=3, 总距离=48.28       |
| 调度器 greedy\_tsp 策略                     | PASS | 46.83ms | 策略=greedy\_tsp, 任务数=4, 总距离=141.35            |
| greedy\_tsp 距离 <= nearest\_neighbor 距离 | PASS | 95.69ms | nearest\_neighbor=141.35, greedy\_tsp=141.35 |
| 调度器 --save-json 保存结果                   | PASS | 50.18ms | 文件存在=True, 内容有效=True                         |
| 调度器单任务执行                               | PASS | 45.06ms | 任务数=1                                        |
| 调度器 patrol 多航点任务                       | PASS | 44.35ms | 航点数=7, 类型=patrol                             |

#### Feature 1: 动态避障

| 测试                 | 结果   | 耗时      | 详情                                                            |
| ------------------ | ---- | ------- | ------------------------------------------------------------- |
| 动态避障脚本 AST 解析      | PASS | 0.95ms  | 278 行, AST 解析通过                                               |
| 动态避障配置加载           | PASS | 14.92ms | enabled=true, scan\_topic=/scan, threshold=1.5m               |
| 距离计算函数正确性          | PASS | 43.88ms | euclidean(0,0->3,4)=5.0, charger->supplies=27.21, unknown=inf |
| 前方扇形区域检测逻辑         | PASS | 12.71ms | forward=0.5m: OK, behind=inf: OK, 45deg=1.2m: OK              |
| 动态避障 argparse 参数定义 | PASS | 0.05ms  | --robot-name, --threshold, --cooldown 均存在                     |
| 避障配置参数范围合理         | PASS | 14.25ms | threshold=1.5m, confirm=3, slowdown=0.3, cooldown=10.0s       |

#### 构建配置

| 测试                   | 结果   | 耗时     | 详情                                                         |
| -------------------- | ---- | ------ | ---------------------------------------------------------- |
| CMakeLists.txt 注册新脚本 | PASS | 0.04ms | scheduler=True, obstacle=True, llm=True                    |
| package.xml 版本和依赖    | PASS | 0.01ms | version=0.3.0, rmf\_task\_msgs=True, rmf\_fleet\_msgs=True |

## 测试方法说明

### 静态验证

- 所有新增 Python 文件通过 `ast.parse()` 语法检查
- CMakeLists.txt 和 package.xml 内容检查

### 配置文件测试

- 验证 `config.toml` 存在且可被内置 TOML 解析器正确解析
- 验证三个配置节（llm, scheduler, obstacle\_avoidance）均存在
- 验证 API Key 为空时自动切换 mock 模式
- 验证环境变量优先级高于 config.toml

### LLM 集成测试

- mock 模式：中英文巡检/配送命令正确解析
- 关键词回退模式：英文关键词匹配
- 默认模式：无 API Key 时自动使用 mock
- `--save-json`：结果文件正确写入
- 模糊输入：返回有效默认任务

### 任务调度器测试

- dry-run 基本功能：3 个任务正确调度
- nearest\_neighbor 策略：贪心最近邻排序
- greedy\_tsp 策略：2-opt 改进 TSP 排序
- 路线优化验证：greedy\_tsp 总距离 <= nearest\_neighbor 总距离
- `--save-json`：结果文件正确写入
- 单任务：正常处理
- patrol 多航点：7 个航点正确解析

### 动态避障测试

- AST 语法检查
- 配置加载：从 config.toml 正确读取避障参数
- 距离计算：`euclidean_distance` 和 `location_distance` 函数正确
- 前方扇形检测逻辑：正前方障碍物检测、后方忽略、45 度检测
- argparse 参数定义：`--robot-name`、`--threshold`、`--cooldown` 存在
- 配置参数范围：threshold > 0、slowdown 0-1、confirm >= 1、cooldown > 0

## 运行级验证（需要 ROS 2 + RMF 环境）

### 前置条件

确保以下系统依赖已安装（`fleet_manager` 需要）：

```bash
sudo apt update && sudo apt install -y \
  python3-socketio python3-fastapi python3-uvicorn python3-pydantic
```

验证：

```bash
python3 -c "import socketio, fastapi, uvicorn, pydantic; print('all ok')"
```

### 步骤 1：构建

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  office office_gz office_demos office_maps office_assets \
  office_fleet_adapter office_tasks
source install/setup.bash
```

### 步骤 2：启动仿真 + 合成 Lidar + 建图

> **必须使用** **`office_perception_mapping.launch.xml`**，而非 `office.launch.xml`。
> 原因：TinyRobot 模型没有真实 lidar 传感器，`office_perception_mapping.launch.xml`
> 会额外启动 `office_synthetic_lidar.py`（提供 `/scan` 数据）和 `office_scan_mapper.py`（建图）。
> 没有合成 lidar，避障节点收不到 `/scan`，`scans` 永远为 0。

终端 1：

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 launch office office_perception_mapping.launch.xml headless:=false
```

等待日志出现以下内容后再继续（约 10-15 秒）：

```
[tinyRobot_fleet_adapter] Successfully added new robot: tinyRobot1
[tinyRobot_fleet_adapter] Successfully added new robot: tinyRobot2
[slotcar_tinyRobot1] Setting nominal drive speed to: 4.000000
```

验证 fleet\_manager 已启动：

```bash
ss -tln | grep 22011
# 应看到 127.0.0.1:22011 在 LISTEN
```

### 步骤 3：派发任务

终端 2：

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash

# 方式 A：使用任务调度器（多任务优化调度）
ros2 run office office_task_scheduler.py --use_sim_time \
  "delivery:supplies:hardware" \
  "go_to_place:lounge" \
  "patrol:charger,patrol_a1,patrol_a2,charger"

# 方式 B：使用单任务派发（简单测试）
ros2 run office_tasks dispatch_patrol \
  -p tinyRobot1_charger patrol_A1 \
  -n 1 --use_sim_time
```

> **航点命名说明**：任务调度器接受小写别名（如 `charger`、`patrol_a1`），
> 内部会自动转换为 RMF nav graph 的正式名称（如 `tinyRobot1_charger`、`patrol_A1`）。
> 使用 `dispatch_patrol` 等单任务工具时，必须使用正式名称。

派发成功后应看到：

```
[rmf_task_dispatcher] Determined winning Fleet Adapter: [tinyRobot]
[tinyRobot_command_handle] Received new path for tinyRobot1
```

### 步骤 4：启动动态避障

终端 3：

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 run office office_dynamic_obstacle_avoidance.py
```

正常输出（每 2 秒打印状态）：

```
[STATUS] tinyRobot1 pos=(10.43, -5.58, yaw=1.33) | scans=42 | fwd_min=3.21m | avoiding=False
```

- **`pos`** **在变化** → 机器人正在移动
- **`scans > 0`** → `/scan` 数据正常
- **`fwd_min`** **有实际数值** → 障碍物检测工作正常

### 步骤 5：测试 LLM 命令解析

终端 4：

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash

# mock 模式（默认，无需 API Key）
ros2 run office office_llm_command.py --execute "请巡检办公室所有检查点"

# 真实 API（需要配置 OPENAI_API_KEY）
export OPENAI_API_KEY="sk-xxx"
ros2 run office office_llm_command.py --execute "please inspect the office"
```

### 可用航点速查

| CLI 别名（调度器用） | RMF 正式名（dispatch\_patrol 用） | 说明             |
| ------------ | --------------------------- | -------------- |
| `charger`    | `tinyRobot1_charger`        | tinyRobot1 充电桩 |
| -            | `tinyRobot2_charger`        | tinyRobot2 充电桩 |
| `patrol_a1`  | `patrol_A1`                 | 巡检点 A1         |
| `patrol_a2`  | `patrol_A2`                 | 巡检点 A2         |
| `patrol_b`   | `patrol_B`                  | 巡检点 B          |
| `patrol_c`   | `patrol_C`                  | 巡检点 C          |
| `patrol_d1`  | `patrol_D1`                 | 巡检点 D1         |
| `patrol_d2`  | `patrol_D2`                 | 巡检点 D2         |
| `supplies`   | `supplies`                  | 物资间            |
| `lounge`     | `lounge`                    | 休息室            |
| `pantry`     | `pantry`                    | 茶水间            |
| `hardware`   | `hardware_2`                | 硬件区            |
| `coe`        | `coe`                       | COE            |
| `trash_room` | `trash_room`                | 垃圾房            |

### 常见问题排查

| 现象                                                  | 原因                                             | 解决方法                                                                                 |
| --------------------------------------------------- | ---------------------------------------------- | ------------------------------------------------------------------------------------ |
| `fleet_adapter` 报 `Connection refused` (port 22011) | 缺 `python3-socketio` 等 4 个包，`fleet_manager` 崩溃 | `sudo apt install python3-socketio python3-fastapi python3-uvicorn python3-pydantic` |
| `ros2 run office xxx` 报 `No executable found`       | 脚本缺 `+x` 权限                                    | `chmod +x src/.../office/scripts/office_*.py` 然后 rebuild                             |
| 避障节点 `scans=0`                                      | 没启动合成 lidar                                    | 用 `office_perception_mapping.launch.xml` 代替 `office.launch.xml`                      |
| 避障节点 `incompatible QoS: DURABILITY`                 | `robot_state` QoS 不匹配                          | 已在代码中修复（`TRANSIENT_LOCAL` → `VOLATILE`）                                              |
| 任务 `dispatched` 但机器人不动                              | 航点名小写，RMF 找不到                                  | 调度器已内置别名映射；`dispatch_patrol` 需用正式名                                                   |
| `Unable to deserialize sdf::Model`                  | Gazebo 加载部分模型失败                                | 通常不影响核心功能，可忽略                                                                        |

