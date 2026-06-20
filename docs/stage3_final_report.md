# 《机器人技术》课程项目 — 阶段三文档

## 仿真实现与期末答辩

***

## 0. 基本信息

- 课程名称：《机器人技术》
- 学期：2025-2026 学年第 2 学期
- 项目类型：选项 A（工程应用项目）
- 项目题目：Office 场景下移动机器人仿真系统 — 仿真实现与评估
- 仓库链接：<https://github.com/Skw03/Robotics_export>
- 小组成员：2356218 孙凯文、2351875 李璐巍、2354275 邱婉盈

***

## 1. 项目概述

本项目基于 ROS 2 Humble + Gazebo Classic 11 仿真平台，设计并实现了一套面向 Office 室内服务场景的移动机器人系统。系统集成了 RMF（Robot Middleware Framework）车队管理、Nav2 导航栈、LLM 自然语言命令解析、动态避障监控和多任务优化调度等核心功能，在仿真环境中完整实现了从自然语言指令输入到机器人自主执行任务的端到端闭环。

### 1.1 系统核心能力

| 能力      | 实现方式                                       | 状态     |
| ------- | ------------------------------------------ | ------ |
| 环境感知与建图 | 合成 LaserScan + OccupancyGrid + AMCL        | 已实现并验证 |
| 实时定位    | AMCL（2000 粒子，likelihood\_field 模型）         | 已实现并验证 |
| 路径规划    | NavfnPlanner(A\*) + SmacPlanner2D 双规划器     | 已实现并对比 |
| 避障策略    | collision\_monitor + baseline\_costmap 双配置 | 已实现并对比 |
| 动态避障    | /scan 监控 + slotcar 自动停车 + RMF 重规划          | 已实现并验证 |
| 任务调度    | nearest\_neighbor + greedy\_tsp(2-opt) 双策略 | 已实现并对比 |
| 自然语言交互  | 智谱 GLM-4.5-air + mock + keyword\_fallback  | 已实现并验证 |
| LLM 集成  | OpenAI 兼容 API（结构化 JSON 输出）                 | 已实现并验证 |
| 多场景任务   | delivery（配送）+ patrol（巡检）                   | 已实现并评估 |

### 1.2 系统架构

```
┌──────────────────────────────────────────────────────────────┐
│                      交互层 (Interaction)                     │
│  自然语言输入 → office_llm_command.py (GLM-4.5-air / mock)  │
└────────────────────────┬─────────────────────────────────────┘
                         │ 解析结果 (delivery / patrol)
                         ▼
┌──────────────────────────────────────────────────────────────┐
│                      任务层 (Task)                            │
│  office_task_scheduler.py (nearest_neighbor / greedy_tsp)    │
│  → dispatch_delivery / dispatch_patrol (RMF ApiRequest)      │
└────────────────────────┬─────────────────────────────────────┘
                         │ 任务请求
                         ▼
┌──────────────────────────────────────────────────────────────┐
│                   车队管理层 (Fleet)                           │
│  RMF Dispatcher → Fleet Adapter → RobotCommandHandle         │
│  fleet_manager (FastAPI:22011) → RobotClientAPI              │
└────────────────────────┬─────────────────────────────────────┘
                         │ 路径跟踪指令
                         ▼
┌──────────────────────────────────────────────────────────────┐
│                   导航层 (Navigation)                         │
│  Nav2: AMCL + Navfn/Smac + RPP Controller                   │
│  collision_monitor: PolygonStop(0.45m) + PolygonSlow(0.70m) │
│  velocity_smoother: max 0.80 m/s                             │
└────────────────────────┬─────────────────────────────────────┘
                         │ 速度指令
                         ▼
┌──────────────────────────────────────────────────────────────┐
│                   感知定位层 (Perception)                     │
│  /scan (gpu_lidar / synthetic_lidar) → OccupancyGrid         │
│  AMCL 定位 → TF: map→odom→base_footprint→lidar_link         │
└────────────────────────┬─────────────────────────────────────┘
                         │ 传感器数据
                         ▼
┌──────────────────────────────────────────────────────────────┐
│                   仿真层 (Simulation)                         │
│  Gazebo Classic 11 + Office World + TinyRobot (差速驱动)     │
│  slotcar 插件 (stop_distance=1.0m) + diff_drive 控制器       │
└──────────────────────────────────────────────────────────────┘
```

***

## 2. 感知与环境建模

### 2.1 传感器配置

机器人搭载以下传感器（URDF 定义于 `robotics_description/urdf/robotics.urdf.xacro`）：

| 传感器     | 类型                  | 规格                      | 话题                  |
| ------- | ------------------- | ----------------------- | ------------------- |
| 2D 激光雷达 | gpu\_lidar (Gazebo) | 360 线，10Hz，范围 0.3-10.5m | `/scan`             |
| 3D 激光雷达 | Ouster OS-0 (32 线)  | 32 线垂直，范围 50m           | `/os_cloud`         |
| RGB 相机  | 普通相机                | 640×480，30Hz            | `/camera/image_raw` |
| IMU     | 6 轴惯性测量             | 100Hz                   | `/imu/data`         |
| GPS     | 卫星定位                | 10Hz                    | `/gps/fix`          |

### 2.2 环境感知实现

系统提供两条感知链路：

**链路 A：合成激光 + 栅格建图（office\_perception\_mapping.launch.xml）**

```
robot_state (RMF) → office_synthetic_lidar.py → /scan (LaserScan)
                                                    ↓
                                          office_scan_mapper.py → /map (OccupancyGrid)
```

- `office_synthetic_lidar.py`：订阅 RMF `robot_state` 获取机器人位姿，基于 Office 建筑模型生成仿真 LaserScan 数据，同时发布完整 TF 链（map→odom→base\_footprint→lidar\_link）
- `office_scan_mapper.py`：订阅 `/scan` 和 `robot_state`，将扫描点转换到地图坐标系，发布 `/map`（OccupancyGrid）

**链路 B：Gazebo 原生传感器（office.launch.xml）**

```
Gazebo gpu_lidar → /scan (LaserScan)
Gazebo diff_drive → /odom → TF: odom→base_footprint
AMCL → TF: map→odom
```

- TinyRobot SDF 模型中集成了 `gpu_lidar` 传感器，直接在 Gazebo 中进行光线投射
- slotcar 插件提供里程计和运动控制

### 2.3 地图构建与维护

- **静态地图**：Office 场景预构建栅格地图（`robotics_nav2/map/office_map.yaml`），分辨率 0.05m
- **拓扑地图**：定义 16 个语义位置及其坐标（`robotics_nav2/map/office_topology.yaml`），用于语义导航和任务调度
- **RMF 导航图**：`office_maps/generated_maps/office/nav_graphs/0.yaml` 定义航点和车道拓扑，RMF 基于此进行路径规划

### 2.4 实时定位

- **AMCL**：2000 粒子，likelihood\_field 激光模型，初始位姿 (55.07, -58.48)
- **EKF**：`robotics_localization` 包提供扩展卡尔曼滤波定位（`dual_ekf_navsat_params.yaml`）
- **HDL 定位**：集成 `hdl_localization` 子模块，支持 3D 激光全局定位（BBS + FPFH-RANSAC）

### 2.5 验证结果

| 验证项          | 方法                                                    | 结果                                      |
| ------------ | ----------------------------------------------------- | --------------------------------------- |
| `/scan` 话题发布 | `ros2 topic hz /scan`                                 | 10Hz 稳定发布                               |
| `/map` 话题发布  | `ros2 topic echo /map --once`                         | OccupancyGrid 正确生成                      |
| TF 链完整性      | `ros2 run tf2_ros tf2_echo base_footprint lidar_link` | map→odom→base\_footprint→lidar\_link 完整 |
| AMCL 生命周期    | Nav2 launch 日志                                        | lifecycle 节点正常进入 active 状态              |

***

## 3. 运动控制与避障

### 3.1 基本运动控制

机器人采用差速驱动模型，通过 Gazebo `libgazebo_ros_diff_drive` 插件实现：

| 参数    | 值                     |
| ----- | --------------------- |
| 底盘尺寸  | 0.82m × 0.62m × 0.18m |
| 轮子半径  | 0.20m                 |
| 轮距    | 0.62m                 |
| 质量    | 15kg                  |
| 最大速度  | 0.80 m/s              |
| 最大加速度 | 2.0 m/s²              |

Nav2 控制器使用 `RegulatedPurePursuitController`（RPP），参数：

- 前瞻距离：0.35-1.1m（动态调节）
- 碰撞检测：开启
- 目标容差：xy 0.12m，yaw 0.25rad

### 3.2 避障策略对比

系统实现了两种避障配置，通过 `generate_nav2_profile.py` 生成参数文件：

#### 策略 1：baseline\_costmap（基于代价地图）

- 全局代价地图：static\_layer + obstacle\_layer + inflation\_layer
- 局部代价地图：obstacle\_layer + inflation\_layer（5m × 5m 滚动窗口）
- 障碍物衰减：1.0s
- 膨胀半径：0.55m

#### 策略 2：collision\_monitor（碰撞监控器）

- `PolygonStop`：0.45m 范围内检测到障碍物时完全停车
- `PolygonSlow`：0.70m 范围内检测到障碍物时减速至 0.55 倍速
- `FootprintApproach`：1.2s 碰撞预测，提前减速

#### 对比分析

| 指标           | baseline\_costmap       | collision\_monitor      |
| ------------ | ----------------------- | ----------------------- |
| 机制           | 代价地图膨胀 + 路径绕行           | 实时几何检测 + 速度调节           |
| 响应速度         | 较慢（需代价地图更新）             | 快速（直接检测）                |
| 路径质量         | 绕行路径更平滑                 | 可能突然停车/减速               |
| 适用场景         | 静态/半静态环境                | 动态障碍物环境                 |
| delivery 成功率 | 100%（navfn）/ 100%（smac） | 60%（navfn）/ 100%（smac）  |
| patrol 成功率   | 100%（navfn）/ 100%（smac） | 100%（navfn）/ 100%（smac） |

**关键发现**：`navfn_astar + collision_monitor` 在 delivery 任务上成功率仅 60%，出现 FAILED 和 TIMEOUT 各 1 次。原因是 collision\_monitor 的激进停车策略与 NavFn 的路径规划产生冲突——NavFn 规划的路径穿过狭窄通道时，collision\_monitor 频繁触发停车，导致任务超时。`smac_2d + collision_monitor` 不受此影响，因为 SmacPlanner 的代价感知规划能主动避开高代价区域。

### 3.3 动态避障系统

`office_dynamic_obstacle_avoidance.py`（338 行）实现了运行时动态障碍物监控：

**检测算法**：

1. 订阅 `/scan`（LaserScan），在机器人前方 ±60° 扇形区域内搜索最近障碍物
2. 过滤无效读数（inf、nan、< range\_min）
3. 使用确认计数器（默认 3 次）防止误触发
4. 障碍物距离低于阈值（1.5m）时触发避障预警

**避障流程**（监控模式，不直接控制机器人）：

```
/scan (gpu_lidar) → DynamicObstacleAvoidance (监控节点)
                          │
                    [早期预警日志]
                          │
slotcar 插件 (Gazebo):    │
  stop_distance=1.0m → 自动停车 → MODE_WAITING
                          │
fleet_manager: replan=True │
                          │
RobotCommandHandle:        │
  update_handle.replan()   │
                          │
RMF dispatcher:           │
  寻找替代路线             │
                          │
follow_new_path() → 机器人重新路由
```

**卡住检测**：若机器人处于 MODE\_WAITING 超过 30 秒，发出警告提示障碍物可能是永久性的且导航图中无替代路径。

### 3.4 避障实验结果

| 实验        | 完成时间   | 行驶距离   | 成功 | 重规划次数 |
| --------- | ------ | ------ | -- | ----- |
| 基线导航（无障碍） | 46.1s  | 1.44m  | 是  | 0     |
| 障碍物避障     | 62.5s  | 1.81m  | 是  | 0     |
| 差值        | +16.4s | +0.37m | -  | -     |

避障系统成功使机器人绕过障碍物完成任务，代价是增加 35.6% 的完成时间和 25.7% 的行驶距离。

***

## 4. 路径规划

### 4.1 规划算法实现

系统实现了两种全局规划算法，通过 Nav2 配置切换：

#### NavfnPlanner (A\*)

- 基于 Dijkstra/A\* 的经典栅格规划器
- 在规则栅格地图上搜索最短路径
- 适合结构化室内环境，路径可解释性强
- 配置：`use_astar: true`，`tolerance: 0.5m`，`use_final_approach: false`

#### SmacPlanner2D

- 基于状态格搜索的规划器
- 考虑代价约束和运动学可行性
- 在狭窄通道和绕行场景下鲁棒性更强
- 配置：`tolerance: 0.5m`，`downsample_costmap: false`

### 4.2 规划器对比实验

实验矩阵：2 规划器 × 2 避障配置 × 2 任务类型，每组 5 次重复试验。

| 规划器          | 避障配置               | 任务       | 成功率        | 平均耗时(s) | 超时率    | 失败模式                |
| ------------ | ------------------ | -------- | ---------- | ------- | ------ | ------------------- |
| navfn\_astar | baseline\_costmap  | delivery | 100.00%    | 7.656   | 0.00%  | -                   |
| navfn\_astar | baseline\_costmap  | patrol   | 100.00%    | 0.733   | 0.00%  | -                   |
| navfn\_astar | collision\_monitor | delivery | **60.00%** | 51.708  | 20.00% | FAILED=1; TIMEOUT=1 |
| navfn\_astar | collision\_monitor | patrol   | 100.00%    | 1.864   | 0.00%  | -                   |
| smac\_2d     | baseline\_costmap  | delivery | 100.00%    | 10.183  | 0.00%  | -                   |
| smac\_2d     | baseline\_costmap  | patrol   | 100.00%    | 0.095   | 0.00%  | -                   |
| smac\_2d     | collision\_monitor | delivery | 100.00%    | 8.252   | 0.00%  | -                   |
| smac\_2d     | collision\_monitor | patrol   | 100.00%    | 0.094   | 0.00%  | -                   |

### 4.3 分析结论

1. **综合最优组合**：`smac_2d + collision_monitor`，delivery/patrol 均达 100% 成功率，且两个任务平均耗时均处于最优区间
2. **NavFn 的脆弱性**：`navfn_astar + collision_monitor / delivery` 成功率仅 60%，平均耗时高达 51.7s。NavFn 规划的路径不考虑代价约束，在狭窄通道中与 collision\_monitor 的激进停车策略冲突
3. **SmacPlanner 的鲁棒性**：SmacPlanner 在所有配置下均达 100% 成功率，其代价感知规划能主动避开高代价区域
4. **patrol 任务稳定性**：所有 4 种组合在 patrol 任务上均达 100% 成功率，因为 patrol 任务路径较短且避开了狭窄通道

### 4.4 Nav2 配置选择依据

| 配置项   | 选择                             | 依据            |
| ----- | ------------------------------ | ------------- |
| 全局规划器 | SmacPlanner2D（推荐）              | 代价感知，鲁棒性更强    |
| 局部控制器 | RegulatedPurePursuitController | 适合差速驱动，前瞻距离可调 |
| 碰撞监控  | collision\_monitor（推荐）         | 实时响应，适合动态环境   |
| 速度平滑  | velocity\_smoother             | 避免急加速，提升乘坐舒适度 |
| 恢复行为  | spin + backup + wait           | 多策略应对卡死情况     |

***

## 5. 应用场景实现与评估

### 5.1 场景一：配送任务（Delivery）

**任务逻辑**：机器人从取货点取货，沿规划路径行驶至送货点完成配送。

**实现方式**：

- RMF 模式：`dispatch_delivery.py` 通过 `ApiRequest` 派发 delivery 类别任务
- Nav2 模式：`course_task_dispatcher.py` 通过 Delivery Action 派发
- 行为树：`office_delivery.xml`（NavigateToPose 序列）

**Office 语义位置**：

| 别名          | RMF 航点名             | 坐标(x, y)       | 功能       |
| ----------- | ------------------- | -------------- | -------- |
| charger     | tinyRobot1\_charger | (10.43, -5.58) | 充电桩      |
| supplies    | supplies            | (8.75, -2.27)  | 物资间（取货点） |
| hardware    | hardware\_2         | (5.50, -2.50)  | 硬件区（送货点） |
| lounge      | lounge              | (16.50, -5.50) | 休息室      |
| pantry      | pantry              | -              | 茶水间      |
| coe         | coe                 | -              | COE      |
| trash\_room | trash\_room         | -              | 垃圾房      |

**成功/失败判定**：

| 判定   | 标准                                         |
| ---- | ------------------------------------------ |
| 成功   | `accepted=true` 且 `status` 为成功状态，机器人到达目标位置 |
| 超时失败 | `status=TIMEOUT`，任务超过 120s 未完成             |
| 运行异常 | `status=ERROR` 或 `status=FAILED`           |
| 调度失败 | `accepted=false`，任务被 RMF 拒绝                |

**评估指标**：

- 成功率：100%（smac\_2d + collision\_monitor）
- 平均完成时间：8.252s
- 超时率：0%

### 5.2 场景二：巡检任务（Patrol）

**任务逻辑**：机器人按指定航点序列依次巡逻，覆盖多个检查点。

**实现方式**：

- RMF 模式：`dispatch_patrol.py` 通过 compose + go\_to\_place 序列派发
- 支持多航点序列：`patrol:charger,patrol_a1,patrol_a2,charger`

**巡检航点**：

| 别名         | RMF 航点名    | 坐标(x, y)       |
| ---------- | ---------- | -------------- |
| patrol\_a1 | patrol\_A1 | (12.54, -6.98) |
| patrol\_a2 | patrol\_A2 | (15.16, -6.91) |
| patrol\_b  | patrol\_B  | -              |
| patrol\_c  | patrol\_C  | -              |
| patrol\_d1 | patrol\_D1 | -              |
| patrol\_d2 | patrol\_D2 | -              |

**成功/失败判定**：同配送任务标准。

**评估指标**：

- 成功率：100%（所有 4 种配置）
- 平均完成时间：0.094s（smac\_2d + collision\_monitor，最优）
- 超时率：0%

### 5.3 场景对比分析

| 指标                    | Delivery                      | Patrol   |
| --------------------- | ----------------------------- | -------- |
| 路径长度                  | 较长（跨区域）                       | 较短（局部区域） |
| 窄通道风险                 | 高（需穿越走廊）                      | 低        |
| collision\_monitor 影响 | 显著（navfn 降至 60%）              | 无影响      |
| 最优配置                  | smac\_2d + collision\_monitor | 任意配置均可   |

### 5.4 扩展场景（探索性）

系统还支持以下扩展任务类型（已实现但未纳入正式评估）：

- **清洁任务**（`dispatch_clean.py`）：机器人前往指定区域执行清洁，完成后返回充电桩
- **循环任务**（`dispatch_loop.py`）：机器人在指定航点间循环运行
- **远程操控**（`dispatch_teleop.py`）：人工远程控制机器人
- **推车配送**（`dispatch_cart_delivery.py`）：带推车的配送任务
- **动态事件**（`dispatch_dynamic_event.py`）：响应动态事件的任务派发

***

## 6. 人机交互

### 6.1 自然语言命令接口

系统提供 `office_llm_command.py` 作为主要交互接口，支持中英文自然语言输入：

```bash
# 巡检命令
ros2 run office office_llm_command.py --execute "请巡检办公室所有检查点"
ros2 run office office_llm_command.py --execute "patrol the office corridors"

# 配送命令
ros2 run office office_llm_command.py --execute "把文件送到硬件区"
ros2 run office office_llm_command.py --execute "deliver files to hardware room"
```

### 6.2 解析模式

| 模式                | 机制                                                   | 延迟         | 适用条件             |
| ----------------- | ---------------------------------------------------- | ---------- | ---------------- |
| openai            | OpenAI Responses API + json\_schema 结构化输出            | 500-2000ms | 有 OpenAI API Key |
| openai\_compat    | OpenAI 兼容 Chat Completions API（智谱/DeepSeek/Moonshot） | 300-1500ms | 有兼容 API Key      |
| mock              | 关键词匹配 + 模拟置信度                                        | <1ms       | 无 API Key 时自动启用  |
| keyword\_fallback | 纯关键词匹配                                               | <1ms       | 始终可用作兜底          |

### 6.3 Prompt 设计

**系统提示词**：

> "Parse an Office robot service command. Only choose delivery or patrol. Reject warehouse or unsupported robot tasks by mapping to the closest Office task with low confidence."

**输出 JSON Schema**：

```json
{
  "task": {"enum": ["delivery", "patrol"]},
  "confidence": {"type": "number", "minimum": 0, "maximum": 1},
  "reason": {"type": "string"}
}
```

### 6.4 命令解析示例

| 输入                               | 解析结果       | 置信度  | 解析器               |
| -------------------------------- | ---------- | ---- | ----------------- |
| "请巡检办公室所有检查点"                    | patrol     | 0.86 | mock              |
| "把文件送到硬件办公室"                     | delivery   | 0.90 | mock              |
| "patrol the office corridors"    | patrol     | 0.85 | keyword\_fallback |
| "deliver files to hardware room" | delivery   | 0.88 | keyword\_fallback |
| "check the supplies area"        | patrol（默认） | 0.55 | keyword\_fallback |

### 6.5 交互流程

```
用户输入自然语言
       │
       ▼
office_llm_command.py
       │
       ├── 读取 config.toml [llm] 配置
       │
       ├── API Key 非空？
       │     ├── 是 → 选择 openai / openai_compat 模式
       │     │         │
       │     │         ├── 调用远程 LLM API
       │     │         │     ├── 成功 → 解析 JSON 输出
       │     │         │     └── 失败 → 回退到 keyword_fallback
       │     │         │
       │     │         └── 验证 task ∈ {delivery, patrol}
       │     │               ├── 是 → 返回解析结果
       │     │               └── 否 → 回退到 keyword_fallback
       │     │
       │     └── 否 → 使用 mock 模式
       │
       ├── --execute 模式？
       │     ├── 是 → 执行 ros2 run office dispatch_{task}
       │     └── 否 → 仅输出 JSON（dry-run）
       │
       └── --save-json → 保存解析和执行证据到文件
```

***

## 7. LLM / VLM / VLA 集成

### 7.1 集成架构

本项目将 LLM 定位于**语义规划层**，负责将自然语言指令映射为可执行的任务类型：

```
自然语言输入
       │
       ▼
┌─────────────────────────────────┐
│       语义规划层 (LLM)           │
│  office_llm_command.py          │
│  provider: openai_compat        │
│  model: glm-4.5-air             │
│  base_url: open.bigmodel.cn     │
│                                 │
│  输入: 自然语言文本              │
│  输出: {task, confidence, reason}│
└──────────────┬──────────────────┘
               │
               ▼
┌─────────────────────────────────┐
│       任务执行层 (ROS 2)         │
│  dispatch_delivery / patrol     │
│  → RMF Dispatcher               │
│  → Fleet Adapter                │
│  → Robot Execution              │
└─────────────────────────────────┘
```

### 7.2 LLM 接入实现

**文件**：`office/scripts/office_llm_command.py`（483 行）

**核心设计决策**：

1. **零第三方依赖**：使用 `urllib.request` 而非 `openai` SDK，自实现 TOML 解析器
2. **多提供商兼容**：通过 `openai_compat` 模式支持智谱、DeepSeek、Moonshot 等
3. **结构化输出**：使用 `json_schema` 约束 LLM 输出格式，确保解析可靠性
4. **三级回退**：openai\_compat → mock → keyword\_fallback

**当前配置**（`config.toml`）：

```toml
[llm]
provider = "openai_compat"
model = "glm-4.5-air"
base_url = "https://open.bigmodel.cn/api/paas/v4"
api_key = "a6d3807a..."
timeout_sec = 20
```

### 7.3 可运行验证

**mock 模式验证**（无需 API Key）：

```bash
ros2 run office office_llm_command.py "请巡检办公室所有检查点"
# 输出: {"parsed": {"task": "patrol", "confidence": 0.86, "parser": "mock"}, ...}

ros2 run office office_llm_command.py "把文件送到硬件办公室"
# 输出: {"parsed": {"task": "delivery", "confidence": 0.90, "parser": "mock"}, ...}
```

**真实 API 验证**（智谱 GLM-4.5-air）：

```bash
ros2 run office office_llm_command.py --execute "please inspect the office"
# 调用智谱 API，解析为 patrol，执行 dispatch_patrol
```

### 7.4 推理延迟分析

| 解析器                          | 平均延迟    | P95 延迟   | 最大延迟     |
| ---------------------------- | ------- | -------- | -------- |
| openai\_compat (GLM-4.5-air) | \~800ms | \~1500ms | \~2000ms |
| mock                         | <1ms    | <1ms     | <1ms     |
| keyword\_fallback            | <1ms    | <1ms     | <1ms     |

**延迟来源分析**：

- 网络往返：\~200-500ms（取决于服务器位置和网络状况）
- 模型推理：\~300-1000ms（取决于输入长度和模型负载）
- JSON 解析：<1ms

### 7.5 输出可靠性分析

**观测到的失败模式**：

| 失败模式       | 描述                     | 频率 | 应对策略                       |
| ---------- | ---------------------- | -- | -------------------------- |
| 语义歧义       | 宽泛输入映射到错误任务类型          | 中等 | 限制输出 schema，低置信度时返回澄清提示    |
| 网络超时       | API 调用超过 20s           | 低  | 自动回退到 keyword\_fallback    |
| 场景外指令      | 超出 delivery/patrol 的命令 | 中等 | 拒绝或映射到最接近任务，标记低置信度         |
| 非法 JSON    | LLM 输出非标准 JSON         | 低  | 三级 JSON 提取策略（直接解析→代码块→花括号） |
| API Key 缺失 | 未配置 API Key            | -  | 自动切换 mock 模式               |

**典型失败案例**：

| 案例编号       | 输入             | LLM 输出              | 问题      | 处置                           |
| ---------- | -------------- | ------------------- | ------- | ---------------------------- |
| AI-CASE-01 | "帮我处理一下办公室的事情" | delivery/patrol 不稳定 | 语义不充分   | 返回低置信度，建议用户明确意图              |
| AI-CASE-02 | "去仓库搬东西"       | 试图映射到 delivery      | 场景外指令   | 拒绝 warehouse 命令，提示仅支持 office |
| AI-CASE-03 | 网络断开时的任意命令     | N/A                 | API 不可达 | 自动回退到 keyword\_fallback      |

### 7.6 LLM 命令解析实验

实验 5 对比了三种解析器在 5 条测试命令上的表现：

| 命令                               | 预期       | mock       | keyword\_fallback | openai\_compat |
| -------------------------------- | -------- | ---------- | ----------------- | -------------- |
| "patrol the office corridors"    | patrol   | patrol     | patrol            | patrol         |
| "deliver files to hardware room" | delivery | delivery   | delivery          | delivery       |
| "巡检办公室走廊"                        | patrol   | patrol     | patrol            | patrol         |
| "把文件送到硬件区"                       | delivery | delivery   | delivery          | delivery       |
| "check the supplies area"        | patrol   | patrol(默认) | patrol(默认)        | patrol         |

**结论**：对于明确的配送/巡检命令，三种解析器均能正确解析；对于模糊命令，均默认映射为 patrol。LLM 的主要价值在于处理更复杂的语义表达和上下文理解。

***

## 8. 系统与部署分析

### 8.1 硬件平台选型论证

| 组件    | 仿真配置                   | 真实部署建议                         | 选型依据                   |
| ----- | ---------------------- | ------------------------------ | ---------------------- |
| 底盘    | 差速驱动（0.82m×0.62m）      | 差速轮式底盘（如 AgileX Scout、思岚 Zeus） | 室内平坦地面，差速驱动结构简单可靠      |
| 主传感器  | gpu\_lidar（360 线，10Hz） | 2D LiDAR（如 RPLIDAR A2/A3，10Hz） | 室内导航 2D 激光足够，成本低于 3D   |
| 辅助传感器 | Ouster 3D 激光           | 可选（3D 场景理解需求时）                 | 增加环境感知冗余               |
| 视觉    | RGB 相机（640×480）        | RGB-D 相机（如 RealSense D435）     | 深度信息辅助避障和语义理解          |
| 惯性    | IMU（100Hz）             | IMU（如 BNO085）                  | 里程计融合，提升定位精度           |
| 定位    | GPS（仿真）                | 不需要（室内无 GPS）                   | 室内依赖 LiDAR + IMU 定位    |
| 计算    | Gazebo 仿真              | 板载工控机/NUC（i7 + 16GB + GPU 可选）  | Nav2 + AMCL + LLM 推理需求 |
| 负载    | 托盘 0.60m×0.42m         | 实际配送负载 5-10kg                  | 配送场景需求                 |

**成本估算**（单台）：

| 项目       | 估算成本               |
| -------- | ------------------ |
| 差速底盘     | ¥15,000-30,000     |
| 2D LiDAR | ¥1,500-5,000       |
| RGB-D 相机 | ¥2,000-4,000       |
| IMU      | ¥300-800           |
| 工控机/NUC  | ¥3,000-8,000       |
| 电池+充电系统  | ¥2,000-5,000       |
| **合计**   | **¥24,000-53,000** |

### 8.2 适用环境约束

| 约束    | 仿真假设    | 真实环境差异           |
| ----- | ------- | ---------------- |
| 地面    | 完美平坦    | 可能有门槛、坡度、湿滑      |
| 障碍物   | 静态+简单动态 | 行人、推车、门等复杂动态     |
| 传感器噪声 | 理想模型    | 玻璃反射、黑色物体误检、阳光干扰 |
| 通信    | 零延迟     | WiFi 延迟和丢包       |
| 执行器   | 瞬时响应    | 电机延迟、轮胎打滑        |

### 8.3 Sim-to-Real 迁移策略

| 技术障碍           | 具体表现                | 应对策略                                                                            |
| -------------- | ------------------- | ------------------------------------------------------------------------------- |
| **LiDAR 噪声差异** | 仿真激光无玻璃反射、无黑色物体漏检   | 对 LaserScan 做滤波（中值+统计滤波），调整 AMCL 激光模型参数（`laser_model_type`、`z_hit`、`z_short`）   |
| **地图与真实环境不一致** | 仿真地图与实际建筑有偏差        | 真实场地重新建图（`ros2 launch slam_toolbox online_async_launch.py`），保留局部 costmap 在线更新能力 |
| **执行器响应延迟**    | 仿真中速度指令瞬时生效，真实电机有惯性 | 降低最大速度/加速度（0.5m/s、1.0m/s²），标定轮径和轴距，重调 RPP 控制器参数                                 |
| **初始定位偏差**     | 仿真中初始位姿精确已知         | 固定起点，使用 `initialpose` 工具设定，建立重定位检查流程（AMCL 收敛验证）                                 |
| **动态障碍物**      | 仿真中障碍物简单且可预测        | 使用更保守的 inflation 半径和 collision\_monitor 参数，加入暂停与人工接管机制                          |
| **LLM 网络失败**   | 仿真中 API 调用稳定        | 保留 keyword\_fallback 规则解析，设置超时和置信度阈值，低置信度时要求人工确认                                |
| **WiFi 通信不稳定** | 仿真中通信零延迟            | 使用 QoS 策略适配（BEST\_EFFORT 用于传感器，RELIABLE 用于控制），增加心跳检测和重连机制                       |
| **电池与续航**      | 仿真中电量无限             | 实现低电量自动返航充电，任务调度考虑电量约束                                                          |

### 8.4 迁移优先级

1. **高优先级**：传感器噪声适配、地图重建、执行器标定
2. **中优先级**：动态障碍物策略调优、通信稳定性保障
3. **低优先级**：LLM 离线部署、电池管理

***

## 9. 实验结果综合评估

### 9.1 阶段三实验总览

| # | 实验        | 描述         | 结果   |
| - | --------- | ---------- | ---- |
| 1 | baseline  | 无障碍基线导航    | PASS |
| 2 | obstacle  | 有障碍物避障导航   | PASS |
| 3 | scheduler | 任务调度器优化对比  | PASS |
| 4 | combined  | 调度器+避障联合测试 | PASS |

**总体成功率**：4/4（100%）

### 9.2 避障效果对比

| 指标    | 基线（无障碍） | 有障碍物  | 差值              |
| ----- | ------- | ----- | --------------- |
| 完成时间  | 46.1s   | 62.5s | +16.4s (+35.6%) |
| 行驶距离  | 1.44m   | 1.81m | +0.37m (+25.7%) |
| 成功率   | 100%    | 100%  | -               |
| 重规划次数 | 0       | 0     | -               |

### 9.3 调度器优化效果

| 策略                   | 总时间   | 总距离   | 节省                 |
| -------------------- | ----- | ----- | ------------------ |
| 顺序派发                 | 92.6s | 4.73m | -                  |
| nearest\_neighbor 优化 | 78.5s | 5.54m | 时间节省 14.1s (15.3%) |

**分析**：nearest\_neighbor 策略通过贪心选择最近任务减少了总时间 15.3%，但总距离略增（+0.81m），因为优化后路线可能包含更远但时间更高效的路径段。

### 9.4 规划器×避障配置矩阵（阶段二补充）

| 规划器          | 避障配置               | delivery 成功率 | patrol 成功率  | delivery 平均耗时 |
| ------------ | ------------------ | ------------ | ----------- | ------------- |
| navfn\_astar | baseline\_costmap  | 100.00%      | 100.00%     | 7.656s        |
| navfn\_astar | collision\_monitor | 60.00%       | 100.00%     | 51.708s       |
| smac\_2d     | baseline\_costmap  | 100.00%      | 100.00%     | 10.183s       |
| smac\_2d     | collision\_monitor | **100.00%**  | **100.00%** | **8.252s**    |

**推荐组合**：`smac_2d + collision_monitor`

### 9.5 Stage 3 功能测试

自动化测试套件覆盖 6 个类别、28 个测试用例，全部通过：

| 类别     | 通过 | 失败 | 总计 |
| ------ | -- | -- | -- |
| 静态验证   | 3  | 0  | 3  |
| 配置文件   | 4  | 0  | 4  |
| LLM 集成 | 6  | 0  | 6  |
| 任务调度器  | 7  | 0  | 7  |
| 动态避障   | 6  | 0  | 6  |
| 构建配置   | 2  | 0  | 2  |

***

## 10. 课程技术要求覆盖总结

| 技术要求              | 基本要求                      | 完成情况                                             | 进阶尝试                   |
| ----------------- | ------------------------- | ------------------------------------------------ | ---------------------- |
| **感知与环境建模**       | 至少一种传感器模态+占据栅格地图          | ✅ 激光雷达+OccupancyGrid+AMCL                        | ✅ 合成激光+3D激光+多传感器       |
| **实时定位**          | AMCL/GMapping/Hector SLAM | ✅ AMCL（2000粒子）                                   | ✅ HDL全局定位+EKF          |
| **运动控制与避障**       | 基本运动控制+2种避障策略对比           | ✅ RPP控制器+collision\_monitor vs baseline\_costmap | ✅ 动态避障监控+RMF重规划        |
| **路径规划**          | 2+种规划算法对比                 | ✅ NavfnPlanner(A\*) vs SmacPlanner2D             | ✅ 基于Nav2的完整导航栈集成调优     |
| **应用场景**          | 2+个独立任务逻辑+成功/失败判定         | ✅ delivery+patrol+量化评估                           | ✅ 清洁/循环/远程操控等扩展场景      |
| **人机交互**          | 至少一种可运行交互接口               | ✅ 自然语言命令接口                                       | ✅ LLM→ROS任务映射+JSON证据保存 |
| **LLM/VLM/VLA集成** | 至少一个基础模型接入+可运行实现          | ✅ 智谱GLM-4.5-air接入+端到端跑通                          | ✅ 失效案例分析+延迟/可靠性评估      |
| **系统与部署分析**       | 硬件选型论证+sim-to-real迁移策略    | ✅ 完整硬件方案+8项迁移障碍及应对                               | -                      |

***

## 11. AI 工具使用专项说明

### 11.1 所用工具及作用

| 工具                      | 版本                         | 作用                              |
| ----------------------- | -------------------------- | ------------------------------- |
| Trae IDE (Coding Agent) | GLM-5.1                    | 代码编写、调试、重构、文档撰写、实验脚本开发          |
| ChatGPT / Codex         | GPT-5.x                    | 代码阅读、实验结果分析、报告结构整理              |
| 智谱 GLM-4.5-air          | glm-4.5-air                | 机器人自然语言指令的 LLM 语义解析层            |
| OpenAI Responses API    | gpt-4o-mini                | Nav2 模式下的自然语言解析（备用）             |
| 本地规则解析器                 | keyword\_fallback          | 无 API Key / 网络失败 / LLM 超时时的兜底解析 |
| 自动化实验脚本                 | office\_experiment.py      | 批量运行避障/调度/LLM 实验，生成结构化报告        |
| Nav2 配置生成器              | generate\_nav2\_profile.py | 根据规划器×避障配置矩阵自动生成参数文件            |
| Stage 3 测试套件            | test\_stage3\_features.py  | 28 项自动化功能测试                     |

### 11.2 估算调用成本与资源消耗

**LLM API 调用成本**：

| 用途            | 调用次数      | 单次 token 量 | 总 token 量 | 估算费用       |
| ------------- | --------- | ---------- | --------- | ---------- |
| 自然语言解析测试      | 50-200 次  | 300-1200   | \~1e4-1e5 | ¥5-20（智谱）  |
| 开发辅助（ChatGPT） | 100-300 次 | 500-2000   | \~1e5-5e5 | ¥10-50     |
| **合计**        | -         | -          | -         | **¥15-70** |

**计算资源消耗**：

| 资源     | 消耗                                       |
| ------ | ---------------------------------------- |
| CPU    | 仿真与导航主耗时在 ROS/Gazebo 运行阶段                |
| 内存     | 中等负载（Office 单 world + Nav2 + RMF + 任务节点） |
| GPU    | Gazebo 渲染（可选 headless 模式）                |
| 单轮实验耗时 | 4 组矩阵 × TRIALS=5，约 40-120 分钟             |

### 11.3 观察到的局限性与失败案例

**Coding Agent 局限性**：

1. 对 ROS 2 launch 文件的 XML 语法理解偶有偏差，需人工校验
2. 生成的 Nav2 参数文件需要根据实际场景微调，不能直接使用默认值
3. 对 RMF 内部调度逻辑的理解有限，fleet\_adapter 代码需人工审查

**LLM 解析局限性**：

1. 语义歧义：宽泛输入可能在 delivery/patrol 间不稳定
2. 网络依赖：API 调用受网络稳定性影响
3. 场景外指令：超出 delivery/patrol 的命令会被强制映射
4. 延迟波动：高峰时段响应延迟增大

**典型失败案例**：

| 案例编号       | 输入                                | LLM 输出              | 问题         | 处置                      |
| ---------- | --------------------------------- | ------------------- | ---------- | ----------------------- |
| AI-CASE-01 | "帮我处理一下办公室的事情"                    | delivery/patrol 不稳定 | 语义不充分      | 限制输出 schema，低置信度返回澄清提示  |
| AI-CASE-02 | "去仓库搬东西"                          | 试图映射到 delivery      | 场景外指令      | 拒绝 warehouse 命令         |
| AI-CASE-03 | 网络断开时的任意命令                        | N/A                 | API 不可达    | 自动回退到 keyword\_fallback |
| AI-CASE-04 | navfn+collision\_monitor delivery | FAILED/TIMEOUT      | 规划器与避障策略冲突 | 切换到 smac\_2d            |

### 11.4 提示词与工具脚本

**LLM 系统提示词**：

```
Parse an Office robot service command. Only choose delivery or patrol.
Reject warehouse or unsupported robot tasks by mapping to the closest
Office task with low confidence.
```

**关键工具脚本**：

- `office/scripts/office_llm_command.py` — LLM 命令解析器
- `office/scripts/office_experiment.py` — 自动化实验运行器
- `office/scripts/office_task_scheduler.py` — 多任务优化调度器
- `office/scripts/office_dynamic_obstacle_avoidance.py` — 动态避障监控
- `robotics_nav2/scripts/generate_nav2_profile.py` — Nav2 配置生成器
- `test_stage3_features.py` — Stage 3 功能测试套件
- `tools/run_stage2_experiments_wsl.sh` — WSL 环境实验启动脚本

### 11.5 对 AI 生成内容的验证与评估方法

1. **语义解析验证**：对每条自然语言命令记录输入、解析结果、延迟、执行结果；低置信度或 fallback 接管样本人工复核
2. **任务执行验证**：统一实验脚本重复运行，输出 CSV/JSONL；以成功率、耗时、超时率作为核心指标
3. **结果一致性验证**：同一配置至少 3 次重复试验；波动异常时增加重复次数
4. **文档可追溯性验证**：所有关键结论可回溯到对应结果文件与日志
5. **代码审查**：AI 生成的代码、脚本和报告文本经人工审阅
6. **抽检比例**：语义命令样本 ≥30%，实验结果文件 ≥20%，关键结论 100%

***

## 12. 系统启动与演示指南

### 12.1 环境准备

```bash
# 安装运行依赖
sudo apt install -y python3-socketio python3-fastapi python3-uvicorn python3-pydantic

# 构建
cd ~/ros_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  office office_gz office_demos office_maps office_assets \
  office_fleet_adapter office_tasks
source install/setup.bash
```

### 12.2 启动仿真

```bash
# 终端 1：启动 Office 仿真 + 感知建图
ros2 launch office office_perception_mapping.launch.xml headless:=false
```

等待日志出现 `Successfully added new robot: tinyRobot1`。

### 12.3 派发任务

```bash
# 终端 2：配送任务
ros2 run office_tasks dispatch_delivery --use_sim_time

# 终端 2：巡检任务
ros2 run office_tasks dispatch_patrol -p tinyRobot1_charger patrol_A1 -n 1 --use_sim_time

ros2 run office dispatch_patrol --use_sim_time -p  patrol_A1 patrol_A2 -n 1

# 终端 2：多任务优化调度
ros2 run office office_task_scheduler.py --use_sim_time \
  "delivery:supplies:hardware" \
  "go_to_place:lounge" \
  "patrol:charger,patrol_a1,patrol_a2,charger"
```

### 12.4 启动动态避障

```bash
# 终端 3
ros2 run office office_dynamic_obstacle_avoidance.py
```

### 12.5 LLM 自然语言交互

```bash
# 终端 4：mock 模式
ros2 run office office_llm_command.py --execute "请巡检办公室所有检查点"

# 终端 4：真实 API
ros2 run office office_llm_command.py --execute "please inspect the office"
```

### 12.6 运行完整实验

```bash
# 终端 2：运行所有实验
ros2 run office office_experiment.py --all

# 生成报告
ros2 run office office_experiment.py --report-only
```

***

## 附录 A：ROS 2 包清单

| 包名                     | 类型            | 功能                         |
| ---------------------- | ------------- | -------------------------- |
| office                 | ament\_cmake  | Office 场景入口（LLM、调度器、避障、实验） |
| office\_tasks          | ament\_python | RMF 任务调度脚本                 |
| office\_fleet\_adapter | ament\_python | RMF 车队适配器                  |
| office\_maps           | ament\_cmake  | Office 导航地图与建筑数据           |
| office\_gz             | ament\_cmake  | Office Gazebo 仿真启动         |
| office\_demos          | ament\_cmake  | Office RMF 演示启动与配置         |
| office\_assets         | ament\_cmake  | Office 3D 模型资源             |
| robotics\_description  | ament\_cmake  | 机器人 URDF/xacro 描述          |
| robotics\_gazebo       | ament\_cmake  | Gazebo 世界文件                |
| robotics\_nav2         | ament\_cmake  | Nav2 参数配置 + 地图 + 启动文件      |
| robotics\_scenario     | ament\_cmake  | 行为树 + 场景管理器 + NL/LLM 命令解析  |
| robotics\_interfaces   | ament\_cmake  | 自定义 ROS 2 接口               |
| robotics\_localization | ament\_cmake  | EKF + HDL 定位               |

## 附录 B：关键文件索引

| 功能                 | 文件路径                                                              |
| ------------------ | ----------------------------------------------------------------- |
| LLM 命令解析           | `office/scripts/office_llm_command.py`                            |
| 动态避障监控             | `office/scripts/office_dynamic_obstacle_avoidance.py`             |
| 任务调度器              | `office/scripts/office_task_scheduler.py`                         |
| 实验运行器              | `office/scripts/office_experiment.py`                             |
| 合成激光雷达             | `office/scripts/office_synthetic_lidar.py`                        |
| 栅格建图               | `office/scripts/office_scan_mapper.py`                            |
| Nav2 参数            | `robotics_nav2/param/office_nav2.yaml`                            |
| Nav2 配置生成器         | `robotics_nav2/scripts/generate_nav2_profile.py`                  |
| 机器人 URDF           | `robotics_description/urdf/robotics.urdf.xacro`                   |
| Gazebo 传感器配置       | `robotics_description/urdf/robotics.gazebo.xacro`                 |
| Fleet Adapter      | `office_fleet_adapter/office_fleet_adapter/fleet_adapter.py`      |
| RobotCommandHandle | `office_fleet_adapter/office_fleet_adapter/RobotCommandHandle.py` |
| 全局配置               | `config.toml`                                                     |
| 拓扑地图               | `robotics_nav2/map/office_topology.yaml`                          |
| 行为树                | `robotics_scenario/behavior_trees/office_delivery.xml`            |
| 自定义 Action         | `robotics_interfaces/robotics_interfaces/action/Delivery.action`  |
| Stage 3 测试         | `test_stage3_features.py`                                         |
| 实验结果               | `experiment_results/experiment_report.md`                         |
| 阶段二矩阵              | `experiment_results/stage2_matrix/stage2_matrix_summary.csv`      |

## 附录 C：实验数据文件

| 文件                                                           | 说明          |
| ------------------------------------------------------------ | ----------- |
| `experiment_results/experiment_results.json`                 | 阶段三实验原始数据   |
| `experiment_results/experiment_report.md`                    | 阶段三实验报告     |
| `experiment_results/stage2_matrix/stage2_matrix_summary.csv` | 阶段二规划器矩阵汇总  |
| `experiment_results/stage2_matrix/stage2_matrix_summary.md`  | 阶段二规划器矩阵报告  |
| `experiment_results/stage2_matrix/*.jsonl`                   | 阶段二各组实验详细数据 |
| `experiment_results/stage2_matrix/*.csv`                     | 阶段二各组实验汇总   |

