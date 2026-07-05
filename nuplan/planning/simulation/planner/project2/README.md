# Project2 —— 基于 nuPlan 的横纵向解耦规划器

本项目在 [nuPlan](https://nuplan-devkit.readthedocs.io/) 仿真框架上实现了一套完整的**横纵向解耦自动驾驶规划器**，包含全局路由、参考线生成、障碍物预测、五次多项式路径规划、ST 图动态规划速度决策以及轨迹融合等模块。

---

## 目录

1. [项目结构](#1-项目结构)
2. [算法架构与数据流](#2-算法架构与数据流)
3. [模块详解](#3-模块详解)
   - [3.1 BFSRouter — 全局路由](#31-bfsrouter--全局路由)
   - [3.2 ReferenceLineProvider — 参考线生成](#32-referencelineprovider--参考线生成)
   - [3.3 SimplePredictor — 障碍物预测](#33-simplepredictor--障碍物预测)
   - [3.4 横向路径规划 (五次多项式)](#34-横向路径规划-五次多项式)
   - [3.5 纵向速度规划 (ST 图 DP)](#35-纵向速度规划-st-图-dp)
   - [3.6 FrameTransform — Frenet ↔ Cartesian 坐标变换](#36-frametransform--frenet--cartesian-坐标变换)
   - [3.7 MergePathSpeed — 路径与速度融合](#37-mergepathspeed--路径与速度融合)
   - [3.8 MyPlanner — 顶层规划器](#38-myplanner--顶层规划器)
4. [运行方法](#4-运行方法)
5. [参数说明](#5-参数说明)
6. [二次开发指南](#6-二次开发指南)

---

## 1. 项目结构

```
nuplan/planning/simulation/planner/project2/
├── my_planner.py              # 顶层规划器入口，实现 AbstractPlanner 接口
├── bfs_router.py              # 全局路由：BFS 车道图搜索
├── reference_line_provider.py # 参考线生成与管理
├── simple_predictor.py        # 障碍物预测（恒定速度模型）
├── abstract_predictor.py      # 预测器抽象基类
├── dp_decider.py              # ST 图动态规划速度决策
├── frame_transform.py         # Frenet ↔ Cartesian 坐标变换
├── merge_path_speed.py        # 路径与速度结果融合，输出轨迹点
└── __init__.py                # 包初始化
```

---

## 2. 算法架构与数据流

`MyPlanner.compute_planner_trajectory()` 每个仿真步执行一次，内部按 **4 个阶段** 串行运行：

```
┌───────────────────────────────────────────────────────────────────┐
│                   compute_planner_trajectory()                    │
│                                                                   │
│  ① Routing（全局路由）                                             │
│     BFSRouter: 车道图 BFS 搜索 → 离散路径点 + 边界 + 限速          │
│         ↓                                                         │
│  ② Reference Line（参考线生成）                                    │
│     ReferenceLineProvider: 以自车为中心前 200m/后 30m              │
│     重采样 Δs=1m → x/y/heading/κ/边界                             │
│         ↓                                                         │
│  ③ Prediction（障碍物预测）                                        │
│     SimplePredictor: 40m 范围内 → 恒定速度外推 → PredictedTrajectory│
│         ↓                                                         │
│  ④ Planning（轨迹规划）                                            │
│     ├─ 横向: 五次多项式 path_planning() — Frenet 坐标             │
│     ├─ 变换: transform_path_planning() — Frenet → Cartesian       │
│     ├─ 纵向: speed_planning() + DpDecider — ST 图 DP              │
│     └─ 融合: merge_path_speed → EgoState 序列                     │
└───────────────────────────────────────────────────────────────────┘
```

---

## 3. 模块详解

### 3.1 BFSRouter — 全局路由

文件: [bfs_router.py](bfs_router.py)

**功能**：从 nuPlan 地图 API 搜索从自车位置到目标区域的可行驶路径。

**核心流程**：
1. `_initialize_route_plan(route_roadblock_ids)` — 从地图 API 读取 Route 上的 Roadblock，提取所有 candidate lane edge IDs
2. `_get_starting_edge(ego_state)` — 在前 5 个 roadblock 中定位自车所在车道
3. `_breadth_first_search(ego_state)` — 从起始 edge 出发，BFS 搜索到目标 roadblock 的路径
4. `_initialize_ego_path(ego_state, max_velocity)` — 沿 BFS 路径提取离散路径点、左右边界距离、限速和弧长

**输出属性**：
| 属性 | 类型 | 说明 |
|------|------|------|
| `_discrete_path` | `List[StateSE2]` | 离散路径点（x, y, heading） |
| `_s_of_path` | `List[float]` | 路径点对应的弧长 |
| `_lb_of_path` | `List[float]` | 各路径点到左边界距离 |
| `_rb_of_path` | `List[float]` | 各路径点到右边界距离 |
| `_max_v_of_path` | `List[float]` | 各路径点限速 |
| `_edge_of_path` | `List[LaneGraphEdgeMapObject]` | 路径经过的 lane edges |

---

### 3.2 ReferenceLineProvider — 参考线生成

文件: [reference_line_provider.py](reference_line_provider.py)

**功能**：以自车后轴为原点，对全局路径重采样生成平滑参考线。

**核心流程**：
1. 在全局离散路径上搜索离自车后轴最近的点作为原点
2. **前向采样**：从原点沿路径向前 200m，Δs = 1m，线性插值得到 (x, y)
3. **后向采样**：从原点沿路径向后 30m（若路径不足则截断），Δs = 1m
4. 用有限差分计算 heading 和曲率 κ

**输出属性**：
| 属性 | 说明 |
|------|------|
| `_x_of_reference_line` / `_y_of_reference_line` | 参考线坐标 |
| `_s_of_reference_line` | 参考线弧长（以自车为原点，s=0） |
| `_heading_of_reference_line` | 参考线各点朝向 |
| `_kappa_of_reference_line` | 参考线各点曲率 |
| `_lb_of_reference_line` / `_rb_of_reference_line` | 左右边界距离 |
| `_interp1d_x / _y / _heading / _kappa` | scipy 插值器，支持任意 s 查询 |

**关键方法**：
- `get_boundary(s_set)` — 根据弧长列表返回左右边界（左正右负）

---

### 3.3 SimplePredictor — 障碍物预测

文件: [simple_predictor.py](simple_predictor.py)

**功能**：对自车周围障碍物的未来轨迹进行预测。

**算法**：
1. 从 `DetectionsTracks` 观测中筛选自车 **40m** 范围内的目标（欧氏距离）
2. 对每个目标使用**恒定速度 + 恒定角速度**模型外推：
   - `x(t) = x₀ + vx · t`
   - `y(t) = y₀ + vy · t`
   - `heading(t) = heading₀ + ω · t`
3. 时间步数 = `duration / sample_time`，生成 `PredictedTrajectory`（概率 = 1.0）
4. 通过 `object.predictions` 设置到每个 Agent 上

**基类**: [abstract_predictor.py](abstract_predictor.py) — `AbstractPredictor`，定义 `predict()` 抽象接口。

> **扩展点**：继承 `AbstractPredictor` 并实现 `predict()` 即可替换为基于学习的预测器。

---

### 3.4 横向路径规划 (五次多项式)

文件: [my_planner.py](my_planner.py) → `path_planning()`

**功能**：在 Frenet 坐标系中规划一条从当前侧向偏移平滑收敛到参考线中心的路径。

**核心流程**：
1. **坐标变换**：将自车后轴状态从 Cartesian → Frenet，得到 `(s₀, l₀, l̇₀, l̈₀)`
2. **五次多项式求解**：解六元线性方程组 Ax = b：

   ```
   边界条件:
     l(0) = l₀,      l'(0) = l̇₀,      l''(0) = l̈₀
     l(L) = 0,       l'(L) = 0,        l''(L) = 0

   其中 L = 30m（横向收敛距离）
   ```

3. **边界裁剪**：将 `l(s)` 限制在参考线左右边界内
4. **输出**：`(path_l, path_dl, path_ddl, path_s)`，其中 s 范围最长 150m，Δs = 1m

**关键参数**：
| 参数 | 值 | 说明 |
|------|-----|------|
| `LATERAL_CONV_DIST` | 30 m | 横向回中收敛距离 |
| `delta_s` | 1 m | 路径采样间隔 |
| `max_plan_length` | 150 m | 路径规划最大长度 |

---

### 3.5 纵向速度规划 (ST 图 DP)

文件: [my_planner.py](my_planner.py) → `speed_planning()` + [dp_decider.py](dp_decider.py)

**两阶段流程**：

#### 阶段一：障碍物轨迹预测（speed_planning 内）
- 对每个障碍物以恒定速度模型生成 `(t, x, y)` 轨迹序列
- 计算障碍物半长作为碰撞安全半径

#### 阶段二：ST 图动态规划（DpDecider）

**ST 图构建**：
1. 将每个障碍物的 `(t, x, y)` 轨迹投影到规划路径上，得到 `(t, s)` 序列
2. 仅保留 `|l| ≤ ego_half_width` 的投影点（障碍物真正与路径有交集的部分）
3. 用 `scipy.interpolate.interp1d` 插值，支持查询任意时刻 t 的障碍物弧长 s

**DP 搜索**：
- **状态网格**：时间 × 弧长（T × S）
  - T 轴：`Δt = 0.5s`，范围 `[0, horizon_time]`
  - S 轴：`Δs = 2m`，范围 `[0, max_v × horizon_time]`
- **代价函数**（三部分组成）：

| 代价项 | 权重 | 说明 |
|--------|------|------|
| `cost_ref_speed` | 40 | 趋近参考速度（越接近 max_v 越小；超速惩罚 ×100） |
| `cost_accel` | 10 | 平滑加速度变化（超出 [-4, 3] m/s² 惩罚 ×1000） |
| `cost_obs` | 200 | 避碰（距离 < buffer 则 ∞；buffer~3×buffer 指数衰减） |

- **递推**：对 DP 表格每列每个节点，遍历前一列所有节点，选择累计代价最小者
- **回溯**：从终点列代价最小的节点反向回溯得到最优 `s(t)` 序列
- **边界提取**：根据最优 `s(t)` 判断每个障碍物的避让/超车决策，输出 `s_lb(t)` 和 `s_ub(t)`

**差分求解**（speed_planning 内）：
- 有限差分由 `s(t)` → `ṡ(t)` → `s̈(t)`
- 在 t=0 处补入初始状态

**调试**：每次调用自动保存 ST 图到 `images/figure_<timestamp>.png`。

---

### 3.6 FrameTransform — Frenet ↔ Cartesian 坐标变换

文件: [frame_transform.py](frame_transform.py)

**核心函数**：

| 函数 | 功能 |
|------|------|
| `get_match_point()` | 在参考线上搜索给定点的最近匹配点（最近邻搜索） |
| `cal_project_point()` | 由匹配点计算投影点的 (x, y, heading, κ, s) |
| `cartesian2frenet()` | Cartesian → Frenet，输出 (s, l, ṡ, l̇, dl, l̈, s̈, ddl) |
| `frenet2cartesian()` | Frenet → Cartesian，输出 (x, y, heading, κ) |
| `local2global_vector()` | 局部坐标系向量 → 全局坐标系向量 |

**Frenet 转换公式**（参考 Moritz Werling 的 Optimal Trajectory Generation）：

```
s  = s_proj
l  = (r_h - r_r) · n_r

ṡ = (v_h · t_r) / (1 - κ_r · l)
l̇ = v_h · n_r

dl = tan(Δθ) · (1 - κ_r · l)       (Δθ = θ_e - θ_r)

s̈ = (a_h · t_r + 2·κ_r·dl·ṡ²) / (1 - κ_r · l)
l̈ = a_h · n_r - κ_r · (1 - κ_r · l) · ṡ²
```

---

### 3.7 MergePathSpeed — 路径与速度融合

文件: [merge_path_speed.py](merge_path_speed.py)

**三个核心函数**：

| 函数 | 功能 |
|------|------|
| `transform_path_planning()` | Frenet 路径 (s, l) → Cartesian (x, y, heading, κ)，建立弧长索引 `path_idx2s` |
| `cal_dynamic_state(t, ...)` | 三次多项式插值：给定时间 t，从速度规划结果中插值 (s, ṡ, s̈) |
| `cal_pose(s, ...)` | 一维插值：给定弧长 s，从 Cartesian 路径中插值 (x, y, heading, κ) |

**融合流程**（在 `MyPlanner.planning()` 中）：
```
for each timestep t:
    s, v, a = cal_dynamic_state(t, speed_profile)
    x, y, heading, κ = cal_pose(s, path_profile)
    δ = arctan(wheelbase × κ)     # 运动学自行车模型
    state = EgoState(x, y, heading, v, a, δ, t)
```

---

### 3.8 MyPlanner — 顶层规划器

文件: [my_planner.py](my_planner.py)

**继承**: `AbstractPlanner`（nuPlan 框架标准接口）

**关键方法**：

| 方法 | 调用时机 | 说明 |
|------|----------|------|
| `initialize(init)` | 场景开始时 | 创建 BFSRouter，初始化路由 |
| `observation_type()` | 每步 | 声明使用 `DetectionsTracks` |
| `compute_planner_trajectory(input)` | **每仿真步** | 四阶段规划主循环 |
| `planning(...)` | `compute_planner_trajectory` 内部 | 横纵向解耦规划 + 轨迹融合 |

**`compute_planner_trajectory()` 内部流程**：
```python
# 1. Routing — 首次调用时初始化 BFS 路径
self._router._initialize_ego_path(ego_state, self.max_velocity)

# 2. Reference Line — 以自车为中心生成参考线
self._reference_path_provider._reference_line_generate(ego_state)

# 3. Prediction — 预测 40m 范围内障碍物未来轨迹
objects = SimplePredictor(...).predict()

# 4. Planning — 横纵向解耦规划
trajectory = self.planning(ego_state, reference_path, objects, ...)
```

---

## 4. 运行方法

### 4.1 环境准备

```bash
conda activate nuplan
cd ~/MyProject/nuplan-devkit
```

确认环境变量：
```bash
export NUPLAN_DATA_ROOT=~/nuplan/dataset
export NUPLAN_MAPS_ROOT=~/nuplan/dataset/maps
export NUPLAN_EXP_ROOT=~/nuplan/exp
```

### 4.2 单核运行（调试用）

```bash
NUPLAN_WORKER=sequential python nuplan/planning/script/run_planner.py
```

### 4.3 多核并行运行（推荐）

```bash
NUPLAN_WORKER=single_machine_thread_pool \
NUPLAN_WORKER_MAX_WORKERS=8 \
NUPLAN_CPUS_PER_SIMULATION=1 \
NUPLAN_WORKER_USE_PROCESS_POOL=true \
python nuplan/planning/script/run_planner.py
```

### 4.4 并行粒度说明

- nuPlan 的并行单位是 **场景（scenario）**，多场景分发到多进程
- 单场景内部仿真循环是串行的
- 必须使用**进程池**（`NUPLAN_WORKER_USE_PROCESS_POOL=true`）绕过 Python GIL

---

## 5. 参数说明

### 5.1 MyPlanner 构造参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `horizon_seconds` | float | 8.0 | 规划时域（秒） |
| `sampling_time` | float | 0.25 | 轨迹采样间隔（秒） |
| `max_velocity` | float | 17.0 | 自车最大速度（m/s，约 61 km/h） |

### 5.2 DpDecider 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `dp_step` | 0.5 s | ST 图时间分辨率 |
| `delta_s` | 2 m | ST 图弧长分辨率 |
| `max_acc` | 3.0 m/s² | 最大加速度 |
| `max_dec` | -4.0 m/s² | 最大减速度 |
| `w_cost_ref_speed` | 40 | 参考速度代价权重 |
| `w_cost_accel` | 10 | 加速度变化代价权重 |
| `w_cost_obs` | 200 | 障碍物代价权重 |

### 5.3 SimplePredictor 参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `_occupancy_map_radius` | 40 m | 感知范围半径 |
| `duration` | horizon_seconds | 预测时长 |
| `sample_time` | sampling_time | 预测步长 |

---

## 6. 二次开发指南

### 6.1 替换预测模块

1. 继承 `AbstractPredictor`（[abstract_predictor.py](abstract_predictor.py)）
2. 实现 `predict()` 方法，返回包含 `.predictions` 属性的 Agent 列表
3. 在 `MyPlanner.compute_planner_trajectory()` 中替换 `SimplePredictor`

### 6.2 替换横向/纵向规划

- **横向**：修改 `path_planning()`，保持输出签名 `(path_l, path_dl, path_ddl, path_s)` 不变
- **纵向**：修改 `speed_planning()` 或 `DpDecider`，保持输出签名 `(s, s_dot, s_2dot, t)` 不变

### 6.3 基于采样的规划器

可在 `MyPlanner.planning()` 中将横纵向解耦替换为统一采样规划（如 Lattice Planner、RRT*），只需保证最终输出 `List[EgoState]` 格式不变。

### 6.4 调试 ST 图

`DpDecider.dynamic_programming()` 中已内置 matplotlib 绘图，每次调用自动保存图片到 `images/` 目录。若需实时显示，取消 `plt.show()` 的注释即可。

### 6.5 查看仿真结果

仿真完成后自动启动 **nuBoard**（Bokeh Web 界面）。结果文件保存在：
```
~/nuplan/exp/exp/simulation/closed_loop_nonreactive_agents/<timestamp>/
```

也可手动启动：
```bash
python nuplan/planning/script/run_nuboard.py \
  scenario_builder=nuplan_mini \
  simulation_path=[<path_to_.nuboard_file>]
```
