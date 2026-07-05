# MyPlanner —— 基于 nuPlan 的自动驾驶规划器

本项目在 [nuPlan](https://nuplan-devkit.readthedocs.io/) 仿真框架上实现了一套完整的**横纵向解耦规划器**（`MyPlanner`），并提供多核并行仿真脚本。适合用于自动驾驶规划算法的学习、实验与评测。

---

## 目录

1. [项目结构](#1-项目结构)
2. [算法架构](#2-算法架构)
3. [各模块详解](#3-各模块详解)
4. [运行方法](#4-运行方法)
5. [参数说明](#5-参数说明)
6. [场景配置](#6-场景配置)
7. [多核并行说明](#7-多核并行说明)
8. [结果可视化](#8-结果可视化)
9. [二次开发指南](#9-二次开发指南)

---

## 1. 项目结构

```
nuplan/planning/simulation/planner/project2/
├── my_planner.py            # 顶层规划器（入口）
├── bfs_router.py            # 全局路由：BFS 车道图搜索
├── reference_line_provider.py # 参考线生成与管理
├── simple_predictor.py      # 障碍物预测（匀速模型）
├── abstract_predictor.py    # 预测器抽象基类
├── dp_decider.py            # ST 图动态规划速度决策
├── frame_transform.py       # Frenet ↔ Cartesian 坐标变换
└── merge_path_speed.py      # 路径与速度结果融合，输出轨迹点

nuplan/planning/script/
└── run_planner.py           # 仿真启动脚本（支持多核并行）
```

---

## 2. 算法架构

`MyPlanner` 每个仿真步执行一次 `compute_planner_trajectory()`，内部按以下 **4 个阶段**串行运行：

```
┌──────────────────────────────────────────────────────────┐
│                    compute_planner_trajectory             │
│                                                          │
│  ① Routing（全局路由）                                    │
│     BFSRouter：在车道图中 BFS 搜索，生成离散路径点          │
│         ↓                                                │
│  ② Reference Line（参考线生成）                           │
│     ReferenceLineProvider：以自车为中心前 200m/后 30m     │
│     重采样（Δs=1m），输出 x/y/heading/κ/边界              │
│         ↓                                                │
│  ③ Prediction（障碍物预测）                               │
│     SimplePredictor：40m 内障碍物，匀速外推               │
│         ↓                                                │
│  ④ Planning（轨迹规划）                                   │
│     横向：五次多项式（Frenet 坐标）                        │
│     纵向：ST 图 DP（DpDecider）                           │
│     融合：merge_path_speed → EgoState 序列               │
└──────────────────────────────────────────────────────────┘
```

---

## 3. 各模块详解

### 3.1 BFSRouter（全局路由）

文件：`bfs_router.py`

- 从 nuPlan 地图 API 读取 **Roadblock → Lane** 层级结构
- 以自车所在车道为起点，对预设路由 roadblock 列表做 **BFS 搜索**，拼接出一条可行驶的离散路径
- 同时记录每个路径点的**左右边界距离**和**限速**，供后续横向规划使用

### 3.2 ReferenceLineProvider（参考线）

文件：`reference_line_provider.py`

- 以**自车后轴**在全局路径上的最近点为原点，向前采样 200m、向后 30m
- 步长 Δs = 1m，插值得到均匀的参考线点集（x, y, heading, κ）
- 提供左右边界（`get_boundary`），用于路径规划的约束

### 3.3 SimplePredictor（障碍物预测）

文件：`simple_predictor.py`

- 从 `DetectionsTracks` 观测中筛选自车 **40m** 范围内的目标
- 采用**匀速直线运动**模型外推障碍物未来轨迹
- 输出为 `List[Agent]`，每个 agent 保留当前速度供速度规划使用

> **扩展点**：将 `SimplePredictor` 替换为基于学习的预测器，只需继承 `AbstractPredictor` 并实现 `predict()` 即可。

### 3.4 横向规划（五次多项式，`path_planning`）

文件：`my_planner.py → path_planning()`

1. 将自车状态由**笛卡尔坐标系**转换到 **Frenet 坐标系**，得到 $(s_0, l_0, \dot{l}_0, \ddot{l}_0)$
2. 构建六元线性方程组，求解**五次多项式**系数，使路径平滑地从当前偏移 $l_0$ 趋向参考线中心 $l=0$：

$$l(s) = a_0 + a_1 s + a_2 s^2 + a_3 s^3 + a_4 s^4 + a_5 s^5$$

3. 边界约束：将 $l(s)$ 裁剪至参考线左右边界内，最终规划范围最长 **150m**

### 3.5 纵向规划（ST 图 DP，`speed_planning` + `DpDecider`）

文件：`my_planner.py → speed_planning()` / `dp_decider.py`

1. 将障碍物轨迹映射到以规划路径为 s 轴的 **ST 图**，计算每个障碍物的 ST 占用区域
2. 在 ST 图上做**动态规划**，在以下约束下搜索最优速度剖面：
   - 最大加速度：$a_{max} = 3.0\ \text{m/s}^2$
   - 最大减速度：$a_{min} = -4.0\ \text{m/s}^2$
   - 速度上限：$v_{max}$（由 `MyPlanner` 参数指定）
   - 避免与障碍物 ST 区域重叠（代价权重 $w_{obs} = 200$）
   - 趋近参考速度（代价权重 $w_{ref} = 40$）
   - 最小化加速度变化（代价权重 $w_{acc} = 10$）
3. DP 输出 $s(t)$ 序列（时间分辨率 0.5s），再通过有限差分求 $\dot{s}(t)$、$\ddot{s}(t)$

### 3.6 轨迹合成（`merge_path_speed`）

文件：`merge_path_speed.py`

- `transform_path_planning`：将 Frenet 路径点 $(s, l, \dot{l}, \ddot{l})$ 转换回笛卡尔坐标 $(x, y, \text{heading}, \kappa)$，并建立弧长索引
- `cal_dynamic_state`：用三次插值由速度规划结果在任意时刻 $t$ 求 $(s, \dot{s}, \ddot{s})$
- `cal_pose`：根据 $s$ 对路径插值，得到对应 $(x, y, \text{heading})$
- 最终按 `sampling_time` 步长（默认 0.25s）输出完整的 `EgoState` 轨迹序列

---

## 4. 运行方法

### 4.1 环境准备

```bash
conda activate nuplan
cd ~/MyProject/nuplan-devkit
```

确认以下环境变量已设置（或使用默认路径 `~/nuplan/`）：

```bash
export NUPLAN_DATA_ROOT=~/nuplan/dataset   # nuPlan 数据集根目录
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
python nuplan/planning/script/run_planner.py
```

仿真结束后会自动打开 **nuBoard** 进行可视化。

---

## 5. 参数说明

### 5.1 MyPlanner 构造参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `horizon_seconds` | float | — | 规划时域（秒），当前设为 **8.0s** |
| `sampling_time` | float | — | 轨迹采样间隔（秒），当前设为 **0.25s** |
| `max_velocity` | float | 5.0 | 自车最大速度（m/s），当前设为 **17 m/s**（约 61 km/h） |

在 `run_planner.py` 中修改：

```python
planner = MyPlanner(horizon_seconds=8.0, sampling_time=0.25, max_velocity=17)
```

### 5.2 并行运行环境变量

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `NUPLAN_WORKER` | `single_machine_thread_pool` | Worker 类型：`sequential` / `single_machine_thread_pool` / `ray_distributed` |
| `NUPLAN_WORKER_MAX_WORKERS` | CPU 核数 − 1 | 最大并发进程数 |
| `NUPLAN_WORKER_USE_PROCESS_POOL` | `true` | `true` 使用进程池（绕过 GIL），`false` 使用线程池 |
| `NUPLAN_CPUS_PER_SIMULATION` | `1` | 每个仿真任务分配的 CPU 数 |

---

## 6. 场景配置

场景在 `run_planner.py` 的 `DATASET_PARAMS` 列表中配置。

### 6.1 配置规则（重要）

Hydra 的 overrides 列表中**同一个 key 只有最后一次生效**。
因此多个场景的 `log_names` 和 `scenario_tokens` 必须写在**同一条覆盖**里：

```python
# ✅ 正确：所有 token 写在同一个列表里
"scenario_filter.log_names=['log_A', 'log_B', 'log_C']",
"scenario_filter.scenario_tokens=['token_A', 'token_B', 'token_C']",

# ❌ 错误：多次覆盖同一 key，只有最后一行生效
"scenario_filter.log_names=['log_A']",
"scenario_filter.scenario_tokens=['token_A']",
"scenario_filter.log_names=['log_B']",   # 覆盖上一行
"scenario_filter.scenario_tokens=['token_B']",
```

### 6.2 运行全部场景类型（批量测试）

```python
DATASET_PARAMS = [
    'scenario_builder=nuplan_mini',
    'scenario_filter=all_scenarios',
    'scenario_filter.scenario_types=[near_multiple_vehicles, starting_unprotected_cross_turn, starting_left_turn]',
    'scenario_filter.num_scenarios_per_type=5',
]
```

### 6.3 限制场景总数（快速验证）

```python
DATASET_PARAMS = [
    'scenario_builder=nuplan_mini',
    'scenario_filter=one_continuous_log',
    "scenario_filter.log_names=['2021.10.01.19.16.42_veh-28_02011_02410']",
    'scenario_filter.limit_total_scenarios=3',
]
```

---

## 7. 多核并行说明

### 7.1 并行粒度

nuPlan 的并行单位是**场景（scenario）**，每个场景独立运行在一个进程中。  
单个场景内部的仿真循环是串行的，**不能通过并行加速单个场景**。

```
多个 scenario → 分发给多个进程 → 每个进程串行跑一个 scenario
```

因此，**并行效果取决于同时运行的场景数量**。

### 7.2 为什么必须用进程池（而不是线程池）

Python 的 GIL（全局解释器锁）使得多个线程无法真正并行执行 CPU 密集型代码。  
规划计算（矩阵运算、路径搜索）是 CPU 密集型，线程池（`use_process_pool=false`）实际上仍是单核运行。  
必须使用**进程池**（`use_process_pool=true`，即 `ProcessPoolExecutor`）才能真正多核并行。

### 7.3 推荐配置

| 需求 | 推荐配置 |
|------|----------|
| 调试单场景 | `NUPLAN_WORKER=sequential` |
| 多场景并行（推荐） | `NUPLAN_WORKER=single_machine_thread_pool` + `NUPLAN_WORKER_USE_PROCESS_POOL=true` |
| 集群分布式 | `NUPLAN_WORKER=ray_distributed` |

---

## 8. 结果可视化

仿真完成后脚本会自动启动 **nuBoard**（基于 Bokeh 的 Web 可视化工具）。  
仿真结果文件（`.nuboard`）保存在：

```
~/nuplan/exp/exp/simulation/closed_loop_nonreactive_agents/<时间戳>/
```

也可手动启动 nuBoard：

```bash
python nuplan/planning/script/run_nuboard.py \
  scenario_builder=nuplan_mini \
  simulation_path=[<path_to_.nuboard_file>]
```

nuBoard 提供：
- **自车轨迹**与专家轨迹对比
- **ST 图**可视化（需在 `dp_decider.py` 中开启 matplotlib 输出）
- **评测指标**（碰撞率、舒适度、进度等）

---

## 9. 二次开发指南

### 9.1 替换预测模块

继承 `AbstractPredictor`，实现 `predict() -> List[Agent]`，在 `MyPlanner.compute_planner_trajectory` 中替换 `SimplePredictor`。

### 9.2 替换横向规划

修改 `my_planner.py` 中的 `path_planning()` 函数。输出接口保持不变：
```python
return optimal_path_l, optimal_path_dl, optimal_path_ddl, optimal_path_s
```

### 9.3 替换纵向规划

修改 `speed_planning()` 或 `DpDecider.dynamic_programming()`。输出接口保持不变：
```python
return optimal_speed_s, optimal_speed_s_dot, optimal_speed_s_2dot, optimal_speed_t
```

### 9.4 基于采样的规划器（备选思路）

`MyPlanner.planning()` 的注释中提到可以实现**基于采样**的规划器（如 RRT*、Lattice Planner）作为替代方案。只需保证最终输出 `List[EgoState]` 格式不变。

### 9.5 调试 ST 图

`DpDecider.dynamic_programming()` 中已有 `matplotlib` 绘图代码，可将障碍物 ST 轨迹实时可视化。若需保存图片，在该函数末尾添加 `plt.savefig(...)` 即可。
