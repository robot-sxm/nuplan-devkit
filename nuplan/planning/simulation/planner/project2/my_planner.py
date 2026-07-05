import math
import logging
from typing import List, Type, Optional, Tuple

import numpy as np
import numpy.typing as npt

from nuplan.common.actor_state.state_representation import StateVector2D, TimePoint
from nuplan.common.actor_state.vehicle_parameters import get_pacifica_parameters
from nuplan.planning.simulation.controller.motion_model.kinematic_bicycle import KinematicBicycleModel
from nuplan.planning.simulation.observation.observation_type import DetectionsTracks, Observation
from nuplan.planning.simulation.planner.abstract_planner import AbstractPlanner, PlannerInitialization, PlannerInput
from nuplan.planning.simulation.trajectory.abstract_trajectory import AbstractTrajectory
from nuplan.common.maps.abstract_map import AbstractMap
from nuplan.planning.simulation.planner.project2.bfs_router import BFSRouter
from nuplan.planning.simulation.planner.project2.reference_line_provider import ReferenceLineProvider
from nuplan.planning.simulation.planner.project2.simple_predictor import SimplePredictor
from nuplan.planning.simulation.planner.project2.abstract_predictor import AbstractPredictor
from nuplan.planning.simulation.planner.project2.dp_decider import DpDecider
from nuplan.planning.simulation.planner.project2.frame_transform import cartesian2frenet, local2global_vector

from nuplan.planning.simulation.planner.project2.merge_path_speed import transform_path_planning, cal_dynamic_state, cal_pose
from nuplan.common.actor_state.ego_state import DynamicCarState, EgoState
from nuplan.planning.simulation.trajectory.interpolated_trajectory import InterpolatedTrajectory
from nuplan.common.actor_state.state_representation import StateSE2, StateVector2D
from nuplan.common.actor_state.agent import Agent
from nuplan.common.actor_state.tracked_objects import TrackedObject, TrackedObjects

logger = logging.getLogger(__name__)


def path_planning(
        ego_state: EgoState,
        reference_path_provider: ReferenceLineProvider) -> Tuple[List[float], List[float], List[float], List[float]]:
    """
    基于 Frenet 坐标系的路径规划（横向规划）。
    使用五次多项式将自车从当前侧向偏移状态平滑引导至参考线中心（l=0）。

    :param ego_state: 自车当前状态
    :param reference_path_provider: 参考线信息提供者
    :return: (optimal_path_l, optimal_path_dl, optimal_path_ddl, optimal_path_s)
             分别为沿参考线的侧向偏移、偏移一阶导数 dl/ds、偏移二阶导数 ddl/ds²、纵向弧长 s
    """
    # 获取自车后轴中心在全局坐标系的位置
    ego_x  = ego_state.rear_axle.x
    ego_y  = ego_state.rear_axle.y
    ego_heading = ego_state.rear_axle.heading

    # rear_axle_velocity_2d 和 rear_axle_acceleration_2d 定义在车辆坐标系（x=纵向, y=横向），
    # 需要转换到全局坐标系才能用于 cartesian2frenet
    ego_vx, ego_vy = local2global_vector(
        ego_state.dynamic_car_state.rear_axle_velocity_2d.x,
        ego_state.dynamic_car_state.rear_axle_velocity_2d.y,
        ego_heading)
    ego_ax, ego_ay = local2global_vector(
        ego_state.dynamic_car_state.rear_axle_acceleration_2d.x,
        ego_state.dynamic_car_state.rear_axle_acceleration_2d.y,
        ego_heading)

    # 将自车状态从笛卡尔坐标系转换到 Frenet 坐标系，得到 (s, l, dl, ddl)
    s_set, l_set, _, _, dl_set, _, _, ddl_set = cartesian2frenet(
        [ego_x], [ego_y], [ego_vx], [ego_vy], [ego_ax], [ego_ay],
        reference_path_provider._x_of_reference_line,
        reference_path_provider._y_of_reference_line,
        reference_path_provider._heading_of_reference_line,
        reference_path_provider._kappa_of_reference_line,
        reference_path_provider._s_of_reference_line,
    )

    # 当前 Frenet 状态（以参考线绝对弧长坐标为基准）
    start_s   = s_set[0]
    start_l   = l_set[0]
    start_dl  = dl_set[0]
    start_ddl = ddl_set[0]

    # 规划 s 范围：自当前位置起，最多向前 150 m，步长 1 m
    # path_s 保留 150 m 供速度规划使用；横向多项式仅在前 LATERAL_CONV_DIST 米内收敛
    LATERAL_CONV_DIST = 30.0   # [m] 横向回中收敛距离；过长会导致每步修正量极小，车辆长期偏离车道中心
    delta_s    = 1.0
    max_s_ref  = reference_path_provider._s_of_reference_line[-1]
    plan_end_s = min(start_s + 150.0, max_s_ref - delta_s)
    path_s     = list(np.arange(start_s, plan_end_s, delta_s))
    if not path_s:
        path_s = [start_s]

    # 以相对起点的 s_rel 建立五次多项式，避免大数值影响数值稳定性
    # 收敛距离取 LATERAL_CONV_DIST（而非整条路径长度），保证横向偏差在 ~30 m 内完成修正
    end_s_rel = max(min(path_s[-1] - path_s[0], LATERAL_CONV_DIST), delta_s)

    # 五次多项式边界条件：
    #   l(0)         = start_l,   l'(0)         = start_dl,   l''(0)         = start_ddl
    #   l(end_s_rel) = 0,         l'(end_s_rel) = 0,          l''(end_s_rel) = 0
    A = np.array([
        [1, 0,          0,               0,                 0,                0           ],
        [0, 1,          0,               0,                 0,                0           ],
        [0, 0,          2,               0,                 0,                0           ],
        [1, end_s_rel,  end_s_rel**2,    end_s_rel**3,      end_s_rel**4,     end_s_rel**5],
        [0, 1,          2*end_s_rel,     3*end_s_rel**2,    4*end_s_rel**3,   5*end_s_rel**4],
        [0, 0,          2,               6*end_s_rel,       12*end_s_rel**2,  20*end_s_rel**3],
    ])
    b_vec = np.array([start_l, start_dl, start_ddl, 0.0, 0.0, 0.0])

    try:
        coeffs = np.linalg.solve(A, b_vec)
    except np.linalg.LinAlgError:
        # 矩阵奇异时退化为沿参考线中心行驶（l 全为 0）
        coeffs = np.zeros(6)
    a0, a1, a2, a3, a4, a5 = coeffs

    # 获取参考线左右边界约束（lb > 0 为左边界，rb < 0 为右边界）
    lb_set, rb_set = reference_path_provider.get_boundary(path_s)

    optimal_path_l   = []
    optimal_path_dl  = []
    optimal_path_ddl = []
    optimal_path_s   = []

    for idx, s_abs in enumerate(path_s):
        s_rel = s_abs - path_s[0]   # 相对起点的弧长

        # 多项式收敛区间内：正常计算 l、dl、ddl
        # 收敛点之后：保持 l=0（中心线），dl=ddl=0，为速度规划提供干净的直线参考路径
        if s_rel <= end_s_rel:
            l   = a0 + a1*s_rel   + a2*s_rel**2   + a3*s_rel**3    + a4*s_rel**4    + a5*s_rel**5
            dl  =      a1         + 2*a2*s_rel     + 3*a3*s_rel**2  + 4*a4*s_rel**3  + 5*a5*s_rel**4
            ddl =                   2*a2           + 6*a3*s_rel     + 12*a4*s_rel**2  + 20*a5*s_rel**3
        else:
            l, dl, ddl = 0.0, 0.0, 0.0

        # 将 l 限制在道路左右边界内，保证路径不越线
        l = float(np.clip(l, rb_set[idx], lb_set[idx]))

        optimal_path_l.append(l)
        optimal_path_dl.append(dl)
        optimal_path_ddl.append(ddl)
        optimal_path_s.append(s_abs)

    return optimal_path_l, optimal_path_dl, optimal_path_ddl, optimal_path_s


def speed_planning(
        ego_state: EgoState,
        horizon_time: float,
        max_velocity: float,
        objects: list,
        path_idx2s: List[float],
        path_x: List[float],
        path_y: List[float],
        path_heading: List[float],
        path_kappa: List[float]) -> Tuple[List[float], List[float], List[float], List[float]]:
    """
    基于 ST 图动态规划的速度规划（纵向规划）。
    利用 DpDecider 在考虑障碍物约束的情况下，搜索最优纵向速度剖面。

    :param ego_state: 自车当前状态
    :param horizon_time: 规划时域（秒）
    :param max_velocity: 速度上限（m/s）
    :param objects: 预测后的周围障碍物列表（List[Agent]）
    :param path_idx2s: 路径点索引到弧长的映射
    :param path_x, path_y, path_heading, path_kappa: 规划路径的笛卡尔坐标信息
    :return: (optimal_speed_s, optimal_speed_s_dot, optimal_speed_s_2dot, optimal_speed_t)
             分别为各时刻的弧长 s、纵向速度 s_dot、纵向加速度 s_2dot 和时间 t
    """
    dp_step  = 0.5   # ST 图时间分辨率（s）
    max_acc  =  3.0  # 最大加速度（m/s²）
    max_dec  = -4.0  # 最大减速度（m/s²）
    n_steps  = int(horizon_time / dp_step) + 1

    # 以匀速直线运动预测障碍物轨迹，格式为 [[t, x, y], ...]
    obs_trajectory: List[List[List[float]]] = []
    obs_radius: List[float] = []

    for obj in objects:
        # 获取障碍物速度（匀速假设）
        try:
            vx = obj.velocity.x
            vy = obj.velocity.y
        except AttributeError:
            vx, vy = 0.0, 0.0

        traj = []
        for step_idx in range(n_steps):
            t = step_idx * dp_step
            x = obj.center.x + vx * t
            y = obj.center.y + vy * t
            traj.append([t, x, y])
        obs_trajectory.append(traj)

        # 使用障碍物半长作为碰撞安全半径
        try:
            radius = float(obj.box.half_length)
        except AttributeError:
            radius = 2.5
        obs_radius.append(radius)

    # 获取自车动力学参数
    ego_v         = ego_state.dynamic_car_state.rear_axle_velocity_2d.magnitude()
    vehicle_params = ego_state.car_footprint.vehicle_parameters
    ego_half_width = vehicle_params.half_width
    ego_length     = vehicle_params.length

    # 构建 DpDecider 并执行 ST 图动态规划，得到 s 的上下界及 DP 最优解
    dp = DpDecider(
        obs_trajectory=obs_trajectory,
        obs_radius=obs_radius,
        path_idx2s=path_idx2s,
        path_x=path_x,
        path_y=path_y,
        path_heading=path_heading,
        path_kappa=path_kappa,
        total_time=horizon_time,
        step=dp_step,
        max_v=max_velocity,
        ego_half_width=ego_half_width,
        ego_length=ego_length,
        ego_v=ego_v,
        max_acc=max_acc,
        max_dec=max_dec,
    )
    _, _, dp_speed_s, _ = dp.dynamic_programming()

    # 重建与 DpDecider 内部一致的时间轴：[dp_step, 2*dp_step, ..., horizon_time]
    t_list_dp = list(np.arange(dp_step, horizon_time, dp_step))
    t_list_dp.append(float(horizon_time))

    # 在 t=0 处插入初始状态（s=0，即规划起点）
    t_full = [0.0] + t_list_dp
    s_full = [0.0] + list(dp_speed_s)

    # 通过有限差分由 s(t) 计算纵向速度 s_dot(t)
    s_dot_full = [float(ego_v)]
    for i in range(len(t_full) - 1):
        dt = t_full[i + 1] - t_full[i]
        ds = s_full[i + 1] - s_full[i]
        v  = float(np.clip(ds / dt if dt > 1e-6 else 0.0, 0.0, max_velocity))
        s_dot_full.append(v)

    # 通过有限差分由 s_dot(t) 计算纵向加速度 s_2dot(t)
    s_2dot_full = [0.0]
    for i in range(len(s_dot_full) - 1):
        dt = t_full[i + 1] - t_full[i]
        dv = s_dot_full[i + 1] - s_dot_full[i]
        a  = float(np.clip(dv / dt if dt > 1e-6 else 0.0, max_dec, max_acc))
        s_2dot_full.append(a)

    return s_full, s_dot_full, s_2dot_full, t_full


class MyPlanner(AbstractPlanner):
    """
    Planner going straight.
    """

    def __init__(
            self,
            horizon_seconds: float,
            sampling_time: float,
            max_velocity: float = 5.0,
    ):
        """
        Constructor for SimplePlanner.
        :param horizon_seconds: [s] time horizon being run.
        :param sampling_time: [s] sampling timestep.
        :param max_velocity: [m/s] ego max velocity.
        """
        self.horizon_time = TimePoint(int(horizon_seconds * 1e6))
        self.sampling_time = TimePoint(int(sampling_time * 1e6))
        self.max_velocity = max_velocity

        self._router: Optional[BFSRouter] = None
        self._predictor: AbstractPredictor = None
        self._reference_path_provider: Optional[ReferenceLineProvider] = None
        self._routing_complete = False

    def initialize(self, initialization: PlannerInitialization) -> None:
        """Inherited, see superclass."""
        self._router = BFSRouter(initialization.map_api)
        self._router._initialize_route_plan(initialization.route_roadblock_ids)

    def name(self) -> str:
        """Inherited, see superclass."""
        return self.__class__.__name__

    def observation_type(self) -> Type[Observation]:
        """Inherited, see superclass."""
        return DetectionsTracks  # type: ignore

    def compute_planner_trajectory(self, current_input: PlannerInput) -> AbstractTrajectory:
        """
        Implement a trajectory that goes straight.
        Inherited, see superclass.
        """

        # 1. Routing
        ego_state, observations = current_input.history.current_state
        if not self._routing_complete:
            self._router._initialize_ego_path(ego_state, self.max_velocity)
            self._routing_complete = True

        # 2. Generate reference line
        self._reference_path_provider = ReferenceLineProvider(self._router)
        self._reference_path_provider._reference_line_generate(ego_state)

        # 3. Objects prediction
        self._predictor = SimplePredictor(ego_state, observations, self.horizon_time.time_s, self.sampling_time.time_s)
        objects = self._predictor.predict()

        # 4. Planning
        trajectory: List[EgoState] = self.planning(ego_state, self._reference_path_provider, objects,
                                                    self.horizon_time, self.sampling_time, self.max_velocity)

        return InterpolatedTrajectory(trajectory)

    def planning(self,
                 ego_state: EgoState,
                 reference_path_provider: ReferenceLineProvider,
                 object: List[TrackedObjects],
                 horizon_time: TimePoint,
                 sampling_time: TimePoint,
                 max_velocity: float) -> List[EgoState]:
        """
        基于横纵向解耦的轨迹规划主函数。
        横向：五次多项式路径规划（Frenet 坐标系）。
        纵向：ST 图动态规划速度规划。

        :param ego_state: 自车当前状态
        :param reference_path_provider: 参考线信息提供者
        :param object: 障碍物预测轨迹列表
        :param horizon_time: 规划时域
        :param sampling_time: 轨迹采样时间间隔
        :param max_velocity: 速度上限（m/s）
        :return: 规划轨迹（EgoState 列表）
        """

        # 可以实现基于采样的planer或者横纵向解耦的planner，此处给出planner的示例，仅提供实现思路供参考
        # 1. 路径规划（横向）：在 Frenet 坐标系下，利用五次多项式求解侧向偏移 l(s)
        optimal_path_l, optimal_path_dl, optimal_path_ddl, optimal_path_s = path_planning(
            ego_state, reference_path_provider)

        # 2. 将 Frenet 路径规划结果转换到笛卡尔坐标系，得到路径点的 x、y、heading、kappa
        path_idx2s, path_x, path_y, path_heading, path_kappa = transform_path_planning(
            optimal_path_s, optimal_path_l,
            optimal_path_dl, optimal_path_ddl,
            reference_path_provider)

        # 3. 速度规划（纵向）：基于 ST 图动态规划，在考虑障碍物的情况下求解速度剖面
        optimal_speed_s, optimal_speed_s_dot, optimal_speed_s_2dot, optimal_speed_t = speed_planning(
            ego_state, horizon_time.time_s, max_velocity, object,
            path_idx2s, path_x, path_y, path_heading, path_kappa)

        # 4. 合成轨迹：将路径规划与速度规划结果融合，生成 EgoState 序列
        # 以当前自车状态作为轨迹起点
        state = EgoState(
            car_footprint=ego_state.car_footprint,
            dynamic_car_state=DynamicCarState.build_from_rear_axle(
                ego_state.car_footprint.rear_axle_to_center_dist,
                ego_state.dynamic_car_state.rear_axle_velocity_2d,
                ego_state.dynamic_car_state.rear_axle_acceleration_2d,
            ),
            tire_steering_angle=ego_state.tire_steering_angle,
            is_in_auto_mode=True,
            time_point=ego_state.time_point,
        )
        trajectory: List[EgoState] = [state]
        for iter in range(int(horizon_time.time_us / sampling_time.time_us)):
            relative_time = (iter + 1) * sampling_time.time_s
            # 根据 relative_time 和速度规划结果，通过三次插值计算当前时刻的 s、velocity、accelerate
            s, velocity, accelerate = cal_dynamic_state(relative_time, optimal_speed_t, optimal_speed_s,
                                                        optimal_speed_s_dot, optimal_speed_s_2dot)
            # 将 s 限制在路径范围内，防止 interp1d 越界
            s = float(np.clip(s, path_idx2s[0], path_idx2s[-1]))
            # 根据当前时刻的 s 和路径规划结果，通过线性插值计算 x、y、heading、kappa
            x, y, heading, kappa = cal_pose(s, path_idx2s, path_x, path_y, path_heading, path_kappa)
            # 由路径曲率 κ 和轴距 L 通过运动学自行车模型计算前轮转角：δ = arctan(L·κ)
            wheelbase = state.car_footprint.vehicle_parameters.wheel_base
            tire_steering = math.atan(wheelbase * float(kappa))

            # 运动学关系：angular_vel = v * kappa = v * tan(delta) / L
            angular_vel = velocity * float(kappa)

            state = EgoState.build_from_rear_axle(
                rear_axle_pose=StateSE2(x, y, heading),
                rear_axle_velocity_2d=StateVector2D(velocity, 0),
                rear_axle_acceleration_2d=StateVector2D(accelerate, 0),
                tire_steering_angle=tire_steering,
                time_point=state.time_point + sampling_time,
                vehicle_parameters=state.car_footprint.vehicle_parameters,
                is_in_auto_mode=True,
                angular_vel=angular_vel,
                angular_accel=0,
            )

            trajectory.append(state)

        return trajectory