# Useful imports
import os
import sys
import time
from pathlib import Path
import tempfile

# Add pluto project to Python path (before other imports that may depend on it)
sys.path.insert(0, '/root/pluto')

import hydra
from nuplan.planning.simulation.planner.project2.my_planner import MyPlanner
from nuplan.planning.simulation.planner.simple_planner import SimplePlanner
from nuplan.planning.simulation.planner.idm_planner import IDMPlanner
from nuplan.planning.simulation.planner.ml_planner.ml_planner import MLPlanner
from tutorials.utils.tutorial_utils import construct_simulation_hydra_paths

# Location of paths with all simulation configs
# Use the absolute path of this script's directory so that hydra searchpath URIs
# (file://...) are always absolute and valid regardless of CWD.
BASE_CONFIG_PATH = os.path.dirname(os.path.abspath(__file__))
simulation_hydra_paths = construct_simulation_hydra_paths(BASE_CONFIG_PATH)

# Create a temporary directory to store the simulation artifacts
# SAVE_DIR = tempfile.mkdtemp()
current_time = time.strftime('%Y-%m-%d_%H-%M-%S', time.localtime())
# SAVE_DIR = Path(tempfile.gettempdir()) / 'tutorial_nuplan_framework' / current_time
SAVE_DIR = Path(os.environ['HOME']) / 'nuplan/exp/exp/simulation/closed_loop_nonreactive_agents' / current_time

# Select simulation parameters
EGO_CONTROLLER = 'perfect_tracking_controller'  # [log_play_back_controller, perfect_tracking_controller]
OBSERVATION = 'box_observation'  # [box_observation, idm_agents_observation, lidar_pc_observation]

# Worker settings for parallel simulation execution.
# You can override them at runtime, e.g.:
# NUPLAN_WORKER=single_machine_thread_pool NUPLAN_WORKER_MAX_WORKERS=8 python .../run_planner.py
WORKER = os.getenv('NUPLAN_WORKER', 'single_machine_thread_pool')
WORKER_MAX_WORKERS = int(os.getenv('NUPLAN_WORKER_MAX_WORKERS', str(max(1, (os.cpu_count() or 1) - 1))))
# 使用线程池而非进程池，避免 CUDA fork 问题
WORKER_USE_PROCESS_POOL = os.getenv('NUPLAN_WORKER_USE_PROCESS_POOL', 'false').lower() == 'true'
CPUS_PER_SIMULATION = int(os.getenv('NUPLAN_CPUS_PER_SIMULATION', '1'))

DATASET_PARAMS = [
    # 'scenario_builder=nuplan_mini',  # use nuplan mini database (2.5h of 8 autolabeled logs in Las Vegas)
    # 'scenario_filter=one_continuous_log',  # simulate only one log
    # "scenario_filter.log_names=['2021.07.16.20.45.29_veh-35_01095_01486']",
    # 'scenario_filter.limit_total_scenarios=2',  # use 2 total scenarios
    # 'scenario_filter=all_scenarios',  # initially select all scenarios in the database
    # 'scenario_filter.scenario_types=[near_multiple_vehicles, on_pickup_dropoff, starting_unprotected_cross_turn, high_magnitude_jerk]',  # select scenario types
    # 'scenario_filter.num_scenarios_per_type=2',  # use 10 scenarios per scenario type
    
    'scenario_builder=nuplan_mini',  # use nuplan mini database (2.5h of 8 autolabeled logs in Las Vegas)
    'scenario_filter=one_continuous_log',  # simulate only one log

    "scenario_filter.log_names=['2021.06.23.15.56.12_veh-16_00839_01285']", # changing_lane
    "scenario_filter.scenario_tokens=['6d1811320c635e82']",

    # "scenario_filter.log_names=['2021.07.09.20.59.12_veh-38_01208_01692']",  # near_multiple_vehicles
    # "scenario_filter.scenario_tokens=['4f3cac1a0bcb5b89']",

    # "scenario_filter.log_names=['2021.08.17.18.54.02_veh-45_00665_01065']",  # following_lane_without_lead
    # "scenario_filter.scenario_tokens=['d5eddf5327a55d5c']",

    # "scenario_filter.log_names=['2021.07.16.00.51.05_veh-17_01352_01901']",  # following_lane_without_lead
    # "scenario_filter.scenario_tokens=['628313fbe48550ac']",

    # "scenario_filter.log_names=['2021.08.17.16.57.11_veh-08_01200_01636']",  # following_lane_without_lead
    # "scenario_filter.scenario_tokens=['6ec306ff06e35a17']",

    # "scenario_filter.log_names=['2021.05.12.22.28.35_veh-35_00620_01164']",  # following_lane_with_slow_lead
    # "scenario_filter.scenario_tokens=['1971267bb0135ef5']",

    # "scenario_filter.log_names=['2021.06.07.12.54.00_veh-35_01843_02314']", # following_lane_with_slow_lead
    # "scenario_filter.scenario_tokens=['4f612f81037e5cf7']",

    # "scenario_filter.log_names=['2021.08.17.18.54.02_veh-45_00665_01065']",  # starting_unprotected_cross_turn
    # "scenario_filter.scenario_tokens=['7ff1de6b23035dc8']",

    # "scenario_filter.log_names=['2021.09.16.15.12.03_veh-42_01037_01434']",  # starting_left_turn
    # "scenario_filter.scenario_tokens=['3ec8944f0e5a5637']",

    # "scenario_filter.log_names=['2021.10.05.07.10.04_veh-52_01442_01802']",   # starting_unprotected_cross_turn
    # "scenario_filter.scenario_tokens=['40cef783435759d3']",

    # "scenario_filter.log_names=['2021.10.06.17.43.07_veh-28_00508_00877']",  # starting_unprotected_cross_turn
    # "scenario_filter.scenario_tokens=['9a48aa6a1ebd5027']",

    # All 3 scenarios listed in a SINGLE override each — Hydra keeps only the last
    # occurrence of the same key, so we must pass all values as one list.
    # "scenario_filter.log_names=['2021.08.17.16.57.11_veh-08_01200_01636', '2021.06.14.16.48.02_veh-12_04978_05337', '2021.10.01.19.16.42_veh-28_02011_02410']",
    # "scenario_filter.scenario_tokens=['6088036cf6d15e1c', '143076200fec5eb1', 'be051cec36545b3d']",

    # "scenario_filter.log_names=['2021.08.17.17.17.01_veh-45_02314_02798']",  # starting_left_turn
    # "scenario_filter.scenario_tokens=['d1352bb76f41547b']",

    # "scenario_filter.log_names=['2021.08.17.16.57.11_veh-08_01200_01636']",  # starting_left_turn
    # "scenario_filter.scenario_tokens=['a186ea974b495ce2']",

    # "scenario_filter.log_names=['2021.08.17.17.17.01_veh-45_02314_02798']",   # starting_left_turn
    # "scenario_filter.scenario_tokens=['6663ee66bfd85604']",
]

# Initialize configuration management system
hydra.core.global_hydra.GlobalHydra.instance().clear()  # reinitialize hydra if already initialized
hydra.initialize_config_dir(config_dir=simulation_hydra_paths.config_path)

# Compose the configuration
overrides = [
    f'group={SAVE_DIR}',
    f'experiment_name=planner_tutorial',
    f'job_name=planner_tutorial',
    'experiment=${experiment_name}/${job_name}',
    f'worker={WORKER}',
    f'number_of_cpus_allocated_per_simulation={CPUS_PER_SIMULATION}',
    f'ego_controller={EGO_CONTROLLER}',
    f'observation={OBSERVATION}',
    f'hydra.searchpath=[{simulation_hydra_paths.common_dir}, {simulation_hydra_paths.experiment_dir}]',
    # 'output_dir=${group}/${experiment}/',
    'output_dir=${group}/',
    *DATASET_PARAMS,
]

if WORKER == 'single_machine_thread_pool':
    overrides.extend(
        [
            f'worker.max_workers={WORKER_MAX_WORKERS}',
            f'worker.use_process_pool={str(WORKER_USE_PROCESS_POOL).lower()}',
        ]
    )

cfg = hydra.compose(config_name=simulation_hydra_paths.config_name, overrides=overrides)


from nuplan.planning.script.run_simulation import run_simulation as main_simulation

# planner = SimplePlanner(horizon_seconds=10.0, sampling_time=0.25, acceleration=[0.0, 0.0])
# planner = IDMPlanner(target_velocity=10.0, min_gap_to_lead_agent=1.0, headway_time=1.5, accel_max=1.0, decel_max=3.0, planned_trajectory_samples=16, planned_trajectory_sample_interval=0.5, occupancy_map_radius=40)
# planner = MyPlanner(horizon_seconds=8.0, sampling_time=0.25, max_velocity=17)


# ============================================================
# Planner 1: RasterModel (ResNet50 backbone)
# ============================================================
from nuplan.planning.training.modeling.models.raster_model import RasterModel
from nuplan.planning.training.modeling.lightning_module_wrapper import LightningModuleWrapper
from nuplan.planning.simulation.trajectory.trajectory_sampling import TrajectorySampling
from nuplan.planning.training.preprocessing.feature_builders.raster_feature_builder import RasterFeatureBuilder
from nuplan.planning.training.preprocessing.target_builders.ego_trajectory_target_builder import EgoTrajectoryTargetBuilder

# 构建与训练时一致的 RasterModel 架构
future_trajectory_sampling = TrajectorySampling(num_poses=16, time_horizon=8.0)

raster_model = RasterModel(
    model_name='resnet50',
    pretrained=True,
    num_input_channels=4,
    num_features_per_pose=3,
    future_trajectory_sampling=future_trajectory_sampling,
    feature_builders=[
        RasterFeatureBuilder(
            map_features={'LANE': 1.0, 'INTERSECTION': 1.0, 'STOP_LINE': 0.5, 'CROSSWALK': 0.5},
            num_input_channels=4,
            target_width=224,
            target_height=224,
            target_pixel_size=0.5,
            ego_width=2.297,
            ego_front_length=4.049,
            ego_rear_length=1.127,
            ego_longitudinal_offset=0.0,
            baseline_path_thickness=1,
        )
    ],
    target_builders=[EgoTrajectoryTargetBuilder(future_trajectory_sampling=future_trajectory_sampling)],
)

# 从 checkpoint 加载训练好的权重
checkpoint_path = '/tmp/tutorial_nuplan_framework/training_raster_experiment/train_default_raster/2026.07.07.15.23.40/best_model/epoch=19-step=1299.ckpt'
raster_ml_model = LightningModuleWrapper.load_from_checkpoint(checkpoint_path, model=raster_model).model

raster_planner = MLPlanner(model=raster_ml_model)

# ============================================================
# Planner 2: Pluto (Transformer-based, 1M auxiliary + CIL)
# ============================================================
from src.models.pluto.pluto_model import PlanningModel
from src.feature_builders.pluto_feature_builder import PlutoFeatureBuilder
from src.planners.pluto_planner import PlutoPlanner

# Pluto feature builder（参数与 pluto_planner.yaml 保持一致）
pluto_feature_builder = PlutoFeatureBuilder(
    radius=120,
    history_horizon=2,
    future_horizon=8,
    sample_interval=0.1,
    max_agents=48,
    build_reference_line=True,
)

# Pluto 模型架构（与 checkpoint 及 pluto_planner.yaml 保持一致）
pluto_model = PlanningModel(
    dim=128,
    state_channel=6,
    polygon_channel=6,
    history_channel=9,
    history_steps=21,
    future_steps=80,
    encoder_depth=4,
    decoder_depth=4,
    drop_path=0.2,
    dropout=0.1,
    num_heads=4,
    num_modes=12,
    use_ego_history=False,
    state_attn_encoder=True,
    state_dropout=0.75,
    use_hidden_proj=True,
    cat_x=True,
    ref_free_traj=True,
    feature_builder=pluto_feature_builder,
)

# PlutoPlanner 的 __init__ 需要 scenario 参数，但 pre_built_planners 路径下框架不会传入。
# 使用 SimpleNamespace（标准库类型，pickle 可跨模块反序列化）而非局部类。
from types import SimpleNamespace

pluto_planner = PlutoPlanner(
    planner=pluto_model,
    scenario=SimpleNamespace(scenario_type="unknown", log_name="unknown", token="unknown"),
    planner_ckpt='/root/pluto/checkpoints/pluto_1M_aux_cil.ckpt',
    use_gpu=True,
    render=False,
)

# ============================================================
# 同时运行两个 planner 进行对比
# ============================================================
planners = [raster_planner, pluto_planner]

# Run the simulation loop (real-time visualization not yet supported, see next section for visualization)
main_simulation(cfg, planners)

# Get nuBoard simulation file for visualization later on
simulation_file = [str(file) for file in Path(cfg.output_dir).iterdir() if file.is_file() and file.suffix == '.nuboard']

from tutorials.utils.tutorial_utils import construct_nuboard_hydra_paths

# Location of paths with all nuBoard configs
nuboard_hydra_paths = construct_nuboard_hydra_paths(BASE_CONFIG_PATH)

# Initialize configuration management system
hydra.core.global_hydra.GlobalHydra.instance().clear()  # reinitialize hydra if already initialized
hydra.initialize_config_dir(config_dir=nuboard_hydra_paths.config_path)

# Compose the configuration
cfg = hydra.compose(config_name=nuboard_hydra_paths.config_name, overrides=[
    'scenario_builder=nuplan_mini',  # set the database (same as simulation) used to fetch data for visualization
    f'simulation_path={simulation_file}',  # nuboard file path, if left empty the user can open the file inside nuBoard
    f'hydra.searchpath=[{nuboard_hydra_paths.common_dir}, {nuboard_hydra_paths.experiment_dir}]',
])


# from nuplan.planning.script.run_nuboard import main as main_nuboard

# # Run nuBoard
# main_nuboard(cfg)
