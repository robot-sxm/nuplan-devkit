import numpy as np
from typing import List, Type, Optional, Tuple
from nuplan.planning.simulation.observation.observation_type import DetectionsTracks, Observation
from nuplan.common.actor_state.ego_state import DynamicCarState, EgoState
from nuplan.planning.simulation.planner.project2.abstract_predictor import AbstractPredictor
from nuplan.common.actor_state.tracked_objects_types import TrackedObjectType
from nuplan.common.actor_state.agent import Agent
from nuplan.common.actor_state.oriented_box import OrientedBox
from nuplan.common.actor_state.state_representation import StateSE2, StateVector2D, TimePoint
from nuplan.common.actor_state.waypoint import Waypoint
from nuplan.planning.simulation.trajectory.predicted_trajectory import PredictedTrajectory


class SimplePredictor(AbstractPredictor):
    def __init__(self, ego_state: EgoState, observations: Observation, duration: float, sample_time: float) -> None:
        self._ego_state = ego_state
        self._observations = observations
        self._duration = duration
        self._sample_time = sample_time
        self._occupancy_map_radius = 40

    def predict(self):
        """Inherited, see superclass."""
        if isinstance(self._observations, DetectionsTracks):
            objects_init = self._observations.tracked_objects.tracked_objects
            objects = [
                object
                for object in objects_init
                if np.linalg.norm(self._ego_state.center.array - object.center.array) < self._occupancy_map_radius
            ]

            # 1. Predicted the Trajectory of each object using constant velocity model
            for object in objects:
                num_steps = int(self._duration / self._sample_time)
                if num_steps <= 0:
                    object.predictions = []
                    continue

                # Start waypoint at current state (t=0)
                waypoints: List[Optional[Waypoint]] = [
                    Waypoint(TimePoint(0), object.box, object.velocity)
                ]

                # Constant velocity extrapolation for future timesteps
                vx, vy = object.velocity.x, object.velocity.y
                heading = object.center.heading
                angular_vel = getattr(object, 'angular_velocity', None) or 0.0

                for step in range(1, num_steps + 1):
                    dt = step * self._sample_time
                    time_us = int(dt * 1e6)
                    future_x = object.center.x + vx * dt
                    future_y = object.center.y + vy * dt
                    future_heading = heading + angular_vel * dt
                    future_pose = StateSE2(future_x, future_y, future_heading)
                    future_box = OrientedBox.from_new_pose(object.box, future_pose)
                    waypoints.append(
                        Waypoint(TimePoint(time_us), future_box, object.velocity)
                    )

                predicted_trajectory = PredictedTrajectory(1.0, waypoints)
                object.predictions = [predicted_trajectory]

            return objects

        else:
            raise ValueError(
                f"SimplePredictor only supports DetectionsTracks. Got {self._observations.detection_type()}")
