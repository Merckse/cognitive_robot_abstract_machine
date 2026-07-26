from __future__ import annotations
import time
from dataclasses import dataclass, field
from typing import Optional

from common.cram_types import Status, TaskStep, ActionType
from demos.pycram_score_aware_planning.common.cram_types import (
    Task, ScoreEvent,
)
from demos.pycram_score_aware_planning.common.values import (
    evaluation, ASSISTANCE_PENALTY,
)
from helper_methods import get_navigation_poses, nav_time_between
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus


@dataclass(kw_only=True)
class RobotScorer:
    """Estimates the expected score, penalty and duration of a task."""

    _score: int = field(default=0, init=False, repr=False)
    _time: int = field(default=0, init=False, repr=False)
    _start_time_action: float = field(default=time.time(), init=False, repr=False)

    def estimate(
        self, context:Context, task_list: list[Task], finished_task_ids: Optional[list[int]] = [],

    ) -> list[Task]:
        for task in task_list:
            total_score :int= 0
            total_time : int= 0
            total_score_penalized : int = 0
            outcome : TaskStatus | Status = TaskStatus.SUCCEEDED

            if task.id in finished_task_ids:
                continue
            robot = context.robot
            challenge_mode = context.challenge_mode
            # predicted robot position after the steps so far, as a plain (x, y)
            robot_position = robot.root.global_pose.to_position()
            predicted_x, predicted_y = float(robot_position.x), float(robot_position.y)

            for step in task.task_steps:
                base_score: int = 0
                expected_time: int = 0
                penalty: int = 0

                # an already successful step costs and earns nothing
                if step.action_outcome == Status.SUCCESS:
                    pass
                else:
                    step_profile = evaluation(
                    step.action_type, step.object_annotations, step.location
                    )
                    base_score = step_profile.score
                    penalty = step_profile.penalty
                    expected_time = step_profile.time

                    # a step without a location must not be charged travel: the "" -> (0,0)
                    # fallback pose would price it as a round trip to the world origin and drag
                    # the predicted position along with it
                    navigation_pose = (get_navigation_poses(challenge_mode, step.location)
                                       if step.location else None)
                    if navigation_pose is not None:
                        x, y, _ = navigation_pose
                        # travel time from where the previous steps leave the robot
                        expected_time += round(nav_time_between(predicted_x, predicted_y, x, y))
                        predicted_x, predicted_y = x, y

                    if step.uncertain:
                        expected_time += evaluation(ActionType.EXPLORE).time

                    if step.action_assisted:
                        outcome = Status.SUCCESS_WITH_ASSIST
                        base_score : int = 0
                        # assistance is charged, not free: the probability side already reset the
                        # step to p=1, so without a cost here asking for help would always look
                        # better than attempting a risky step
                        penalty = ASSISTANCE_PENALTY

                total_score += base_score
                total_time += expected_time
                total_score_penalized += base_score + penalty

                step.action_score = base_score
                step.action_penalty = penalty
                step.action_time = expected_time
                step.task_score_penalized = base_score+penalty

            task.score = total_score
            task.duration = total_time
            task.score_penalized = total_score_penalized
            task.score_per_seconds = total_score_penalized / total_time
            task.status = TaskStatus.CREATED
        return task_list

    def get_score_task_list(self, task_list : list[TaskStep]) -> int:
        return sum(t.action_score for t in task_list)
