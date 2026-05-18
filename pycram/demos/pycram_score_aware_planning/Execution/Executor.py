from demos.pycram_score_aware_planning.Evaluate.ScoreEvaluator import RobotScorer
from demos.pycram_score_aware_planning.Evaluate.types import TaskMode
from pycram.robot_plans.actions.core.navigation import NavigateAction
from semantic_digital_twin.spatial_types.spatial_types import Pose

taskmode = TaskMode.PP
score_evaluator = RobotScorer()
estimation = score_evaluator.estimate(taskmode=taskmode, finished_task_ids=[])
score_evaluator.summary_estimate(estimates=estimation)


