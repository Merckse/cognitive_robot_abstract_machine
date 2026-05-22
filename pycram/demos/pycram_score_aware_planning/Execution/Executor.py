from demos.pycram_score_aware_planning.Evaluate.ScoreEvaluator import RobotScorer
from pycram_score_aware_planning.common.types import TaskMode

taskmode = TaskMode.PP
score_evaluator = RobotScorer()
estimation = score_evaluator.estimate(taskmode=taskmode, finished_task_ids=[])
score_evaluator.summary_estimate(estimates=estimation)
