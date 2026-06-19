from dataclasses import dataclass

from Evaluate.CompositeEvaluator import CompositeEvaluator
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from common.types import Task, ActionOutcome, TaskStep
from common.values import evaluation
from giskardpy.motion_statechart.exceptions import CollisionViolatedError
from giskardpy.motion_statechart.goals.pick_up import ObjectNotReachableException, ObjectDoesntFitException
from helper_methods import get_remaining_task_steps
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus, PlanTransformationOperator
from pycram.exceptions import MotionDidNotFinish
from pycram.plans.failures import ObjectNotGrasped


@dataclass(kw_only=True)
class PlanStabilizer:
    """
    Notes:
        - Dependent on different method and situation there are different solutions
        - meaning, find out error 
        - then try to fix error
        - then go on
    """
    def stabilize(self, context: Context, task: Task, exception: Exception, evaluator : CompositeEvaluator, scoretime_monitor : ScoreTimeMonitor):
        """
        Stabilize works by taking the task and the exception and using it to match it to candidate solutions.
        These solutions are hardcoded by the developers. Since there is (at least not expected to be) a unified way of
        solving the evaluation of every stabilizer operator, for every failure.
        :param context: The context of the Beliefstate/world
        :param task: The failed task, with all its TaskSteps
        :param exception: The exception raised by the failed task, with all its TaskSteps
        :param evaluator: The evaluator, that is used to evaluate the tasks
        :param scoretime_monitor: Monitor for the scoring and time spent on the task

        TODO: if the evaluator turns out to be useless or overkill, just throw it
        """
        remaining_task_steps : list[TaskStep]= get_remaining_task_steps(task)
        transformation_operators : list[PlanTransformationOperator] = []

        # Pickup-Motion issues
        if isinstance(exception,ObjectNotReachableException):
            # TODO: implement Bring closer action or reposition action (more difficult)
            transformation_operators.insert(0, PlanTransformationOperator.SUBSTITUTE_WITH_ASSISTANCE)
        elif isinstance(exception, ObjectDoesntFitException):
            # TODO: Implement is graspable function
            transformation_operators.insert(0, PlanTransformationOperator.RETRY_WITH_ASSISTANCE)
        # Any-Motion issues
        elif isinstance(exception, MotionDidNotFinish) or isinstance(ObjectNotGrasped):
            transformation_operators.insert(0, PlanTransformationOperator.RETRY)
            transformation_operators.insert(0, PlanTransformationOperator.REPLAN)
            transformation_operators.insert(0, PlanTransformationOperator.SKIP)
        # Navigate, Place, Pickup motion issues
        elif isinstance(exception, CollisionViolatedError):
            transformation_operators.insert(0, PlanTransformationOperator.REPLAN)
        else:
            transformation_operators.insert(0, PlanTransformationOperator.REPLAN)
            transformation_operators.insert(0, PlanTransformationOperator.SKIP)

        sorted_operators = sorted(transformation_operators,
                                  key=lambda o: self._evaluate_operators(task,remaining_task_steps, scoretime_monitor),
                                  reverse=True)



    def _build_stabilized_task_list(self, task_list: list[TaskStep], operator_list: list[PlanTransformationOperator]) -> list[TaskStep]:




    def skip(self, task: Task):
        """
        Skips the current task and continues with a new one.
        Potentially continuing the skipped task, at the last task step, if worth it.
        """
        task.status = TaskStatus.SKIPPED

    def retry(self, task: Task):
        """
        Rawly retrying the entire task.
        Potentially cutting TaskSteps, like the Navigation in order of minimizing failure.
        """
        # TODO
        pass

    def assisted_replan(self, task: Task):
        """
        Rawly replaning the entire task.
        But with assistance, meaning a object gets moved, so the action works.

        """
        # TODO
        pass

    def substitute(self, task: Task):
        """
        The substitution, by requesting assistance.
        If the system expects that the task still earns enough points or that the rest of the plan is still
        executeable, then it can be assisted and continue its´ task. This is mostly useful, since execution time of new plans and navigation times
        can strongly vary.
        """
        # TODO
        pass


    def assisted(self, task: Task):
        """
        The substitution, by requesting assistance.
        If the system expects that the task still earns enough points or that the rest of the plan is still
        executeable, then it can be assisted and continue its´ task. This is mostly useful, since execution time of new plans and navigation times
        can strongly vary.
        """
        # TODO
        pass


    def replan(self, task: Task):
        """
        The substitution, by requesting assistance.
        If the system expects that the task still earns enough points or that the rest of the plan is still
        executeable, then it can be assisted and continue its´ task. This is mostly useful, since execution time of new plans and navigation times
        can strongly vary.
        """
        # TODO
        pass

    def _evaluate_operators(self, task:Task, task_list :list[TaskStep], transformation_operator : PlanTransformationOperator, scoretime_monitor : ScoreTimeMonitor, composite_evaluator : CompositeEvaluator):
        """
        Scores a single transformation operator against the remaining task steps and returns its expected value.

        The base expected value of the remaining plan is computed first via _expected_value. Each operator
        then adjusts that value according to its specific cost/benefit model:

        - RETRY: adds the probability-weighted score gain of each remaining step, minus the time cost and the
          expected loss from potential failure (1 - probability). Rewards operators that keep high-value steps
          in reach, penalises time-expensive retries.
        - SKIP: the value collapses to the penalty incurred by skipping the failed step. Use when the failed
          step is low-value and the penalty is small enough to justify moving on.
        - RETRY_WITH_ASSISTANCE: same as SKIP for now — the penalty of the failed step. The distinction from
          SKIP is semantic: assistance is requested, so the step may still be completed, but the current
          formula does not yet model the assistance gain.
        - Fallback (e.g. SUBSTITUTE, REPLAN): returns 0, treated as a neutral baseline until those branches
          are formalised.

        A negative final value signals that continuing the plan (under this operator) costs more than it earns,
        which can be used upstream to trigger an early-abort or an alternative strategy.

        :param task: The parent task whose overall time budget is tracked.
        :param task_list: The remaining TaskSteps to be executed (post-failure slice).
        :param transformation_operator: The candidate PlanTransformationOperator being evaluated.
        :param scoretime_monitor: Provides time-budget queries used by _expected_value.
        :param composite_evaluator: Reserved for future probability/score overrides (not yet used here).
        :return: Expected value (float) — higher is better. Negative means the operator is not worth applying.
        """


        transformation_operator : PlanTransformationOperator = transformation_operator
        expected_value : float = self._expected_value(task,
                                                      task_list,
                                                      scoretime_monitor)
        task_list_without_failed : list[TaskStep] = task_list
        task_list_without_failed.pop()
        expected_value_with_assistance: float = self._expected_value(task,
                                                      task_list_without_failed,
                                                      scoretime_monitor)

        # Check the expected score and if there is enough time, to still execute a task with average legnth.
        if expected_value < expected_value_with_assistance and expected_value_with_assistance > 0:
            transformation_operator = PlanTransformationOperator.ASSISTED
        if expected_value < 0:
            limited = scoretime_monitor.limited_time()
            transformation_operator = transformation_operator if limited else PlanTransformationOperator.SKIP


        return transformation_operator

    def _expected_value(self, task : Task, task_list: list[TaskStep], scoretime_monitor : ScoreTimeMonitor) -> float:
        """
        Estimates the value of continuing a given task plan from the current point forward.

        The formula is:
            expected_value = expected_score * time_ratio

        where:
            expected_score = Σ (probability * score) - (|penalty| * (1 - probability))
                             summed over all remaining task steps. This is the classic
                             probability-weighted reward minus the expected penalty for failure.

            time_ratio     = clamp(task_time_remaining / expected_time, 0, ∞)
                             Scales the score down the more overdue the task is. A ratio of 1
                             means the task is exactly on schedule; below 1 means we are running
                             behind and the achievable score shrinks proportionally; 0 means the
                             task budget is fully exhausted and no further value can be realised.

        A negative expected_value means the plan is more likely to incur penalties than to earn
        points — the stabilizer should prefer SKIP or operator alternatives in that case.

        :param task: The task whose step profiles are evaluated.
        :param task_list: The remaining TaskSteps used to compute the expected execution time.
        :param scoretime_monitor: Provides task-level and global time-budget queries.
        :return: Scalar expected value — positive is profitable, negative means cut losses.
        """
        expected_score: float = 0
        # Evaluate the possible score, that can be gained and how much time has been spent so far.

        # retrieving the time it took and we are already overdue, to see if it is worth it
        overrun_time : float= scoretime_monitor.task_overtime_seconds(task) # Positive if underperformed, negative if time is left
        task_time_remaining : float = scoretime_monitor.time_remaining_seconds_task(task)
        expected_time : float= scoretime_monitor.time_expected_seconds_task_list(task_list)

        time_ratio : float = max(0, task_time_remaining / expected_time)

        # retrieving the score and possibilities of the remaining tasks, to see if they are worth it
        # possible_task_score = composite_evaluator.get_score_task_list(task_list=task_list)
        # possible_task_possibility = composite_evaluator.get_probability_task_list(task_list=task_list)

        for t in task.task_steps:
            profile = evaluation(t.action_type, t.object_annotations, t.location)
            expected_score += ((profile.probability * profile.score) -
                               (abs(profile.penalty) * (1 - profile.probability)))
        expected_value : float= expected_score * time_ratio # expected_Score multiplied, by

        return expected_value

    def _stability_evaluation(self):
        """
        The method is intended to compare two plans and tell the stability difference, with facotrs like:
        - distance (look paper)
        - expected time
        - probability
        """
        #TODO
        pass