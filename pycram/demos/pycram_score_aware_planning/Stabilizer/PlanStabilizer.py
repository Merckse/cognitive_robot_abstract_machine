from dataclasses import dataclass

from Evaluate.CompositeEvaluator import CompositeEvaluator
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from common.types import Task, ActionOutcome, TaskStep
from giskardpy.motion_statechart.exceptions import CollisionViolatedError
from giskardpy.motion_statechart.goals.pick_up import ObjectNotReachableException, ObjectDoesntFitException
from helper_methods import get_post_failed_taskstep
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus, PlanTransformationOperator
from pycram.exceptions import MotionDidNotFinish


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
        post_failed : list[TaskStep]= get_post_failed_taskstep(task)
        transformation_operators : list[PlanTransformationOperator] = []


        # Pickup-Motion issues
        if isinstance(exception,ObjectNotReachableException):
            # TODO: implement Bring closer action or reposition action (more difficult)
            transformation_operators.insert(0, PlanTransformationOperator.SUBSTITUTE_WITH_ASSISTANCE)
        elif isinstance(exception, ObjectDoesntFitException):
            # TODO: Implement is graspable function
            transformation_operators.insert(0, PlanTransformationOperator.RETRY_WITH_ASSISTANCE)
        # Any-Motion issues
        elif isinstance(exception, MotionDidNotFinish):
            transformation_operators.insert(0, PlanTransformationOperator.RETRY)
            transformation_operators.insert(0, PlanTransformationOperator.REPLAN)

        # Navigate, Place, Pickup motion issues
        elif isinstance(exception, CollisionViolatedError):
            transformation_operators.insert(0, PlanTransformationOperator.REPLAN)
        else:
            transformation_operators.insert(0, PlanTransformationOperator.RETRY)

        sorted_operators = sorted(transformation_operators,
                                  key=lambda o: self._expected_value(task,post_failed, o, scoretime_monitor),
                                  reverse=True)



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

    def assisted_retry(self, task: Task):
        """
        Rawly retrying the entire task.
        Potentially cutting TaskSteps, like the Navigation in order of minimizing failure.
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

    def _expected_value(self, task:Task, task_list :list[TaskStep], operator : PlanTransformationOperator, scoretime_monitor : ScoreTimeMonitor, composite_evaluator : CompositeEvaluator):
        expected_value : float = 0
        # Evaluate the possible score, that can be gained and how much time has been spent so far.

        # retrieving the time it took and we are already overdue, to see if it is worth it
        overrun_time = scoretime_monitor.task_overtime_seconds(task) # Positive if underperformed, negative if time is left
        task_remaining = scoretime_monitor.time_remaining_seconds_task(task)
        time_remaining = scoretime_monitor.time_remaining_seconds_total()

        # retrieving the score and possibilities of the remaining tasks, to see if they are worth it
        possible_task_score = composite_evaluator.get_score_task_list(task_list=task_list)
        possible_task_possibility = composite_evaluator.get_probability_task_list(task_list=task_list)



        if operator == PlanTransformationOperator.RETRY:
            task_list = task_list
            for t in task_list:
                # TODO: Unsure if this is really practicable
                expected_value += t.action_score * t.action_probability - (1 - t.action_probability) - t.action_time
        elif operator == PlanTransformationOperator.SKIP:
            failed_task =  task_list[0]
            expected_value = failed_task.action_penatly
        elif operator == PlanTransformationOperator.RETRY_WITH_ASSISTANCE:
            failed_task =  task_list[0]
            expected_value = failed_task.action_penatly
        # operator == PlanTransformationOperator.SKIP
        else:
            expected_value = 0





        return expected_value