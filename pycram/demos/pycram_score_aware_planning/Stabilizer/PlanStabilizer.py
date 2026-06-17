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
            transformation_operators.insert(0, PlanTransformationOperator.SKIP)

        for o in transformation_operators:
            self._expected_value(post_failed, o, scoretime_monitor)


    def stabilize_unreachable(self, task: Task):
        # TODO
        pass

    def stabilize_failure_any(self, task: Task):
        # TODO
        pass

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

    def _expected_value(self, task_list :list[TaskStep], operator : PlanTransformationOperator, scoretime_monitor : ScoreTimeMonitor):
        expected_value : float = 0
        if operator == PlanTransformationOperator.RETRY:
            task_list = task_list
            for t in task_list:
                expected_value += t.action_score * t.action_probability
        elif operator == PlanTransformationOperator.SKIP:
            expected_value = 0

        # operator == PlanTransformationOperator.SKIP
        else:
            expected_value = 0





        return expected_value