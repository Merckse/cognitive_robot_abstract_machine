import time
from dataclasses import dataclass
from unittest import skip

from Evaluate.CompositeEvaluator import CompositeEvaluator
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from common.cram_types import Task, Status, TaskStep, ActionType
from common.values import evaluation, lookup_operators
from giskardpy.motion_statechart.exceptions import CollisionViolatedError
from giskardpy.motion_statechart.goals.pick_up import ObjectNotReachableException, ObjectDoesntFitException
from helper_methods import get_remaining_task_steps, navigation_subplan, pickup_subplan, place_subplan
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus, PlanTransformationOperator, Arms
from pycram.exceptions import MotionDidNotFinish
from pycram.language import SequentialNode
from pycram.plans.factories import make_node, sequential
from pycram.plans.failures import ObjectNotGrasped
from pycram.plans.plan_node import PlanNode
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.robot_plans.actions.core.pick_up import PickUpAction
from pycram.robot_plans.actions.core.placing import PlaceAction
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction


@dataclass(kw_only=True)
class PlanStabilizer:
    """
    Notes:
        - Dependent on different method and situation there are different solutions
        - meaning, find out error 
        - then try to fix error
        - then go on
    """

    context : Context = None
    """
    Context is needed for robots state as well as world state. It is updated on method calls
    """


    """
    del-relaxation to check if action is possible at all?
    """
    def stabilize(self, context : Context, plan: SequentialNode, task: Task, exception: Exception, scoretime_monitor : ScoreTimeMonitor):
        """
        Stabilize works by taking the task and the exception and using it to match it to candidate solutions.
        These solutions are hardcoded by the developers. Since there is (at least not expected to be) a unified way of
        solving the evaluation of every stabilizer operator, for every failure.
        :param context: The context of the Beliefstate/world
        :param task: The failed task, with all its TaskSteps
        :param exception: The exception raised by the failed task, with all its TaskSteps
        :param evaluator: The evaluator, that is used to evaluate the tasks
        :param scoretime_monitor: Monitor for the scoring and time spent on the task

        # TOOD: add a constant probability decrease, on actually failed task, so the "hope", becomes less
        """
        self.context = context
        candidate_operators : list[PlanTransformationOperator] = [PlanTransformationOperator.SKIP]

        # sorting out all Nodes, that have already succeeded, expecting, to not doing them again
        for i, child in enumerate(plan.plan.root.children):
            task_step = task.task_steps[i]
            if child.status == TaskStatus.SUCCEEDED:
                task_step.action_outcome = Status.SUCCESS
            else:
                task_step.action_outcome = Status.FAILURE
                task_step.action_failures += 1
                break

        unfinished_node : PlanNode | None= plan.plan.unfinished_node()
        if unfinished_node is None:
            return None

        remaining_plan_nodes : list[PlanNode] = [unfinished_node] + unfinished_node.right_siblings

        remaining_task_steps : list[TaskStep]= get_remaining_task_steps(task)

        # Pickup-Motion issues
        score : list[list[PlanTransformationOperator, float, list[PlanNode], list[TaskStep]]]= []

        # TO CHECK IF CORRECT, DO SOMETHING, THAT NEEDS A MOVETORSOHIGH, BUT RESULTS IN FAILURE; SINCE NOT REACHABLE
        for op in candidate_operators:
            repaired_task_list : list[TaskStep] = self._build_stabilized_task_list(list(remaining_task_steps), op)
            expected_value : float= self._expected_value(task, repaired_task_list, scoretime_monitor)
            repaired_plan : list[PlanNode]= self._build_stabilized_plan(remaining_plan_nodes, repaired_task_list, plan.plan.context)

            score.append([op, expected_value, repaired_plan, repaired_task_list])

        maxed_task_list = max(score, key=lambda row: row[1])

        return maxed_task_list

    def _build_stabilized_task_list(self, task_list: list[TaskStep], operator: PlanTransformationOperator) -> list[TaskStep]:
        if operator == PlanTransformationOperator.SKIP:
            transformed_task_list = self.skip()
        elif operator == PlanTransformationOperator.RETRY:
            transformed_task_list = self.retry(task_list)
        elif operator == PlanTransformationOperator.REPLAN:
            transformed_task_list = self.replan(task_list)
        elif operator == PlanTransformationOperator.REPLAN_WITH_ASSISTANCE:
            transformed_task_list = self.replan_with_asstistance(task_list)
        elif operator == PlanTransformationOperator.SUBSTITUTE_WITH_ASSISTANCE:
            transformed_task_list = self.substitute_with_assistance(task_list)
        else:
            transformed_task_list = task_list
        return transformed_task_list

    def _build_stabilized_plan(self, plan_nodes: list[PlanNode], task_list: list[TaskStep],
                               context: Context) -> list[PlanNode]:
        """
        Reconciles a repaired task_list against the plan nodes that are still pending.

        The repaired task_list is the source of truth: it is the remaining TaskSteps after a
        transformation operator has been applied (e.g. RETRY prepends a PARK, REPLAN prepends
        DETECT + PARK). This walks the task_list in order against the existing plan_nodes and:

          - reuses the existing node when the next pending node already represents the step
            (same ActionType) -- it keeps its already-built ActionDescription / world bindings,
            so nothing already resolved gets thrown away,
          - builds a fresh node from the TaskStep when the step has no counterpart, i.e. it was
            inserted by the operator.

        Steps that resolve to no executable node (e.g. DETECT today) are skipped, mirroring
        generate_plan_task.

        :param plan_nodes: The still-pending plan nodes (failed node onward).
        :param task_list: The repaired remaining TaskSteps to realise as a plan.
        :param context: Context providing the world used to build inserted nodes.
        :return: An ordered list of PlanNodes matching the repaired task_list.
        """
        stabilized_plan: list[PlanNode] = []
        node_cursor: int = 0

        for step in task_list:
            head_node = plan_nodes[node_cursor] if node_cursor < len(plan_nodes) else None
            if head_node is not None and self._node_action_type(head_node) == step.action_type:
                # already represented by an existing, already-built node -> reuse it
                stabilized_plan.append(head_node)
                node_cursor += 1
            else:
                # not represented -> inserted by the repair operator, build it from the TaskStep
                built_node = self._build_node_for_step(step, context)
                if built_node is not None:
                    stabilized_plan.append(built_node)

        return stabilized_plan

    def _node_action_type(self, node: PlanNode) -> ActionType | None:
        """
        Maps an existing plan node back to its ActionType so it can be compared against a TaskStep.
        Only ActionNodes carry an action; anything else returns None and is treated as a mismatch.
        """
        action = getattr(node, "action", None)
        if isinstance(action, NavigateAction):
            return ActionType.NAVIGATE
        if isinstance(action, PickUpAction):
            return ActionType.PICKUP
        if isinstance(action, PlaceAction):
            return ActionType.PLACE
        if isinstance(action, ParkArmsAction):
            return ActionType.PARK
        if isinstance(action, MoveTorsoAction):
            return ActionType.MOVE_TORSO
        return None

    def _build_node_for_step(self, step: TaskStep, context: Context) -> PlanNode | None:
        """
        Builds a single executable plan node from a TaskStep, mirroring generate_plan_task.
        Returns None for steps that have no executable node (e.g. DETECT).
        """
        arm = Arms.LEFT
        match step.action_type:
            case ActionType.NAVIGATE:
                action = navigation_subplan(target_location=step.location, world=context.world)
            case ActionType.PICKUP:
                action = pickup_subplan(object_annotation=step.object_annotations, arm=arm, world=context.world)
            case ActionType.PLACE:
                action = place_subplan(object_annotation=step.object_annotations, arm=arm,
                                       target_location=step.location, world=context.world)
            case ActionType.PARK:
                action = ParkArmsAction(Arms.LEFT)
            case ActionType.DETECT:
                return None
            case _:
                raise NotImplementedError(f"Action type not implemented: {step.action_type}")
        if action is None:
            return None
        return make_node(action)

    def build_recovery_plan(self, task_list: list[TaskStep], context: Context) -> SequentialNode:
        """
        Realises the winning repaired task_list as a fresh, executable plan and returns it.

        This is the "splice + re-perform" step: the original plan's root SequentialNode was
        interrupted on failure and re-performing it would re-run the already-succeeded steps
        (SequentialNode._perform performs *all* children unconditionally). So instead of
        re-attaching to the interrupted root, we build a brand-new sequential containing only
        the repaired remaining steps, with freshly built nodes (no cross-plan node membership
        and no lingering interrupt flag). Performing it continues execution from the point of
        failure onward.

        An empty task_list (e.g. the SKIP operator) yields a childless plan whose perform()
        is a no-op -- the task is simply abandoned, which is the intended SKIP behaviour.

        :param task_list: The winning repaired remaining TaskSteps (maxed_task_list row entry).
        :param context: Context providing the world used to build the nodes.
        :return: A fresh SequentialNode ready to perform.
        """
        recovery_plan = sequential(children=[], context=context)
        for step in task_list:
            node = self._build_node_for_step(step, context)
            if node is not None:
                recovery_plan.add_child(node)
        return recovery_plan

    def replan_with_asstistance(self, task_list : list[TaskStep]):
        # TODO
        return task_list

    def skip(self):
        """
        Skips the current task and continues with a new one.
        Potentially continuing the skipped task, at the last task step, if worth it.
        """
        return []

    def retry(self, task_list: list[TaskStep]):
        """
        Rawly retrying the entire task.
        Potentially cutting TaskSteps, like the Navigation in order of minimizing failure.
        """
        task_list.insert(0, TaskStep(ActionType.PARK)) # defaulting to parked arms, before doing so.
        task_list.insert(0, TaskStep(ActionType.DETECT))
        return task_list

    def substitute_with_assistance(self, task_list: list[TaskStep]):
        # TODO
        return task_list

    def replan(self, task_list: list[TaskStep]):
        task_list.insert(0, TaskStep(ActionType.PARK))
        return task_list


    def addition(self, task_list: list[TaskStep]):
        return task_list

    def _evaluate_operators(self, task:Task,
                            task_list :list[TaskStep],
                            transformation_operator : PlanTransformationOperator,
                            scoretime_monitor : ScoreTimeMonitor):
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

        expected_value : float = self._expected_value(task,
                                                      task_list,
                                                      scoretime_monitor)
        task_list_without_failed : list[TaskStep] = task_list
        task_list_without_failed.pop()
        expected_value_with_assistance: float = self._expected_value(task,
                                                      task_list_without_failed,
                                                      scoretime_monitor)

        # a.) if the penalty is too high, then the assistance will be worth it. b.) if there is no further value with assistance, then its not worth it.
        # The base assumption here being, that the action is always more worth it to continue, if there are points to be gained
        if expected_value < expected_value_with_assistance and expected_value_with_assistance > 0:
            transformation_operator = [PlanTransformationOperator.ASSISTED].append(transformation_operator)
            return transformation_operator

        elif expected_value < 0:
            limited = scoretime_monitor.limited_time()
            transformation_operator = transformation_operator if limited else PlanTransformationOperator.SKIP
            return transformation_operator

        # TODO: evaluate, when what operator is more worth, all assisted operators are lowkey out anyway




        return transformation_operator

    def _expected_value(self, task : Task, task_list: list[TaskStep], scoretime_monitor : ScoreTimeMonitor) -> float:
        """
        Estimates the value of continuing a given task plan from the current point forward.

        The formula is:
            expected_value = expected_reward * time_ratio - expected_penalty

        where:
            expected_reward  = Σ (probability * score)           over all remaining task steps
            expected_penalty = Σ (|penalty| * (1 - probability)) over all remaining task steps

            time_ratio       = clamp(task_time_remaining / expected_time, 0, ∞)
                               Discounts only the achievable REWARD the more overdue the task is:
                               ratio 1 = on schedule, below 1 = running behind (reward shrinks
                               proportionally), 0 = budget exhausted. The penalty is charged in full
                               regardless of time -- so an overdue, penalty-heavy plan converges to
                               -expected_penalty (genuinely negative) instead of being "forgiven"
                               toward 0 as time is wasted.

        A negative expected_value means the plan is more likely to incur penalties than to earn
        points — the stabilizer should prefer SKIP or operator alternatives in that case.

        :param task: The task whose step profiles are evaluated.
        :param task_list: The remaining TaskSteps used to compute the expected execution time.
        :param scoretime_monitor: Provides task-level and global time-budget queries.
        :return: Scalar expected value — positive is profitable, negative means cut losses.
        """
        # Evaluate the possible score, that can be gained and how much time has been spent so far.

        # retrieving the time it took and we are already overdue, to see if it is worth it
        overrun_time : float= scoretime_monitor.task_overtime_seconds(task) # Positive if underperformed, negative if time is left
        task_time_remaining : float = scoretime_monitor.time_remaining_seconds_task(task)
        expected_time : float= scoretime_monitor.time_expected_seconds_task_list(task_list)
        if expected_time == 0:
            expected_time: float = 1 # fallback for skipping

        time_ratio : float = max(0, task_time_remaining / expected_time)

        # retrieving the score and possibilities of the remaining tasks, to see if they are worth it
        # possible_task_score = composite_evaluator.get_score_task_list(task_list=task_list)
        # possible_task_possibility = composite_evaluator.get_probability_task_list(task_list=task_list)

        expected_reward: float = 0
        expected_penalty: float = 0
        for t in task_list:
            profile = evaluation(t.action_type, t.object_annotations, t.location)
            expected_reward += profile.probability * profile.score
            expected_penalty += abs(profile.penalty) * (1 - profile.probability)

        expected_value: float = expected_reward * time_ratio - expected_penalty

        return expected_value

    def _stability_evaluation_distance(self, original_plan : list[TaskStep], repaired_plan : list[TaskStep]):
        """
        The method is intended to compare two plans and tell the stability difference, with facotrs like:
        - distance (look paper)
        - expected time
        - probability
        """

        pass