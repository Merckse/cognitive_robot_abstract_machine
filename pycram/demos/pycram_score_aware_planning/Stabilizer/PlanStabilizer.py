import time
from copy import deepcopy
from dataclasses import dataclass
from typing import Optional
from unittest import skip


from Evaluate.CompositeEvaluator import CompositeEvaluator
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from common.cram_types import Task, Status, TaskStep, ActionType
from common.values import evaluation, lookup_operators, CANDIDATE_OPERATORS
from giskardpy.motion_statechart.exceptions import CollisionViolatedError
from giskardpy.motion_statechart.goals.pick_up import ObjectNotReachableException, ObjectDoesntFitException
from helper_methods import get_remaining_task_steps, navigation_subplan, pickup_subplan, place_subplan, \
    NAVIGATION_POSES, at_location, throw_away_subplan, open_subplan
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus, PlanTransformationOperator, Arms
from pycram.exceptions import MotionDidNotFinish
from pycram.language import SequentialNode
from pycram.plans.factories import make_node, sequential
from pycram.plans.failures import ObjectNotGrasped
from pycram.plans.plan_node import PlanNode
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.robot_plans.actions.core.pick_up import PickUpAction
from pycram.robot_plans.actions.core.placing import PlaceAction, PlaceInTrashAction
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
    def stabilize(self,
                  context : Context,
                  plan: SequentialNode,
                  task: Task,
                  exception: Exception,
                  scoretime_monitor : ScoreTimeMonitor,
                  exec_steps: list[TaskStep],
                  remaining_candidate_operators : list[PlanTransformationOperator] = [],):
        """
        Stabilize works by taking the task and the exception and using it to match it to candidate solutions.
        These solutions are hardcoded by the developers. Since there is (at least not expected to be) a unified way of
        solving the evaluation of every stabilizer operator, for every failure.
        :param context: The context of the Beliefstate/world
        :param task: The failed task, with all its TaskSteps
        :param exception: The exception raised by the failed task, with all its TaskSteps
        :param evaluator: The evaluator, that is used to evaluate the tasks
        :param scoretime_monitor: Monitor for the scoring and time spent on the task
        :param exec_steps: The task-step list actually used to build `plan` (may contain extra,
            synthetic NAVIGATE steps not present in task.task_steps) -- aligns 1:1 with plan.plan.root.children.
        :param operator_list: List of PlanTransformationOperators

        # TOOD: add a constant probability decrease, on actually failed task, so the "hope", becomes less
        """
        self.context = context
        candidate_operators = remaining_candidate_operators if remaining_candidate_operators else lookup_operators(exception=exception)

        # sorting out all Nodes, that have already succeeded, expecting, to not doing them again
        for i, child in enumerate(plan.plan.root.children):
            task_step = exec_steps[i]
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
        score : list[list[PlanTransformationOperator, float, PlanNode, list[TaskStep]]]= []




        # TO CHECK IF CORRECT, DO SOMETHING, THAT NEEDS A MOVETORSOHIGH, BUT RESULTS IN FAILURE; SINCE NOT REACHABLE
        for op in candidate_operators:
            repaired_task_list : list[TaskStep] = self._build_stabilized_task_list(list(remaining_task_steps), op, context)
            expected_value : float= self._expected_value(task, repaired_task_list, scoretime_monitor)
            repaired_plan : PlanNode= self._build_stabilized_plan(remaining_plan_nodes, repaired_task_list, plan.plan.context)

            score.append([op, expected_value, repaired_plan, repaired_task_list])

        maxed_task_list = max(score, key=lambda row: row[1])

        return maxed_task_list

    def _build_stabilized_task_list(self, task_list: list[TaskStep], operator: PlanTransformationOperator,context: Context) -> list[TaskStep]:
        if operator == PlanTransformationOperator.SKIP:
            transformed_task_list = self.skip(task_list)
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

        exec_steps = []
        temp_robot = deepcopy(self.context.robot)
        for ts in transformed_task_list:
            if ts.location != "" and not at_location(context=context, location=ts.location, robot=temp_robot):
                # location known -> place the robot at the approach pose so distance is measured from there
                if ts.location in NAVIGATION_POSES:
                    x, y, _ = NAVIGATION_POSES[ts.location]
                    temp_robot.root.global_pose.x = x
                    temp_robot.root.global_pose.y = y
                exec_steps.append(TaskStep(action_type=ActionType.NAVIGATE, location=ts.location))
            exec_steps.append(ts)
        return exec_steps

    def _build_stabilized_plan(self, plan_nodes: list[PlanNode], task_list: list[TaskStep],
                               context: Context) -> PlanNode:
        stabilized_plan_node: list[PlanNode] = []
        node_pointer: int = 0


        for step in task_list:
            head_node = plan_nodes[node_pointer] if node_pointer < len(plan_nodes) else None
            if head_node is not None and self._node_action_type(head_node) == step.action_type:
                if step.action_assisted:
                    if head_node.action.assisted != step.action_assisted:
                        built_node = self._build_node_for_step(step, context)
                        if built_node is not None:
                            stabilized_plan_node.append(built_node)
                            node_pointer += 1
                            continue

                stabilized_plan_node.append(head_node)
                node_pointer += 1
            else:
                built_node = self._build_node_for_step(step, context)
                if built_node is not None:
                    stabilized_plan_node.append(built_node)

        stabilized_plan = sequential(children=stabilized_plan_node, context=context)
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
        challenge_mode = context.challenge_mode
        arm = Arms.LEFT
        world = context.world
        match step.action_type:
            case ActionType.NAVIGATE:
                action = navigation_subplan(challenge_mode=challenge_mode, target_location=step.location, world=world)
            case ActionType.PICKUP:
                action = pickup_subplan(object_annotation=step.object_annotations, arm=arm,
                                        world=world, assisted=step.action_assisted)
            case ActionType.PLACE:
                action = place_subplan(challenge_mode=challenge_mode, object_annotation=step.object_annotations, arm=arm,
                                       target_location=step.location, world=world, assisted=step.action_assisted, robot=context.robot)
            case ActionType.PARK:
                action = ParkArmsAction(Arms.LEFT)
            case ActionType.DETECT:
                return None
            case ActionType.THROW_AWAY:
                action =  throw_away_subplan(object_annotation=step.object_annotations, arm=arm, world=world, assisted=step.action_assisted)
            case ActionType.OPEN:
                action = open_subplan(object_annotation=step.object_annotations, arm=arm,
                               world=world, assisted=step.action_assisted)
            case _:
                raise NotImplementedError(f"Action type not implemented: {step.action_type}")
        if action is None:
            return None
        return make_node(action)

    def build_recovery_plan(self, task_list: list[TaskStep], context: Context) -> SequentialNode:
        recovery_plan = sequential(children=[], context=context)
        for step in task_list:
            node = self._build_node_for_step(step, context)
            if node is not None:
                recovery_plan.add_child(node)
        return recovery_plan

    def replan_with_asstistance(self, task_list : list[TaskStep]) -> list[TaskStep]:
        return task_list

    def skip(self,task_list: list[TaskStep]):
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
        tmp_task_list : list[TaskStep] = task_list
        tmp_task_list.insert(0, TaskStep(ActionType.PARK)) # defaulting to parked arms, before doing so.
        tmp_task_list.insert(0, TaskStep(ActionType.DETECT))
        return tmp_task_list

    def substitute_with_assistance(self, task_list: list[TaskStep]):
        # Open, Pick, Place, Close
        tmp_task_list : list[TaskStep] = task_list
        tmp_task_list[0].action_assisted = True
        return tmp_task_list

    def replan(self, task_list: list[TaskStep]):
        tmp_task_list : list[TaskStep] = task_list

        tmp_task_list.insert(0, TaskStep(ActionType.PARK))
        return tmp_task_list


    def addition(self, task_list: list[TaskStep]):
        return task_list

    def _evaluate_operators(self, task:Task,
                            task_list :list[TaskStep],
                            transformation_operator : PlanTransformationOperator,
                            scoretime_monitor : ScoreTimeMonitor):


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
        overrun_time : float= scoretime_monitor.task_overtime_seconds(task) # Positive if underperformed, negative if time is left
        task_time_remaining : float = scoretime_monitor.time_remaining_seconds_task(task)
        expected_time : float= scoretime_monitor.time_expected_seconds_task_list(task_list)
        if expected_time == 0:
            expected_time: float = 1 # fallback for skipping

        time_ratio : float = max(0, task_time_remaining / expected_time)


        expected_reward: float = 0
        expected_penalty: float = 0
        for t in task_list:
            if t.action_assisted:
                continue
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