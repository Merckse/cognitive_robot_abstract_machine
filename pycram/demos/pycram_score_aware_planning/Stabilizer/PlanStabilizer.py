import logging
from collections import Counter
from dataclasses import dataclass, replace
from typing import NamedTuple

from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from common.cram_types import Task, Status, TaskStep, ActionType
from common.values import evaluation, lookup_operators
from helper_methods import get_remaining_task_steps, navigation_subplan, pickup_subplan, place_subplan, \
    throw_away_subplan, open_subplan, reposition_task_list, \
    request_reposition_subplan, compute_reposition_pose
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus, PlanTransformationOperator, Arms
from pycram.language import SequentialNode
from pycram.plans.factories import make_node, sequential
from pycram.plans.plan_node import PlanNode
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.robot_plans.actions.core.pick_up import PickUpAction
from pycram.robot_plans.actions.core.placing import PlaceAction
from pycram.robot_plans.actions.core.robot_body import ParkArmsAction, MoveTorsoAction

logger = logging.getLogger(__name__)

ASSISTABLE_ACTION_TYPES: set[ActionType] = {
    ActionType.PICKUP,
    ActionType.PLACE,
    ActionType.THROW_AWAY,
    ActionType.OPEN,
    ActionType.CLOSE,
}
"""The steps a human can take over. PARK, NAVIGATE and DETECT are excluded: there is nothing for a
human to do there and their actions carry no `assisted` field."""


class RepairCandidate(NamedTuple):
    """One operator applied to the failed plan, together with what it produced."""
    operator: PlanTransformationOperator
    expected_value: float
    plan: PlanNode
    task_steps: list[TaskStep]


class RepairResult(NamedTuple):
    """The repair that stabilize() picked, and the numbers used to judge it."""
    operator: PlanTransformationOperator
    expected_value: float
    plan: PlanNode
    task_steps: list[TaskStep]
    stability_distance: int
    probability_stability_before: float
    probability_stability_after: float


@dataclass(kw_only=True)
class PlanStabilizer:
    """Repairs a failed plan by applying the candidate operators of the raised exception."""

    context : Context = None
    """Context is needed for robot and world state. It is updated on method calls."""

    value_tolerance : float = 1e-6
    """How close two repairs' expected values have to be before plan stability, rather than value,
    decides between them."""

    def stabilize(self,
                  context : Context,
                  plan: SequentialNode,
                  task: Task,
                  exception: Exception,
                  scoretime_monitor : ScoreTimeMonitor,
                  exec_steps: list[TaskStep],
                  remaining_candidate_operators : list[PlanTransformationOperator] = [],):
        self.context = context
        candidate_operators = remaining_candidate_operators if remaining_candidate_operators else lookup_operators(exception=exception)

        executed_steps = []
        for task_step in exec_steps:
            if task_step.action_type != ActionType.DETECT:
                executed_steps.append(task_step)
        for i, child in enumerate(plan.plan.root.children):
            task_step = executed_steps[i]
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

        original_identity = []
        for step in remaining_task_steps:
            original_identity.append(self._step_identity(step))
        original_probability_stability = self._probability_stability(remaining_task_steps)

        candidates: list[RepairCandidate] = []
        for op in candidate_operators:
            repaired_task_list = self._build_stabilized_task_list(list(remaining_task_steps), op, context)
            expected_value = self._expected_value(task, repaired_task_list, scoretime_monitor)
            repaired_plan = self._build_stabilized_plan(remaining_plan_nodes, repaired_task_list, plan.plan.context)
            candidates.append(RepairCandidate(op, expected_value, repaired_plan, repaired_task_list))

        best_expected_value = max(c.expected_value for c in candidates)
        contenders = []
        for candidate in candidates:
            if candidate.expected_value >= best_expected_value - self.value_tolerance:
                contenders.append(candidate)
        winner = min(contenders,
                     key=lambda c: self._stability_distance_to(original_identity, c.task_steps))

        winner_distance = self._stability_distance_to(original_identity, winner.task_steps)
        repaired_probability_stability = self._probability_stability(winner.task_steps)
        logger.info(
            f"stabilize: {len(contenders)} of {len(candidates)} operator(s) within "
            f"{self.value_tolerance} of the best expected value {best_expected_value:.2f}; "
            f"chose {winner.operator} with plan-stability distance {winner_distance}, "
            f"Stability() {original_probability_stability:.3f} -> {repaired_probability_stability:.3f}"
        )

        return RepairResult(winner.operator, winner.expected_value, winner.plan, winner.task_steps,
                            winner_distance, original_probability_stability,
                            repaired_probability_stability)

    def _build_stabilized_task_list(self, task_list: list[TaskStep], operator: PlanTransformationOperator,context: Context) -> list[TaskStep]:
        if operator == PlanTransformationOperator.SKIP:
            transformed_task_list = self.skip(task_list)
        elif operator == PlanTransformationOperator.RETRY:
            transformed_task_list = self.retry(task_list)
        elif operator == PlanTransformationOperator.REPLAN:
            transformed_task_list = self.replan(task_list)
        elif operator == PlanTransformationOperator.SUBSTITUTE_WITH_ASSISTANCE:
            transformed_task_list = self.substitute_with_assistance(task_list)
        elif operator == PlanTransformationOperator.REQUEST_OBJECT_REPOSITION:
            transformed_task_list = self.request_object_reposition(task_list)
        elif operator == PlanTransformationOperator.REPOSITION_ROBOT:
            transformed_task_list = self.reposition_robot_and_retry(task_list, context)
        else:
            transformed_task_list = task_list

        # shared with the executioner so a repaired plan inserts navigation on the same rules the
        # original plan was built with
        return reposition_task_list(
            context=context, task_list=transformed_task_list, robot=self.context.robot
        )

    def _build_stabilized_plan(self, plan_nodes: list[PlanNode], task_list: list[TaskStep],
                               context: Context) -> PlanNode:
        """Reuses the still-pending nodes of the failed plan and only builds what an operator changed."""
        stabilized_plan_node: list[PlanNode] = []
        node_pointer: int = 0

        for step in task_list:
            head_node = plan_nodes[node_pointer] if node_pointer < len(plan_nodes) else None
            if head_node is not None and self._node_action_type(head_node) == step.action_type:
                if step.action_assisted:
                    # getattr, not attribute access: only the manipulation actions carry an
                    # `assisted` field
                    if getattr(head_node.action, "assisted", False) != step.action_assisted:
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
        """Maps an existing plan node back to its ActionType so it can be compared to a TaskStep."""
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
        """Builds a single executable plan node from a TaskStep, mirroring generate_plan_task."""
        challenge_mode = context.challenge_mode
        arm = Arms.LEFT
        world = context.world

        # these all need a concrete annotation type to look the body up with, so drop the one
        # step rather than taking the whole run down
        if step.action_type in (
            ActionType.PICKUP, ActionType.PLACE, ActionType.THROW_AWAY, ActionType.OPEN,
            ActionType.REQUEST_REPOSITION,
        ) and not isinstance(step.object_annotations, type):
            logger.warning(
                f"cannot build a {step.action_type} node: no object annotation on the step "
                f"(got {step.object_annotations!r}) -> skipping it in the repaired plan"
            )
            return None

        match step.action_type:
            case ActionType.NAVIGATE:
                action = navigation_subplan(challenge_mode=challenge_mode, target_location=step.location, world=world,
                                            pose_override=step.nav_pose_override)
            case ActionType.PICKUP:
                action = pickup_subplan(object_annotation=step.object_annotations, arm=arm,
                                        world=world, assisted=step.action_assisted,
                                        perceived_object=step.perceived_object)
            case ActionType.PLACE:
                action = place_subplan(challenge_mode=challenge_mode, object_annotation=step.object_annotations, arm=arm,
                                       target_location=step.location, world=world, assisted=step.action_assisted, robot=context.robot,
                                       perceived_object=step.perceived_object)
            case ActionType.PARK:
                action = ParkArmsAction(Arms.LEFT)
            case ActionType.REQUEST_REPOSITION:
                action = request_reposition_subplan(object_annotation=step.object_annotations,
                                                    arm=arm, world=world,
                                                    perceived_object=step.perceived_object)
            case ActionType.DETECT:
                return None
            case ActionType.THROW_AWAY:
                action =  throw_away_subplan(object_annotation=step.object_annotations, arm=arm, world=world, assisted=step.action_assisted,
                                             perceived_object=step.perceived_object)
            case ActionType.OPEN:
                action = open_subplan(object_annotation=step.object_annotations, arm=arm,
                               world=world, assisted=step.action_assisted)
            case _:
                raise NotImplementedError(f"Action type not implemented: {step.action_type}")
        if action is None:
            return None
        return make_node(action)

    def skip(self,task_list: list[TaskStep]):
        """Skips the rest of the task."""
        return []

    def retry(self, task_list: list[TaskStep]):
        """Retries the task from the failed step, re-perceiving and parking the arms first."""
        tmp_task_list : list[TaskStep] = task_list
        tmp_task_list.insert(0, TaskStep(ActionType.PARK))
        tmp_task_list.insert(0, TaskStep(ActionType.DETECT))
        return tmp_task_list

    def request_object_reposition(self, task_list: list[TaskStep]):
        tmp_task_list : list[TaskStep] = task_list
        if not tmp_task_list:
            return tmp_task_list

        failed_step = tmp_task_list[0]
        tmp_task_list.insert(0, TaskStep(
            ActionType.REQUEST_REPOSITION,
            object_annotations=failed_step.object_annotations,
            location=failed_step.location,
        ))
        return tmp_task_list

    def reposition_robot_and_retry(self, task_list: list[TaskStep], context: Context) -> list[TaskStep]:
        if not task_list:
            return task_list
        failed_step = task_list[0]
        override = compute_reposition_pose(context, failed_step)
        if override is None:
            return task_list
        repositioned_list = list(task_list)
        repositioned_list[0] = replace(failed_step, nav_pose_override=override)
        return repositioned_list

    def substitute_with_assistance(self, task_list: list[TaskStep]):
        """
        Has a human perform the failed manipulation instead of the robot.

        Only flags a step a human can actually take over, and only a single one: needing help with
        one action says nothing about the next, so the flag must not spread down the plan.
        """
        tmp_task_list : list[TaskStep] = task_list
        for step in tmp_task_list:
            if step.action_type in ASSISTABLE_ACTION_TYPES:
                step.action_assisted = True
                break
        return tmp_task_list

    def replan(self, task_list: list[TaskStep]):
        tmp_task_list : list[TaskStep] = task_list
        tmp_task_list.insert(0, TaskStep(ActionType.PARK))
        return tmp_task_list

    def _expected_value(self, task : Task, task_list: list[TaskStep], scoretime_monitor : ScoreTimeMonitor) -> float:
        """Expected reward of a repair, discounted by how much of it the challenge can still afford."""
        expected_time : float= scoretime_monitor.time_expected_seconds_task_list(task_list)
        if expected_time == 0:
            expected_time: float = 1 # fallback for skipping

        # measured against the challenge clock, not the task's own estimate: a task that just
        # failed has by definition overrun its estimate, which would clamp the ratio to 0 and
        # make skipping the best option after every first failure
        challenge_time_remaining : float = scoretime_monitor.time_remaining_seconds_total()
        time_ratio : float = min(1.0, max(0.0, challenge_time_remaining / expected_time))

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

    @staticmethod
    def _step_identity(step: TaskStep) -> tuple:
        """
        What makes a step a different action to carry out: what is done, to what, where, and
        whether a human is asked for help.

        Deliberately not the object's identity: a repair that rebuilds a step from scratch
        produces a different object describing the very same action, and counting that as a
        change would report every repair as maximally unstable.
        """
        object_name = getattr(step.object_annotations, "__name__", None)
        return step.action_type, object_name, step.location, step.action_assisted

    def _stability_distance_to(self, original_identity: list[tuple], repaired_plan: list[TaskStep]) -> int:
        """
        Plan-stability distance of Fox et al.: the number of steps that would have to be removed
        from one plan plus the number that would have to be added to arrive at the other. 0 means
        the repair reused the original plan entirely.
        """
        original_steps = Counter(original_identity)
        repaired_identities = []
        for step in repaired_plan:
            repaired_identities.append(self._step_identity(step))
        repaired_steps = Counter(repaired_identities)
        removed = original_steps - repaired_steps
        added = repaired_steps - original_steps
        return sum(removed.values()) + sum(added.values())

    @staticmethod
    def _probability_stability(task_steps: list[TaskStep]) -> float:
        """
        Stability(Pi_task) of the theoretical model: the product of the step probabilities.

        """
        stability = 1.0
        for step in task_steps:
            stability *= step.action_probability
        return stability
