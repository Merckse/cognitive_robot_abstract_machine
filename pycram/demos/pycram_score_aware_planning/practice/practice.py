import time

from common.hsrb_testing import setup_world
from common.types import TaskStep, ActionType
from helper_methods import generic_object_spawner, navigation_subplan, pickup_subplan, place_subplan
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import Arms, TaskStatus
from pycram.motion_executor import simulated_robot
from pycram.plans.factories import sequential, make_node
from pycram.plans.plan_node import PlanNode
from pycram.robot_plans.actions.core.navigation import NavigateAction
from pycram.robot_plans.actions.core.pick_up import PickUpAction
from pycram.robot_plans.actions.core.placing import PlaceAction
from pycram.robot_plans.actions.core.robot_body import MoveTorsoAction, ParkArmsAction
from semantic_digital_twin.datastructures.definitions import TorsoState
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.semantic_annotations.semantic_annotations import Bowl
from semantic_digital_twin.spatial_types.spatial_types import Pose

world, dispatcher = setup_world()
hsrb = HSRB.from_world(world)

context = Context(world , hsrb)


plan = sequential(
    [
        MoveTorsoAction(TorsoState.HIGH),
        ParkArmsAction(Arms.BOTH),
        MoveTorsoAction(TorsoState.LOW),

    ],
    context=context,
)
task_list : list[TaskStep] = [ TaskStep(ActionType.MOVE_TORSO), TaskStep(ActionType.MOVE_TORSO)]
plan_node = plan.plan.root.children

def _build_stabilized_plan( plan_nodes: list[PlanNode], task_list: list[TaskStep],
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
    reconciled: list[PlanNode] = []
    node_cursor: int = 0

    for step in task_list:
        head_node = plan_nodes[node_cursor] if node_cursor < len(plan_nodes) else None
        if head_node is not None and _node_action_type(head_node) == step.action_type:
            # already represented by an existing, already-built node -> reuse it
            reconciled.append(head_node)
            node_cursor += 1
        else:
            # not represented -> inserted by the repair operator, build it from the TaskStep
            built_node = _build_node_for_step(step, context)
            if built_node is not None:
                reconciled.append(built_node)

    return reconciled

def _node_action_type( node: PlanNode) -> ActionType | None:
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

def _build_node_for_step(step: TaskStep, context: Context) -> PlanNode | None:
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
        case ActionType.MOVE_TORSO:
            action = MoveTorsoAction(TorsoState.HIGH)
        case _:
            raise NotImplementedError(f"Action type not implemented: {step.action_type}")
    if action is None:
        return None
    return make_node(action)

def replan_with_asstistance(self, task_list : list[TaskStep]):
    # TODO
    return task_list

task = _build_stabilized_plan(plan_node, task_list, context)
print(task)

# with simulated_robot:
#     plan.perform()
#     plan.perform()
#
#     child = plan.children
#     for c in child:
#         print(c)
#         print(c.status)
#     #     if c.status == TaskStatus.INTERRUPTED:
#     #         print(c)
#     #         print("interrupted")
#     # plan.resume()
#     # plan.perform()
#     # print(plan.status)
#
#         # print(e)
#         # print("status",plan.status)
        # print(plan.plan)
        # children = plan.children
        #
        # for c in plan.children:
        #     print("CHILD",c,c.status)
