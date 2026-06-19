import datetime
import logging
import threading
from queue import Empty, Queue
from typing import Optional

from Evaluate.CompositeEvaluator import CompositeEvaluator
from Execution.clean_the_table import scoretime_monitor
from ScoreTimeMonitoring.ScoreTimeMonitor import ScoreTimeMonitor
from Stabilizer.PlanStabilizer import PlanStabilizer
from common.types import Task, TaskStep
from common.values import CHALLENGE_TASKS
from helper_methods import generate_plan_task, perceive_and_spawn_all_objects
from pycram.datastructures.dataclasses import Context
from pycram.datastructures.enums import TaskStatus
from pycram.motion_executor import simulated_robot
from pycram.plans.factories import make_node, sequential
from semantic_digital_twin.robots.abstract_robot import AbstractRobot, Arm

from demos.pycram_score_aware_planning.Structurizer.structurizer import PlanStructurizer
from demos.pycram_score_aware_planning.common.types import ChallengeMode

logger = logging.getLogger(__name__)


class ArmExecutorInterface:
    """
    Execution interface bound to a single robot arm.
    Drains its assigned task queue, running each task step-by-step
    and recording score events via the shared monitor.
    """

    def __init__(
        self,
        arm: Optional[Arm],
        arm_index: int,
        context: Context,
        stabilizer: PlanStabilizer,
        scoretime_monitor: ScoreTimeMonitor,
    ):
        self.arm = arm
        self.arm_index = arm_index
        self.context = context
        self.stabilizer = stabilizer
        self.scoretime_monitor = scoretime_monitor
        self._task_queue: Queue[Task] = Queue()

    def enqueue(self, task: Task) -> None:
        self._task_queue.put(task)

    def run(self) -> None:
        """Execute all queued tasks for this arm."""
        while not self._task_queue.empty():
            try:
                task: Task = self._task_queue.get_nowait()
            except Empty:
                break
            self._execute_task(task)
            self._task_queue.task_done()

    def _execute_task(self, task: Task) -> None:
        task.action_list = generate_plan_task(task=task, context=self.context)
        task.task_begin = datetime.datetime.now()

        with simulated_robot:
            for i, action in enumerate(task.action_list):
                task_step: TaskStep = task.task_steps[i]

                if task_step.uncertain:
                    try:
                        perceive_and_spawn_all_objects(world=self.context.world)
                    except Exception:
                        pass

                plan = sequential([], context=self.context)
                plan.add_child(make_node(action))
                plan.perform()

                self.scoretime_monitor.record_score(task_step=task_step, plan=plan)

                if plan.status is TaskStatus.FAILED:
                    try:
                        plan = self.stabilizer.stabilize(plan)
                    except NotImplementedError:
                        logger.warning("PlanStabilizer not yet implemented — skipping stabilization.")


class Executor:
    """
    Unified task executor.

    Detects the robot's arm count from the world context and creates one
    ArmExecutorInterface per arm.  Each evaluation-structurize cycle batches
    num_arms tasks, assigns one per interface, and runs them in parallel
    threads when the robot has more than one arm.

    CompositeEvaluator handles probability + score estimation internally and
    prints a summary after each estimate call — no separate scorer needed here.
    """

    def __init__(
        self,
        context: Context,
        task_mode: ChallengeMode,
        challenge_duration_seconds: int = 500,
    ):
        self.context = context
        self.task_mode = task_mode

        robot: AbstractRobot = context.robot
        self.arms: list[Arm] = getattr(robot, "arms", [])
        self.num_arms: int = max(len(self.arms), 1)
        logger.info(
            "Executor: robot '%s' — %d arm(s) detected, %d parallel interface(s) created.",
            robot.name,
            self.num_arms,
            self.num_arms,
        )

        self.evaluator = CompositeEvaluator()
        self.structurizer = PlanStructurizer()
        self.stabilizer = PlanStabilizer()
        self.scoretime_monitor = ScoreTimeMonitor(
            challenge_duration_seconds=challenge_duration_seconds
        )

        self.arm_interfaces: list[ArmExecutorInterface] = [
            ArmExecutorInterface(
                arm=self.arms[i] if i < len(self.arms) else None,
                arm_index=i,
                context=self.context,
                stabilizer=self.stabilizer,
                scoretime_monitor=self.scoretime_monitor,
            )
            for i in range(self.num_arms)
        ]

    def run(self, task_list: Optional[list[Task]] = None) -> None:
        """
        Evaluate → structurize → execute until task_list is empty.

        CompositeEvaluator.estimate() prints a summary on every call, so the
        first loop iteration serves as the initial score preview.

        Each iteration takes a batch of num_arms tasks from the front of the
        prioritized list and assigns one to each arm interface.  Multi-arm
        robots execute the batch in parallel threads; single-arm robots run
        it directly on the main thread.
        """
        if task_list is None:
            task_list = list(CHALLENGE_TASKS.get(self.task_mode) or [])

        while task_list:
            evaluated: list[Task] = self.evaluator.estimate(
                context=self.context, task_list=task_list
            )
            task_list = self.structurizer.structurize(task_list=evaluated)

            # Pull one task per arm from the front of the prioritized list
            batch: list[Task] = []
            for _ in range(self.num_arms):
                if not task_list:
                    break
                batch.append(task_list.pop(0))

            for idx, task in enumerate(batch):
                self.arm_interfaces[idx].enqueue(task)

            if self.num_arms > 1:
                threads = [
                    threading.Thread(
                        target=iface.run,
                        name=f"arm-{iface.arm_index}",
                        daemon=True,
                    )
                    for iface in self.arm_interfaces
                ]
                for t in threads:
                    t.start()
                for t in threads:
                    t.join()
            else:

                self.arm_interfaces[0].run()


# ---------------------------------------------------------------------------
# Entry point — PR2 demo setup (2-arm robot → 2 ArmExecutorInterfaces)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    from demos.pycram_score_aware_planning.common.pr2_testing import setup_world
    from demos.pycram_score_aware_planning.helper_methods import generic_object_spawner
    from semantic_digital_twin.robots.pr2 import PR2
    from semantic_digital_twin.world_description.geometry import Color

    world, dispatcher = setup_world()

    pr2 = PR2.from_world(world)
    context = Context(world=world, robot=pr2)
    context.evaluate_conditions = False
    dispatcher.known_furniture = world.bodies

    generic_object_spawner(["Bowl"],   [(1.325,   6.23,   0.81)],  world, color=Color.GREEN())
    generic_object_spawner(["Plate"],  [(-0.15,   0.88,   0.85)],  world, color=Color.ORANGE())
    generic_object_spawner(["Milk"],   [(1.037,  -2.31,   0.645)], world, color=Color.RED())
    generic_object_spawner(["Knife"],  [(4.65,    4.84,   1.62)],  world, color=Color.CYAN())
    generic_object_spawner(["Apple"],  [(4.135,   1.865,  0.54)],  world, color=Color.WHITE())
    generic_object_spawner(["Cereal"], [(2.42,    0.128,  0.945)], world, color=Color.BLUE())

    executor = Executor(
        context=context,
        task_mode=ChallengeMode.PP,
        challenge_duration_seconds=500,
    )
    executor.run()