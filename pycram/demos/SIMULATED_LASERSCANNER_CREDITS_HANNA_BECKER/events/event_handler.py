# source for base: https://medium.com/@idelossantosruiz/events-in-python-e2b3cb76ac2d
import os
from typing import Tuple, Any
from contextlib import contextmanager


from demos.SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.classes.tasks import SetTableTask, CleanTableTask, PutAwayObjectTask, LoadDishwasherTask, \
    UnloadDishwasherTask
from semantic_digital_twin.reasoning.queries import semantic_annotations_on_surfaces
from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.semantic_annotations.semantic_annotations import Bowl, Cuttlery, Plate, Cup
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body, SemanticAnnotation
from semantic_digital_twin.reasoning.predicates import reachable
from demos.SIMULATED_LASERSCANNER_CREDITS_HANNA_BECKER.actions.predicate_mock import (
    reachable,
    misplaced,
    human_near,
    semantic_annotations_on_surface_cached,
    is_supported_by_surface_cached,
)
import datetime
from time import sleep, time as tm


def perf_print(message: str):
    pass


@contextmanager
def perf_step(label: str):
    yield


class EventDispatcher:
    def __init__(self):
        self._listeners = []

        #----- correct locations for items -----------------------------------------------------------------------------
        self.correct_location_tableware = None
        self.correct_location_food = None
        self.correct_location_drinks = None
        self.correct_location_all_other_items = None

        ##### replaces direct use of predicate functions for faster runtime ############################################
        self.perceived_objects = []
        """
        objects detected via robot perception
        """

        self.reachable_objects = []
        """
        objects detected via robot perception, that are reachable from one of the observe positions
        """

        self.misplaced_objects = []
        """
        objects that are at the wrong location, that are misplaced
        """

        self.known_furniture = []
        """
        furniture and walls existing in the world
        """

        ################################################################################################################

        self.activated_tasks = []
        """
        all tasks that were triggered
        """

        self.used_by_tasks = []
        """
        objects that are used by a task
        """

        ########## DEBUG ###############################################################################################
        self.locations = []
        self.trigger_nr = 0
        self.surface_annotation_cache = {}
        self.support_relation_cache = {}
        self.perceived_objects_changed = False

        self.add_listener(update_perceived_objects)
        self.add_listener(trigger_task)
        #self.add_listener(update_reachable_objects)

    def add_listener(self, listener):
        """Register a new listener."""
        self._listeners.append(listener)

    def remove_listener(self, listener):
        """Unregister an existing listener."""
        if listener in self._listeners:
            self._listeners.remove(listener)

    def trigger_event(self, event_data : list[Body], world : World):
        """Fire the event, passing event_data to every listener."""
        with perf_step(f"trigger_event with {len(event_data)} visible bodies and {len(self._listeners)} listeners"):
            sem_annotations = []
            for data in event_data:
                if not data in self.known_furniture:
                    sem_annotations.append(world.get_semantic_annotation_by_name(data.name))

            for listener in self._listeners:
                listener_name = getattr(listener, "__name__", str(listener))
                with perf_step(f"listener: {listener_name}"):
                    listener(self, sem_annotations, world)


# Usage example
def update_perceived_objects(handler : EventDispatcher, data : list[SemanticAnnotation], world : World):
    with perf_step("update_perceived_objects"):
        print("#"*110 + "\n" + "#"*110)
        print(f"NEW UPDATE No. {handler.trigger_nr}\n \n")
        handler.trigger_nr += 1
        new_objects = 0

        for obj in data:
            if obj not in handler.perceived_objects: # furniture already filtered out
                new_objects += 1
                obj_sem_annotation = world.get_semantic_annotation_by_name(obj.name)

                handler.perceived_objects.append(obj_sem_annotation)

                with perf_step(f"misplaced({obj.name})"):
                    out_misplaced = misplaced(
                        obj_sem_annotation,
                        world,
                        handler.surface_annotation_cache,
                        handler.support_relation_cache,
                    )

                with perf_step(f"reachable({obj.name})"):
                    is_reachable = reachable(obj_sem_annotation)

                if is_reachable:
                    handler.reachable_objects.append(obj_sem_annotation)

                if out_misplaced[0]:
                    handler.misplaced_objects.append(obj_sem_annotation)

                handler.locations.append([obj_sem_annotation, out_misplaced[1]])
        handler.perceived_objects_changed = new_objects > 0
        perf_print(f"new objects processed: {new_objects}")
        with perf_step("print perceived objects"):
            print_perceived_objects(handler)



# def update_reachable_objects(handler : EventDispatcher, data : list[SemanticAnnotation], world : World):
#     robot = world.get_body_by_name("base_footprint")
#     print("I can reach following objects:")
#     for obj in data:
#         if (obj not in handler.reachable_objects) and (obj not in handler.known_furniture):
#             if reachable(obj.global_pose, robot.global_pose, world):
#                 handler.reachable_objects.append(obj)
#                 print(f" - {obj.name}")

# noinspection D
def trigger_task(handler: EventDispatcher, data : list[SemanticAnnotation], world : World):
    with perf_step("trigger_task"):
        # trigger set the table
        ts = tm()
        # time = datetime.datetime.fromtimestamp(ts)
        time = datetime.datetime(year=2026, month=5, day=6, hour=9, minute=10) # for testing
        with perf_step("trigger set table task"):
            if (time.hour == 9 or time.hour == 13 or time.hour == 19) and not human_near():
                table_name = "table"
                exists = False
                for task in handler.activated_tasks:
                    if task.name == ("set_table_task_" + table_name):
                        exists = True
                        if handler.perceived_objects_changed:
                            with perf_step(f"update task: {task.name}"):
                                task.update_to_current_world_state(
                                    world,
                                    handler.perceived_objects,
                                    handler.surface_annotation_cache,
                                )
                        else:
                            perf_print(f"skip update task {task.name}: perceived objects unchanged")
                if not exists:
                    with perf_step("create task: set_table_task_table"):
                        handler.activated_tasks.append(
                            SetTableTask(
                                "set_table_task_"+table_name,
                                table=world.get_semantic_annotation_by_name("table"),
                                world=world,
                                perceived_objects=handler.perceived_objects,
                                surface_cache=handler.surface_annotation_cache,
                            )
                        )


        # trigger clean the table
        with perf_step("trigger clean table task"):
            if (time.hour != 9 and time.hour != 13 and time.hour != 19) and not human_near():
                table_name = "table"
                exists = False
                for task in handler.activated_tasks:
                    if task.name == ("clean_table_task_" + table_name):
                        exists = True
                        if handler.perceived_objects_changed:
                            with perf_step(f"update task: {task.name}"):
                                task.update_to_current_world_state(world, handler.perceived_objects)
                        else:
                            perf_print(f"skip update task {task.name}: perceived objects unchanged")
                if not exists:
                    with perf_step("create task: clean_table_task_table"):
                        handler.activated_tasks.append(CleanTableTask("clean_table_task_"+table_name, table=world.get_semantic_annotation_by_name("table"), world=world, perceived_objects=handler.perceived_objects))

        # trigger put away object
        with perf_step(f"trigger put away object tasks for {len(handler.misplaced_objects)} misplaced objects"):
            for obj in handler.misplaced_objects:
                task_name = "put_away_object_task_" + obj.name.name
                exists = False
                for task in handler.activated_tasks:
                    if task.name == task_name:
                        exists = True
                        if handler.perceived_objects_changed:
                            with perf_step(f"update task: {task.name}"):
                                task.update_to_current_world_state(world, handler.perceived_objects)
                        else:
                            perf_print(f"skip update task {task.name}: perceived objects unchanged")
                if not exists:
                    with perf_step(f"create task: {task_name}"):
                        handler.activated_tasks.append(PutAwayObjectTask(task_name, required_objects=[obj], world=world, perceived_objects=handler.perceived_objects))

        with perf_step(f"scan {len(handler.perceived_objects)} perceived objects for dishware"):
            perceived_dishware = []
            for obj in handler.perceived_objects:
                annotation = world.get_semantic_annotation_by_name(obj.name)
                if isinstance(annotation, (Cuttlery, Plate, Bowl, Cup)):
                    perceived_dishware.append((annotation, obj))

        # trigger load dishwasher task
        load_dishwasher_task = None
        load_dishwasher_objects = []
        if len(perceived_dishware) > 0:
            with perf_step(f"scan {len(perceived_dishware)} perceived dishware objects for load dishwasher"):
                for _, obj_body in perceived_dishware:
                    if obj_body not in handler.misplaced_objects and not human_near():
                        load_dishwasher_task = "load_dishwasher_task"
                        load_dishwasher_objects.append(obj_body)
        else:
            perf_print("skip counterTop query: no perceived dishware/cutlery")

        if load_dishwasher_task is not None:
            with perf_step("trigger load dishwasher task"):
                exists = False
                for task in handler.activated_tasks:
                    if task.name == load_dishwasher_task:
                        exists = True
                        if handler.perceived_objects_changed:
                            with perf_step(f"update task: {task.name}"):
                                task.update_to_current_world_state(
                                    world,
                                    handler.perceived_objects,
                                    surface_cache=handler.surface_annotation_cache,
                                    support_cache=handler.support_relation_cache,
                                    required_objects=load_dishwasher_objects,
                                )
                        else:
                            perf_print(f"skip update task {task.name}: perceived objects unchanged")
                if not exists:
                    with perf_step("create task: load_dishwasher_task"):
                        handler.activated_tasks.append(
                            LoadDishwasherTask(
                                load_dishwasher_task,
                                handler.perceived_objects,
                                world,
                                surface_cache=handler.surface_annotation_cache,
                                support_cache=handler.support_relation_cache,
                                required_objects=load_dishwasher_objects,
                            )
                        )

        # trigger unload dishwasher task
        unload_dishwasher_task = None
        unload_dishwasher_objects = []
        if len(perceived_dishware) > 0:
            dishwasher_rack = world.get_semantic_annotation_by_name("dishwasher_rack")
            with perf_step(f"scan {len(perceived_dishware)} perceived dishware objects for unload dishwasher"):
                for annotation, obj_body in perceived_dishware:
                    if is_supported_by_surface_cached(
                        annotation,
                        dishwasher_rack,
                        handler.support_relation_cache,
                    ) and not human_near():
                        unload_dishwasher_task = "unload_dishwasher_task"
                        unload_dishwasher_objects.append(obj_body)
        else:
            perf_print("skip dishwasher_rack query: no perceived dishware/cutlery")

        if unload_dishwasher_task is not None:
            with perf_step("trigger unload dishwasher task"):
                exists = False
                for task in handler.activated_tasks:
                    if task.name == unload_dishwasher_task:
                        exists = True
                        if handler.perceived_objects_changed:
                            with perf_step(f"update task: {task.name}"):
                                task.update_to_current_world_state(
                                    world,
                                    handler.perceived_objects,
                                    surface_cache=handler.surface_annotation_cache,
                                    required_objects=unload_dishwasher_objects,
                                )
                        else:
                            perf_print(f"skip update task {task.name}: perceived objects unchanged")
                if not exists:
                    with perf_step("create task: unload_dishwasher_task"):
                        handler.activated_tasks.append(
                            UnloadDishwasherTask(
                                unload_dishwasher_task,
                                handler.perceived_objects,
                                world,
                                surface_cache=handler.surface_annotation_cache,
                                required_objects=unload_dishwasher_objects,
                            )
                        )

        with perf_step("print tasks"):
            print_tasks(handler)


def print_tasks(handler : EventDispatcher):
    print("\n \n")
    print("ACTIVE TASKS")
    print("-" * 110)

    header = f"{'Name':<60} | {'Feasibility':<15} | {'Score':<10} | {'Normalized Score':<18}"
    print(header)
    print("-" * 110)

    for task in handler.activated_tasks:
        name = task.name
        with perf_step(f"calculate feasibility: {name}"):
            feasibility = task.calculate_feasibility()
        score = task.reward * feasibility
        norm_score = score / task.duration

        line = f"{name:<60} | {feasibility:<15.3f} | {score:<10.3f} | {norm_score:<18.3f}"
        print(line)


def print_perceived_objects(handler : EventDispatcher):
    print("PERCEIVED OBJECTS")
    print("-"*110)
    print(
        f"{'Name':<35} | "
        f"{'Reachable':<10} | "
        f"{'Misplaced':<10} | "
        f"{'Belongs at Location':<30} ")
    print("-"*110)
    
    for obj in handler.perceived_objects:
        is_reachable = False
        is_misplaced = False
        location = None

        if obj in handler.reachable_objects:
            is_reachable = True
        if obj in handler.misplaced_objects:
            is_misplaced = True
        for loc in handler.locations:
            if loc[0]==obj:
                location = loc[1].name.name

        print(
            f"{obj.name.name:<35} | "
            f"{'YES' if is_reachable else 'NO':<10} | "
            f"{'YES' if is_misplaced else 'NO':<10} | "
            f"{str(location):<30} ")
