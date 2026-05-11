"""
put knowledge queries here, like:
empty(supportingSurface) = true
def misplaced(object):
    get objects location
    if location = place where belongs -> return false
    else -> return true
"""
import os
from contextlib import contextmanager

from sqlalchemy.dialects.oracle.dictionary import all_objects

from krrood.symbolic_math.symbolic_math import Scalar
from semantic_digital_twin.reasoning.queries import semantic_annotations_on_surfaces
from semantic_digital_twin.semantic_annotations.mixins import HasSupportingSurface
from semantic_digital_twin.semantic_annotations.semantic_annotations import Food, Fruit, Cuttlery, Plate, Cup, Bowl, \
    Bottle, CounterTop, Table, ShelfLayer
from semantic_digital_twin.spatial_types.spatial_types import Pose
from semantic_digital_twin.world import World
from semantic_digital_twin.reasoning.predicates import is_supported_by

from time import sleep


import math

from semantic_digital_twin.world_description.world_entity import SemanticAnnotation, Body


def perf_print(message: str):
    pass


@contextmanager
def perf_step(label: str):
    yield


def semantic_annotations_on_surface_cached(
    furniture: HasSupportingSurface,
    world: World,
    surface_cache: dict | None = None,
):
    cache_key = str(furniture.name)
    if surface_cache is not None and cache_key in surface_cache:
        perf_print(f"CACHE HIT semantic_annotations_on_surfaces: {cache_key}")
        return surface_cache[cache_key]

    with perf_step(f"semantic_annotations_on_surfaces: {cache_key}"):
        annotations = semantic_annotations_on_surfaces([furniture], world)

    if surface_cache is not None:
        surface_cache[cache_key] = annotations
    return annotations


def is_supported_by_surface_cached(
    obj: SemanticAnnotation,
    surface: HasSupportingSurface,
    support_cache: dict | None = None,
):
    cache_key = (str(obj.name), str(surface.name))
    if support_cache is not None and cache_key in support_cache:
        perf_print(f"CACHE HIT is_supported_by: {cache_key[0]} on {cache_key[1]}")
        return support_cache[cache_key]

    with perf_step(f"is_supported_by: {obj.name} on {surface.name}"):
        result = is_supported_by(obj.root, surface.root)

    if support_cache is not None:
        support_cache[cache_key] = result
    return result


def misplaced(
    obj: SemanticAnnotation,
    world: World,
    surface_cache: dict | None = None,
    support_cache: dict | None = None,
):
    """
    returns true if object is misplaced and false if it is already at the correct location
    """

    with perf_step(f"misplaced determine correct location: {obj.name}"):
        if isinstance(obj, (Food, Bottle)):
            correct_location = world.get_semantic_annotations_by_name("cooking_table")[0]
        elif isinstance(obj, (Cuttlery, Plate, Bowl, Cup)):
            correct_location = world.get_semantic_annotations_by_name("counterTop")[0]
        else:
            correct_location = world.get_semantic_annotations_by_name("table")[0]


    if isinstance(correct_location, (CounterTop, Table, ShelfLayer)):
        with perf_step(f"misplaced direct support check: {obj.name} on {correct_location.name}"):
            is_on_correct_location = is_supported_by_surface_cached(
                obj,
                correct_location,
                support_cache,
            )
        if is_on_correct_location:
            return False, correct_location
        else:
            return True, correct_location
    else:
        TypeError("Correct location is not of type HasSupportingSurface")
        return True, correct_location


def is_empty(
    furniture: HasSupportingSurface,
    known_objects: list[SemanticAnnotation],
    world: World,
    surface_cache: dict | None = None,
):
    """
    returns false, if the furniture has an already perceived object on it
    returns true, if it is empty (only acknowledged objects that are already perceived)
    """
    with perf_step(f"is_empty query surface contents: {furniture.name}"):
        annotations_on_surface = semantic_annotations_on_surface_cached(
            furniture,
            world,
            surface_cache,
        )
    with perf_step(f"is_empty scan {len(annotations_on_surface)} annotations on {furniture.name}"):
        for ann in annotations_on_surface:
            if ann in known_objects:
                return False
        return True

def human_near():
    # only useful in real scenario, no human in simulation
    return False

    


def reachable(object : SemanticAnnotation):
    # debug, WIP for later
    return True


    surfaces = []
    for annotation in world.semantic_annotations:
        if isinstance(annotation, (Table, CounterTop)):#, ShelfLayer)):
            surfaces.append(annotation)


    object_surface = None
    for surface in surfaces:
        for anno in semantic_annotations_on_surfaces([surface], world):
            print(surface.name, anno.name)
            if anno == object_to_reach:
                object_surface = surface

    if object_surface is None:
        Exception(f"Object {object_to_reach} is not on a Table, CounterTop or ShelfLayer surface")

    surface_region = object_surface.supported_surface[0].get_region().combined_mesh()

    edges = surface_region.edges_boundary

    print("Region: ", surface_region, "\n Edges: ", edges)

    # # --- distance check ---
    # dx = object_location.x - robot_location.x
    # dy = object_location.y - robot_location.y
    # dist = math.sqrt(dx**2 + dy**2)
    #
    # if dist > 0.8 or dist < 0.3:
    #     return False
    #
    # # --- height check ---
    # if object_location.z > Scalar(1.2):
    #     return False
    #
    # # --- blocking check ---
    # for obj in world.bodies:
    #
    #     if obj.global_pose is None:
    #         continue
    #
    #     if is_between(robot_location, object_location, obj.global_pose):
    #         if is_same_height(obj.global_pose, object_location):
    #             return False

    # return True


def is_between(p1: Pose, p2: Pose, p: Pose):
    """
    Checks if point p lies approximately on the segment p1->p2
    """

    # vector projection
    dx1 = p2.x - p1.x
    dy1 = p2.y - p1.y

    dx2 = p.x - p1.x
    dy2 = p.y - p1.y

    dot = dx1*dx2 + dy1*dy2
    len_sq = dx1*dx1 + dy1*dy1

    if dot < 0 or dot > len_sq:
        return False

    # distance from line
    cross = abs(dx1*dy2 - dy1*dx2)
    dist = cross / math.sqrt(len_sq)

    return dist < 0.1


def is_same_height(p1: Pose, p2: Pose):
    return abs(p1.z - p2.z) < 0.1
