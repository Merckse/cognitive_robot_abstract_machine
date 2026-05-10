from semantic_digital_twin.semantic_annotations.mixins import HasSupportingSurface
from semantic_digital_twin.semantic_annotations.semantic_annotations import Table, CounterTop, ShelfLayer
from semantic_digital_twin.spatial_types.spatial_types import Pose, Point3, HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.shape_collection import BoundingBoxCollection
from semantic_digital_twin.world_description.world_entity import Body

import random
from time import sleep


def random_location_list(world : World, number_locations : int):
    furniture = world.semantic_annotations
    surfaces = []
    locations = []

    for fur in furniture:
        if isinstance(fur, (Table, CounterTop, ShelfLayer)):
            print(fur.name)
            surfaces.append(fur.as_bounding_box_collection_in_frame(world.root).shapes[0]) # coordinates of the bounding box

    while number_locations > 0:
        for surface in surfaces:
            some_more_randomness = random.random() # use this to skip surfaces randomly, so there aren't only objects on the same surfaces, because a surface is only chosen at a possibility of 70 percent
            if number_locations > 0 and some_more_randomness < 0.7:
                min_x = surface.min_x
                min_y = surface.min_y
                max_x = surface.max_x
                max_y = surface.max_y
                z = surface.max_z

                random_location = Pose(Point3(random.uniform(min_x + 0.1, max_x - 0.1), random.uniform(min_y + 0.1, max_y - 0.1), z))

                if not check_too_close_to_other_object(locations, random_location):
                    locations.append(random_location)
                    number_locations -= 1
            else:
                break


    return locations

def check_too_close_to_other_object(location_list : list[Pose], loc : Pose):
    for obj in location_list:
        if abs(loc.x - obj.x) < 0.1 and abs(loc.y - obj.y):
            return True
        else:
            return False
    return False

# pose.to_homogenous_matrix
def pose_to_homogeneous_transformation_matrix_from_xyz_quaternion(pose : Pose, world : World):
    return HomogeneousTransformationMatrix.from_xyz_quaternion(
        pose.x, pose.y, pose.z, reference_frame=world.root
    )



