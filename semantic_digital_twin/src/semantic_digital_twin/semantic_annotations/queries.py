from krrood.entity_query_language.entity import let, entity, contains
from krrood.entity_query_language.quantify_entity import an
from semantic_digital_twin.collision_checking.trimesh_collision_detector import TrimeshCollisionDetector
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.world_entity import Region

# def query_fav_drink(world):
#     """
#
#     """
#     body = let(type_=Region, domain=world.regions)
#     query = an(entity(body, contains(body.name.name, "kitchen")))
#     kitchen_room_area = list(query.evaluate())[0]
#     return [float(kitchen_room_area.global_pose.x.to_np()[0]), float(kitchen_room_area.global_pose.y.to_np()[0]), float(kitchen_room_area.global_pose.z.to_np()[0])]


from semantic_digital_twin.world_description.world_entity import Human
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
#from semantic_digital_twin.semantic_annotations.semantic_annotation import Cola  # adjust import path if needed

#rody = Human(name=PrefixedName("Rody"), age=30, height=1.75, weight=70, fav_drink="Cola")

#print(rody.name)
#print(rody.fav_drink)


human1 = Human(name=PrefixedName("Alice"))#, fav_drink="Cola")
print(human1.name)