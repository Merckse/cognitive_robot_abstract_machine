from common.hsrb_testing import setup_world
from semantic_digital_twin.semantic_annotations.semantic_annotations import Bowl

world, dispatcher = setup_world()

world.get_semantic_annotations_by_type(Bowl)