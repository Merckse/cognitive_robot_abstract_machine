# ----------------------------------------------------------------------------------------------------------------------
# This script generates the ORM classes for the semantic_digital_twin package.
# Dataclasses can be mapped automatically to the ORM model
# using the ORMatic library, they just have to be registered in the classes list.
# Classes that are self_mapped and explicitly_mapped are already mapped in the model.py file. Look there for more
# information on how to map them.
# ----------------------------------------------------------------------------------------------------------------------
from __future__ import annotations

import os
import uuid
from dataclasses import is_dataclass

import sqlalchemy

import semantic_digital_twin.adapters.procthor.procthor_resolver
import semantic_digital_twin.orm.model
import semantic_digital_twin.reasoning.predicates
import semantic_digital_twin.robots.abstract_robot
import semantic_digital_twin.semantic_annotations.semantic_annotations
import semantic_digital_twin.world  # ensure the module attribute exists on the package
import semantic_digital_twin.world_description.degree_of_freedom
import semantic_digital_twin.world_description.geometry
import semantic_digital_twin.world_description.shape_collection
import semantic_digital_twin.world_description.world_entity
from krrood.class_diagrams import ClassDiagram
from krrood.ormatic.ormatic import ORMatic
from krrood.ormatic.utils import classes_of_module
from krrood.utils import recursive_subclasses
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.orm.model import *  # type: ignore
from semantic_digital_twin.reasoning.predicates import ContainsType
from semantic_digital_twin.semantic_annotations.mixins import HasBody
from semantic_digital_twin.spatial_computations.forward_kinematics import (
    ForwardKinematicsManager,
)
from semantic_digital_twin.world import (
    ResetStateContextManager,
    WorldModelUpdateContextManager,
)
from semantic_digital_twin.world import WorldModelManager
from semantic_digital_twin.world_description.connections import (
    FixedConnection,
    HasUpdateState,
)

all_classes = set(
    classes_of_module(semantic_digital_twin.world_description.world_entity)
)
all_classes |= set(classes_of_module(semantic_digital_twin.world_description.geometry))
all_classes |= set(
    classes_of_module(semantic_digital_twin.world_description.shape_collection)
)
all_classes |= set(classes_of_module(semantic_digital_twin.world))
all_classes |= set(
    classes_of_module(semantic_digital_twin.datastructures.prefixed_name)
)

all_classes |= set(
    classes_of_module(semantic_digital_twin.world_description.connections)
)
all_classes |= set(
    classes_of_module(semantic_digital_twin.semantic_annotations.semantic_annotations)
)
all_classes |= set(
    classes_of_module(semantic_digital_twin.world_description.degree_of_freedom)
)
all_classes |= set(classes_of_module(semantic_digital_twin.robots.abstract_robot))
# classes |= set(recursive_subclasses(ViewFactory))
all_classes |= set([HasBody] + recursive_subclasses(HasBody))
all_classes |= set(classes_of_module(semantic_digital_twin.reasoning.predicates))
all_classes |= set(classes_of_module(semantic_digital_twin.semantic_annotations.mixins))
all_classes |= set(
    classes_of_module(semantic_digital_twin.adapters.procthor.procthor_resolver)
)
all_classes |= set(
    classes_of_module(semantic_digital_twin.world_description.world_modification)
)
all_classes |= set(classes_of_module(semantic_digital_twin.callbacks.callback))

# remove classes that should not be mapped
all_classes -= {
    ResetStateContextManager,
    WorldModelUpdateContextManager,
    HasUpdateState,
    ForwardKinematicsManager,
    WorldModelManager,
    semantic_digital_twin.adapters.procthor.procthor_resolver.ProcthorResolver,
    ContainsType,
}
# keep only dataclasses that are NOT AlternativeMapping subclasses
all_classes = {
    c for c in all_classes if is_dataclass(c) and not issubclass(c, AlternativeMapping)
}
all_classes |= {am.original_class() for am in recursive_subclasses(AlternativeMapping)}

alternative_mappings = [
    am
    for am in recursive_subclasses(AlternativeMapping)
    if am.original_class() in all_classes
]

def _patch_ormatic_interface_make_idempotent(file_path: str) -> None:
    """
    Patch the generated ormatic_interface.py to be import-idempotent.

    Some CI/pytest environments may cause the ORM module to be executed more than once
    in the same process. Clearing Base.metadata prevents duplicate Table registration
    errors in SQLAlchemy.
    """
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    marker = "class Base(DeclarativeBase):"
    if marker not in content:
        return

    # Insert directly after the Base class block header and its body start; easiest robust
    # approach: insert after the first occurrence of "class Base(DeclarativeBase):"
    # and after the next blank line following that class definition.
    # We do a simple, safe insertion after the first occurrence of "class Base..."
    # by inserting after the first double newline that follows it.
    idx = content.find(marker)
    if idx == -1:
        return

    # Only patch once
    patch_snippet = "\n# Make module import-idempotent (required for some test runners)\nBase.metadata.clear()\n"
    if "Base.metadata.clear()" in content:
        return

    # Find end of Base class definition by locating the first occurrence of "\n\n"
    # after the marker; this is stable in generated files.
    after_marker = content.find("\n\n", idx)
    if after_marker == -1:
        return

    new_content = content[: after_marker + 2] + patch_snippet + content[after_marker + 2 :]

    with open(file_path, "w", encoding="utf-8") as f:
        f.write(new_content)

def generate_orm():
    """
    Generate the ORM classes for the pycram package.
    """
    class_diagram = ClassDiagram(
        list(sorted(all_classes, key=lambda c: c.__name__, reverse=True))
    )

    instance = ORMatic(
        class_dependency_graph=class_diagram,
        type_mappings={
            trimesh.Trimesh: semantic_digital_twin.orm.model.TrimeshType,
            uuid.UUID: sqlalchemy.UUID,
        },
        alternative_mappings=alternative_mappings,
    )

    instance.make_all_tables()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    path = os.path.abspath(
        os.path.join(script_dir, "..", "src", "semantic_digital_twin", "orm")
    )
    with open(os.path.join(path, "ormatic_interface.py"), "w") as f:
        instance.to_sqlalchemy_file(f)
    # Make generated ORM module idempotent for test runners:
    # In some CI/pytest setups the module can be executed multiple times in the same
    # process, which would otherwise lead to "Table ... already defined" errors.
    ormatic_path = os.path.join(path, "ormatic_interface.py")
    _patch_ormatic_interface_make_idempotent(ormatic_path)

if __name__ == "__main__":
    generate_orm()
