import datetime
import random
import tempfile
from dataclasses import is_dataclass, dataclass
from typing import Type, List

import tqdm

import pycram.orm.ormatic_interface  # type: ignore
import semantic_digital_twin  # type: ignore
import experiments.orm.ormatic_interface
from experiments.experiment_definitions import (
    ExperimentResult,
    ExperimentsTable,
    TypstRenderer,
)
from krrood.class_diagrams import ClassDiagram
from krrood.ormatic.data_access_objects.alternative_mappings import AlternativeMapping
from krrood.ormatic.helper import get_classes_of_ormatic_interface
from krrood.ormatic.ormatic import ORMatic
from krrood.ormatic.type_dict import TypeDict
from krrood.utils import recursive_subclasses


# import classes from the existing interface
classes, alternative_mappings, type_mappings = get_classes_of_ormatic_interface(
    pycram.orm.ormatic_interface
)
classes = set(classes)

alternative_mappings += [am for am in recursive_subclasses(AlternativeMapping)]
alternative_mappings = list(set(alternative_mappings))
# keep only dataclasses that are NOT AlternativeMapping subclasses
classes = {
    c for c in classes if is_dataclass(c) and not issubclass(c, AlternativeMapping)
}
classes |= {am.original_class() for am in recursive_subclasses(AlternativeMapping)}

alternative_mappings = [
    am
    for am in recursive_subclasses(AlternativeMapping)
    if am.original_class() in classes
]


@dataclass
class ORMaticScalabilityExperimentResult(ExperimentResult):
    total_duration: datetime.timedelta
    class_diagram_creation_duration: datetime.timedelta
    ormatic_reasoning_duration: datetime.timedelta
    writing_to_file_duration: datetime.timedelta
    number_of_classes: int
    number_of_associations: int
    number_of_inheritances: int


def ormatic_scalability_experiment(
    classes: List[Type], class_drop_probability: float = 0.3
) -> ORMaticScalabilityExperimentResult:
    """
    Run an ORMatic scalability experiment.
    This is done by collecting all classes from CRAM and then dropping a random subset of them.
    With the remaining classes, we create a new ORMatic interface and generate the ORM classes and measure the timings.

    :param classes:
    :param class_drop_probability:
    :return:
    """

    # kick out random classes
    filtered_classes = {
        c for c in classes if random.uniform(0, 1) > class_drop_probability
    }

    filtered_classes |= {am.original_class() for am in alternative_mappings}

    begin = datetime.datetime.now()

    # create the new ormatic interface
    class_diagram = ClassDiagram(
        list(sorted(filtered_classes, key=lambda c: c.__name__, reverse=True))
    )

    class_diagram_creation_time = datetime.datetime.now()

    # Create an ORMatic object with the classes to be mapped
    ormatic = ORMatic(
        class_diagram,
        type_mappings=TypeDict(type_mappings),
        alternative_mappings=alternative_mappings,
    )

    # Generate the ORM classes
    ormatic.make_all_tables()

    ormatic_reasoning_time = datetime.datetime.now()

    with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".py") as f:
        temp_path = f.name
        ormatic.to_sqlalchemy_file(f)

    writing_to_file_time = datetime.datetime.now()

    return ORMaticScalabilityExperimentResult(
        total_duration=writing_to_file_time - begin,
        class_diagram_creation_duration=class_diagram_creation_time - begin,
        ormatic_reasoning_duration=ormatic_reasoning_time - class_diagram_creation_time,
        writing_to_file_duration=writing_to_file_time - ormatic_reasoning_time,
        number_of_classes=len(filtered_classes),
        number_of_associations=len(class_diagram.associations),
        number_of_inheritances=len(class_diagram.inheritance_relations),
    )


def main():
    results = []
    for class_drop_probability in tqdm.tqdm(
        [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    ):
        results.append(ormatic_scalability_experiment(classes, class_drop_probability))

    table = ExperimentsTable(results)
    print(TypstRenderer(table).render_table())


if __name__ == "__main__":
    main()
