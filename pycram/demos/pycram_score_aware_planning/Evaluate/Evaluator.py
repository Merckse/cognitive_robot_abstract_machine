from abc import ABC, abstractmethod
from typing import Optional

from demos.pycram_score_aware_planning.Evaluate.types import ActionType, ActionOutcome
from pycram.datastructures.enums import TaskStatus


class Evaluator(ABC):

    @abstractmethod
    def record(self,
        action_type: ActionType ,
        outcome: ActionOutcome | TaskStatus,
        object_name: Optional[str] = "",
        custom_points: Optional[int] = None,
        note: Optional[str] = None,):
        pass

    @abstractmethod
    def summary(self):
        pass