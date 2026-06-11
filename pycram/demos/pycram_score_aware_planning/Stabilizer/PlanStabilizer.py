from dataclasses import dataclass


@dataclass(kw_only=True)
class PlanStabilizer:

    def __init__(self):
        pass

    """
    Notes:
        - Dependent on different method and situation there are different solutions
        - meaning, find out error 
        - then try to fix error
        - then go on
    """
    def stabilize(self, task):
        raise NotImplementedError

    def place_stabilize(self, task):
        raise NotImplementedError

    def pickup_stabilize(self, task):
        raise NotImplementedError