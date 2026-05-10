from enum import Enum


class ActionType(str, Enum):
    PICKUP          = "pickup"
    PLACE           = "place"
    OPEN            = "open"
    CLOSE           = "close"
    NAVIGATE        = "navigate"
    HAND_OVER       = "hand_over"
    POUR            = "pour"
    PUSH            = "push"
    DETECT          = "detect"
    CUSTOM          = "custom"

"""
Structurizer there to create a plan and sequence, as the building block,
between the probability and the possible score, to structurize all needed and known tasks.
"""
class Structurizer:

    def __init__(self):
        actionType : list[ActionType] = []

        pass