"""
Exceptions of the HSR demonstrations in this folder.
"""

from __future__ import annotations

from dataclasses import dataclass

from krrood.exceptions import DataclassException

# %% perception


@dataclass
class MilkNotSeen(DataclassException):
    """
    RoboKudo reported no milk the robot could pick up.
    """

    seen_categories: list[str]
    """
    The categories of everything RoboKudo reported instead.
    """

    def error_message(self) -> str:
        return (
            f"RoboKudo reported no milk with a pose and a size, only "
            f"{self.seen_categories or 'nothing'}."
        )

    def suggest_correction(self) -> str:
        return (
            "place the robot so that the milk is in view of its head camera, and check "
            "that the RoboKudo pipeline reports poses and sizes."
        )
