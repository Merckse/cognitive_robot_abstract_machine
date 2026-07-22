from __future__ import annotations
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional
from uuid import UUID

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from semantic_digital_twin.adapters.ros.msg_converter import SemDTToRos2Converter
from semantic_digital_twin.adapters.ros.tf_publisher import TFPublisher
from semantic_digital_twin.callbacks.callback import ModelChangeCallback

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ....world import World


class ShapeSource(Enum):
    """
    Enum to specify which shapes to use for visualization.
    """

    VISUAL_ONLY = "visual_only"
    """
    The shapes to use for visualization are visual shapes only.
    """

    COLLISION_ONLY = "collision_only"
    """
    The shapes to use for visualization are collision shapes only.
    """

    VISUAL_WITH_COLLISION_BACKUP = "visual_with_collision_backup"
    """
    The shapes to use for visualization are visual shapes, but if there are no visual shapes, use collision shapes as a backup.
    """


@dataclass(eq=False)
class VizMarkerPublisher(ModelChangeCallback):
    """
    Publishes the world model as a visualization marker.
    .. warning:: Relies on the tf tree to correctly position the markers.
        Use TFPublisher to publish the tf tree.
    .. warning:: To see something in Rviz you must:
        1. add a MarkerArray plugin,
        2. set the current topic name,
        3. set DurabilityPolicy.TRANSIENT_LOCAL,
        4. make sure that the fixed frame is the tf root.
    """

    node: Node = field(kw_only=True)
    """
    The ROS2 node that will be used to publish the visualization marker.
    """

    topic_name: str = "/semworld/viz_marker"
    """
    The name of the topic to which the Visualization Marker should be published.
    """

    shape_source: ShapeSource = field(
        kw_only=True, default=ShapeSource.VISUAL_WITH_COLLISION_BACKUP
    )
    """
    Which shapes to use for each body
    """

    alpha: float = field(kw_only=True, default=1.0)
    """
    Marker transparency in [0.0, 1.0]. 0.0 is fully transparent.
    """

    show_labels: bool = field(kw_only=True, default=False)
    """
    Whether to publish a text label above every body that has an IsPerceivable semantic annotation.
    """

    label_text_height: float = field(kw_only=True, default=0.08)
    """
    Height of the label text in meters.
    """

    markers: MarkerArray = field(init=False, default_factory=MarkerArray)
    """Maker message to be published."""
    qos_profile: QoSProfile = field(
        default_factory=lambda: QoSProfile(
            depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
    )
    """QoS profile for the publisher."""

    def __post_init__(self):
        super().__post_init__()

        self.pub = self.node.create_publisher(
            MarkerArray, self.topic_name, self.qos_profile
        )
        time.sleep(0.2)
        self.notify()
        time.sleep(0.2)

    def with_tf_publisher(self):
        """
        Launches a tf publisher in conjunction with the VizMarkerPublisher.
        """
        TFPublisher(_world=self._world, node=self.node)

    def _select_shapes(self, body):
        if self.shape_source is ShapeSource.VISUAL_ONLY:
            return body.visual.shapes
        if self.shape_source is ShapeSource.COLLISION_ONLY:
            return body.collision.shapes
        if self.shape_source is ShapeSource.VISUAL_WITH_COLLISION_BACKUP:
            return body.visual.shapes if body.visual.shapes else body.collision.shapes
        raise ValueError(f"Unsupported shape_source: {self.shape_source!r}")

    def _notify(self, **kwargs):
        self.markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        self.markers.markers.append(delete_all)
        labeled_bodies = self._labeled_bodies() if self.show_labels else set()
        for body in self._world.bodies:
            shapes = self._select_shapes(body)
            if not shapes:
                continue
            marker_ns = str(body.name)
            for i, shape in enumerate(shapes):
                marker = SemDTToRos2Converter.convert(shape)
                if not marker.mesh_use_embedded_materials:
                    marker.color.a = self.alpha
                marker.frame_locked = True
                marker.id = i
                marker.ns = marker_ns
                self.markers.markers.append(marker)
            if body in labeled_bodies:
                self.markers.markers.append(self._label_marker(body, marker_ns))
        self.pub.publish(self.markers)

    def _labeled_bodies(self) -> set:
        """
        Collect the root bodies of all IsPerceivable annotations, i.e. the bodies that should get a name tag.
        """
        # local import to keep the adapter importable without the annotations module
        from semantic_digital_twin.semantic_annotations.mixins import IsPerceivable

        bodies = set()
        for annotation in self._world.get_semantic_annotations_by_type(IsPerceivable):
            root = getattr(annotation, "root", None)
            if root is not None:
                bodies.add(root)
        return bodies

    def _label_marker(self, body, marker_ns: str) -> Marker:
        """
        Build a camera-facing text marker floating above the body, in the body's own tf frame so it follows the body.
        """
        label = Marker()
        label.header.frame_id = str(body.name)
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.text = body.name.name
        # separate namespace so all labels can be toggled independently of the bodies in Rviz
        label.ns = marker_ns + "/label"
        label.id = 0
        label.pose.orientation.w = 1.0
        label.pose.position.z = self._label_z_offset(body)
        label.scale.z = self.label_text_height
        label.color.r = label.color.g = label.color.b = 1.0
        label.color.a = 1.0
        label.frame_locked = True
        return label

    def _label_z_offset(self, body) -> float:
        """
        Place the label slightly above the top of the body's collision geometry, with a fallback for bodies
        whose bounding box cannot be computed.
        """
        try:
            bb = body.collision.as_bounding_box_collection_in_frame(body).bounding_box()
            return bb.max_z + 0.05
        except Exception:
            return 0.25
