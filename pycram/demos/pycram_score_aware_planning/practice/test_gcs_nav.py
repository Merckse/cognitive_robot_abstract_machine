import itertools
import math

import rclpy

from demos.pycram_score_aware_planning.common.hsrb_testing import setup_world
from helper_methods import NAVIGATION_POSES
from semantic_digital_twin.spatial_types import Point3
from semantic_digital_twin.world_description.graph_of_convex_sets import (
    GraphOfConvexSets,
    PoseOccupiedError,
)

V_EFF = 0.15      # m/s, HSR cruising speed (~0.55 km/h)
T_FIXED = 5.0     # s, planning + rotate + settle overhead
ROBOT_RADIUS = 0.25
DETOUR_FALLBACK = 1.3  # euclidean multiplier when a pose is unreachable/occupied

rclpy.init()
world, dispatcher = setup_world()

gcs = GraphOfConvexSets.navigation_map_from_world(world, bloat_obstacles=ROBOT_RADIUS)
print(f"navigation map: {len(gcs.graph.nodes())} free-space boxes, "
      f"{len(gcs.graph.edges())} connections\n")


def to_point(location: str, z: float = 0.0) -> Point3:
    x, y = NAVIGATION_POSES[location][:2]
    return Point3(x, y, z)


def path_length(path: list[Point3]) -> float:
    return sum(float(p.euclidean_distance(q)) for p, q in zip(path, path[1:]))


def nav_time(distance: float) -> float:
    return T_FIXED + distance / V_EFF


locations = [loc for loc in NAVIGATION_POSES if loc]  # skip the "" fallback pose

# check every pose is actually inside the free space first
for loc in locations:
    node = gcs.node_of_point(to_point(loc))
    if node is None:
        print(f"!! approach pose of {loc!r} {NAVIGATION_POSES[loc][:2]} lies in an "
              f"occupied/bloated region -> will use euclidean fallback")
print()

header = f"{'from':15s} {'to':15s} {'eucl [m]':>9s} {'path [m]':>9s} {'ratio':>6s} {'t_nav [s]':>10s}"
print(header)
print("-" * len(header))

for a, b in itertools.combinations(locations, 2):
    pa, pb = to_point(a), to_point(b)
    eucl = math.dist((pa.x, pa.y), (pb.x, pb.y))
    note = ""
    try:
        path = gcs.path_from_to(pa, pb)
        dist = path_length(path) if path is not None else float("inf")
        if path is None:
            note = "  UNREACHABLE"
    except PoseOccupiedError:
        dist = DETOUR_FALLBACK * eucl
        note = "  (occupied -> euclidean fallback)"
    ratio = dist / eucl if eucl > 0 and math.isfinite(dist) else float("nan")
    print(f"{a:15s} {b:15s} {eucl:9.2f} {dist:9.2f} {ratio:6.2f} {nav_time(dist):10.1f}{note}")

# visual check: free-space decomposition should tile the walkable floor
gcs.plot_and_show_free_space()