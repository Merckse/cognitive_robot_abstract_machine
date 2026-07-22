
from ScoreAwareExecutioner import score_aware_execution
from demos.pycram_score_aware_planning.helper_methods import generic_object_spawner
from helper_methods import challenge_setup
from pycram.datastructures.dataclasses import Context

from demos.pycram_score_aware_planning.common.hsrb_testing import setup_world
from pycram.datastructures.enums import ChallengeMode
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.world_description.geometry import Color



def main():
    # HSRB specified local world setup

    challenge_mode: ChallengeMode = ChallengeMode.PP
    world, dispatcher, context= challenge_setup(challenge_mode)

    score_aware_execution(dispatcher, context, challenge_mode)

if __name__ == "__main__":
    main()