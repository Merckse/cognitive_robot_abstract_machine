
from ScoreAwareExecutioner import score_aware_execution
from StaticExecution import static_execution
from helper_methods import challenge_setup

from pycram.datastructures.enums import ChallengeMode



def main():
    challenge_mode: ChallengeMode = ChallengeMode.PP
    world, dispatcher, context= challenge_setup(challenge_mode)
    score_aware_execution(dispatcher, context, challenge_mode, enable_exploration=True, enabled_repair_operators=False, )



if __name__ == "__main__":
    main()