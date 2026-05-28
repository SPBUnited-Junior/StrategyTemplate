import math  # type: ignore
from time import time  # type: ignore
from typing import Optional
from enum import Enum

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates

class Robot_Status(Enum):
    Not_Kick = 0
    Pass_Straight = 1
    Pass_Turn_Kick= 2
    Goal_Turn_Kick = 3
    Goal_Straight = 4
    Kick_in_goal_hull = 5
    
kick_status: list[Enum] = [Robot_Status.Not_Kick] * 15
