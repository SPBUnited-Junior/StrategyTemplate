import math  # type: ignore
from time import time  # type: ignore
from typing import Optional
from enum import Enum

from bridge import const
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from bridge.const import State as GameStates

class Kick_Status(Enum):
    Not_Kick = 0
    Pass_Straight = 1
    Pass_Turn_Kick= 2
    Goal_Turn_Kick = 3
    Goal_Straight = 4

class Kick_Status_Holder:
    def __init__(self) -> None:
        self.kick_status: Enum = Kick_Status.Not_Kick

    @property
    def value(self) -> Enum:
        return self.kick_status

    
    @value.setter
    def value(self, new_kick_status : Enum) -> None:
        self.kick_status = new_kick_status

kick_status = Kick_Status_Holder()