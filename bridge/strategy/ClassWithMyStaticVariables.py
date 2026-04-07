from typing import Optional

from bridge.auxiliary import aux  # type: ignore
import bridge.strategy.myConst as myConst
from bridge.strategy.myConst import whatWeDoStates
from bridge.router.myDefaultFunc import myIsBallInClass  # type: ignore


class ClassWithMyStaticVariables:
    def __init__(self)->None:
        self.we_active: bool = False
        self.maxVelBall: float = 0
        self.idGettingPass: Optional[int] = None
        self.idDoPass: Optional[int] = None
        self.oldIdDoPass: Optional[int] = None
        self.GKLastState: Optional[str] = None

        self.idFirstAttacker: int = myConst.idFirstAttacker
        self.idSecondAttacker: int = myConst.idSecondAttacker

        self.TimeWeTryDoPass: Optional[float] = None
        self.TimerWeHoldBall: Optional[float] = None

        self.whatWeDoAtThisRun: whatWeDoStates = myConst.whatWeDoAtThisRun
        self.constForTimerWeTryDoPass: float = myConst.constForTimerWeTryDoPass

        self.PointFromBallKicked: Optional[aux.Point] = None
        self.AngleWithWhatBallKicked: float = 0.0

        self.myIsBallInClass = myIsBallInClass()