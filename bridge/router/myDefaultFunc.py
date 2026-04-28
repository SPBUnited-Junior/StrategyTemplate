from typing import Optional
from bridge.auxiliary import aux, fld, rbt  # type: ignore
from time import time  # type: ignore

import bridge.strategy.myConst as myConst

class myIsBallInClass():

    TimerWeHoldBall: Optional[float] = None
    rWhichHoldBall: Optional[rbt.Robot] = None

    def updateTimerWeHoldBall(self, field: fld.Field) -> None:
        if self.TimerWeHoldBall is None:
            for r in field.active_allies(True):
                if field.is_ball_in(r):
                    self.TimerWeHoldBall = time()
                    self.rWhichHoldBall = r
        elif all(not field.is_ball_in(r) for r in field.active_allies(True)):
            self.TimerWeHoldBall = None

    def myIsBallIn(self, robot: rbt.Robot) -> bool:
        return self.TimerWeHoldBall is not None and self.TimerWeHoldBall > myConst.timerForHoldBallForMyIsBallIn and self.rWhichHoldBall is not None and self.rWhichHoldBall.r_id == robot.r_id
    
