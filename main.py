"""
Точка входа в стратегию
"""

from strategy_bridge.processors.base_processor import BaseProcessor
from strategy_bridge.runner import Runner

from bridge import const
from bridge.processors.drawing_processor import Drawer
from bridge.processors.field_creator import FieldCreator
from bridge.processors.python_controller import SSLController
from bridge.processors.router_processor import CommandSink
from environment.setup_environment import get_from_env

if __name__ == "__main__":

    PROCESSORS: list[BaseProcessor] = [
        FieldCreator(),
        SSLController(
            ally_color=const.COLOR,
        ),
        Drawer(),
        CommandSink(),
    ]

    TWO_TEAMS_PLAYING = get_from_env("TWO_TEAMS_PLAYING", bool)
    if TWO_TEAMS_PLAYING:
        PROCESSORS.append(
            SSLController(
                ally_color=const.COLOR.reverse(),
            ),
        )

    RUNNER = Runner(processors=PROCESSORS)
    RUNNER.run()
