import os
import shutil
from enum import Enum
from typing import Type, TypeVar

from dotenv import load_dotenv

USER_ENV_FILE = ".env"
DEFAULT_ENV_FILE = "environment/default.env"


def check_environment_existence() -> None:
    sync_file_with_template(DEFAULT_ENV_FILE, USER_ENV_FILE)


T = TypeVar("T", int, float, bool)
E = TypeVar("E", bound=Enum)


def get_from_env(name: str, needed_type: Type[T]) -> T:
    check_environment_existence()
    RED_BOLD = "\033[91m\033[1m"
    RESET = "\033[0m"

    load_dotenv(USER_ENV_FILE)
    value = os.getenv(name)
    if value is None:
        raise RuntimeError(f"{RED_BOLD}Environment variable '{name}' not found{RESET}")

    if needed_type == bool:
        return value.lower() == "true"

    if needed_type in [int, float]:
        return needed_type(value)

    raise RuntimeError(f"{RED_BOLD}Unsupported type: {needed_type}{RESET}")


def get_from_env_specific_type(name: str, specific_type: Type[E]) -> E:
    check_environment_existence()
    RED_BOLD = "\033[91m\033[1m"
    RESET = "\033[0m"

    load_dotenv(USER_ENV_FILE)
    value = os.getenv(name)
    if value is None:
        raise RuntimeError(f"{RED_BOLD}Environment variable '{name}' not found{RESET}")

    if issubclass(specific_type, Enum):
        if value not in specific_type.__members__:
            raise RuntimeError(f'{RED_BOLD}Invalid value "{value}" for {name}. ' f"Check .env file{RESET}")
        return specific_type[value]

    raise RuntimeError(f"{RED_BOLD}Unsupported type: {specific_type}{RESET}")


def get_first_line(path: str) -> str | None:
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return f.readline().strip()


def sync_file_with_template(template: str, target: str) -> None:
    if not os.path.exists(template):
        raise RuntimeError(f"Template not found: {template}")

    template_version = get_first_line(template)
    target_version = get_first_line(target)

    if target_version != template_version:
        shutil.copy(template, target)
