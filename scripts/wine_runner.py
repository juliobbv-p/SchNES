"""Run a Windows executable with paths translated through Wine's Z drive."""

import os
from pathlib import Path
import sys


def windows_arg(argument):
    return "Z:" + argument.replace("/", "\\") if Path(argument).is_absolute() else argument


if __name__ == "__main__":
    wine, executable, *arguments = sys.argv[1:]
    os.execv(wine, [wine, executable, *map(windows_arg, arguments)])
