"""Command-line flags shared by every tutorial script.

    --headless      run without a window (servers, CI)
    --test          run a fixed number of steps, check the result, exit 0 (pass) / 1 (fail)
    --record NAME   save docs/assets/video/NAME.mp4 and docs/assets/img/NAME.png
"""

import argparse


def parse_args(description: str, extra=None):
    """Parse the common flags. ``extra(parser)`` may add tutorial-specific flags."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--headless", action="store_true", help="run without a window")
    parser.add_argument("--test", action="store_true", help="run a short automatic check and exit")
    parser.add_argument("--record", metavar="NAME", default=None, help="save a video + image for the docs")
    if extra is not None:
        extra(parser)
    # SimulationApp also reads sys.argv (Kit flags such as --/app/...); ignore what we don't know.
    args, _unknown = parser.parse_known_args()
    return args
