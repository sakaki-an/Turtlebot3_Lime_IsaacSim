"""Paths to the assets of this repository (works from any working directory)."""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

# Robot description (generated from the xacro by scripts/gen_urdf.sh; meshes resolve relative to it)
LIME_URDF = REPO_ROOT / "assets" / "urdf" / "turtlebot3_lime.urdf"

# Output folder of the URDF importer (tutorial 02 writes here; the importer adds a subfolder per robot)
USD_DIR = REPO_ROOT / "assets" / "usd"

# Robot USD exactly as the URDF importer produced it (tutorial 02)
LIME_IMPORTED_USD = USD_DIR / "turtlebot3_lime" / "turtlebot3_lime.usda"

# Tuned robot USD (tutorial 03) - the one all later tutorials use
LIME_USD = USD_DIR / "turtlebot3_lime_tuned.usda"

# Scenes
SCENE_DIR = REPO_ROOT / "assets" / "scenes"
LIME_ROOM_USD = SCENE_DIR / "lime_room.usda"

# Where --record writes media (used by the documentation pages)
MEDIA_DIR = REPO_ROOT / "docs" / "assets" / "video"
IMG_DIR = REPO_ROOT / "docs" / "assets" / "img"
