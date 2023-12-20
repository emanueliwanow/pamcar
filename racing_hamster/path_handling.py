"""
This module handles the path structure and should be use whenever something is saved or loaded from disc
"""
from pathlib import Path
import os
from typing import List
# ______ path handling ______
ROOT_DIR = Path(__file__).parent
ENVIRONMENT_DIR = ROOT_DIR / 'environments'
INPUT_TRAJECTORY_DIR = ROOT_DIR / 'input_trajectories'

# create folder structure of data if not available
_dirs = [ROOT_DIR, ENVIRONMENT_DIR, INPUT_TRAJECTORY_DIR]
for _dir in _dirs:
    if not _dir.exists():
        os.makedirs(_dir)


def get_file_names_in_dir(path: Path, remove_extension: bool = False) -> List[str]:
    """
    Get the names of all files in the specified directory.

    Args:
        path (Path): The path to the directory.
        remove_extension (bool): Whether to remove file extensions from the file names.

    Returns:
        list[str]: A list of file names in the directory.
    """
    if path.is_dir():
        files = [file.name if not remove_extension else file.stem for file in path.iterdir()]
        return files
    else:
        return []
