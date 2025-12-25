"""Configuration loading utilities for YAML config files."""

import yaml
from pathlib import Path
from typing import Dict, Any


def load_config(config_name: str) -> Dict[str, Any]:
    """Load YAML configuration file from config/ directory.

    Args:
        config_name: Name of config file without .yaml extension

    Returns:
        Dictionary containing configuration data

    Raises:
        FileNotFoundError: If config file doesn't exist
        yaml.YAMLError: If config file has invalid YAML syntax
    """
    config_path = Path(__file__).parents[3] / 'config' / f'{config_name}.yaml'

    if not config_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def get_camera_config() -> Dict[str, Any]:
    """Load camera calibration configuration.

    Returns:
        Camera calibration parameters including matrix and distortion coefficients
    """
    return load_config('camera_calibration')


def get_robot_config() -> Dict[str, Any]:
    """Load robot hardware parameters.

    Returns:
        Robot physical parameters and motor configuration
    """
    return load_config('robot_params')


def get_serial_config(task: str = 'task1') -> Dict[str, Any]:
    """Load serial communication configuration for specific task.

    Args:
        task: 'task1' or 'task2'

    Returns:
        Serial port configuration for the specified task
    """
    config = load_config('serial_config')
    return config[task]
