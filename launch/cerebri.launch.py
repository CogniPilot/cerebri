from launch import LaunchDescription
from launch.actions import Shutdown, ExecuteProcess
from ament_index_python.packages import get_package_prefix
from pathlib import Path


def generate_launch_description():
    cerebri_path = Path(get_package_prefix('cerebri'))
    cerebri_bin = cerebri_path / 'lib' / 'cerebri' / 'cerebri'
    cerebri_cmd = f"terminator -u -T cerebri --geometry=500x700+0+0 -e {cerebri_bin} 2>&1"
    return LaunchDescription([
        ExecuteProcess(
            cmd=cerebri_cmd.split(),
            output="log",
            shell=True),
    ])
