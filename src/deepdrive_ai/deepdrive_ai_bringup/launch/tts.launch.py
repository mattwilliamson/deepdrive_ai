# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")
    coqui_agreed_envvar = SetEnvironmentVariable("COQUI_AGREED", "1")

    tts_node_cmd = Node(
        package="tts_ros",
        executable="tts_node",
        output="both",
        parameters=[{
            "device": "cuda",
            "model": "tts_models/multilingual/multi-dataset/xtts_v2",
            "chunk": 4096,
            "speaker_wav": os.path.join(get_package_share_directory("deepdrive_ai"), "audio", "scarjo.wav"),
            # "speaker": "Maja Ruoho",
            "stream": LaunchConfiguration("stream", default=True),

        }],
        remappings=[("audio", "/audio/out")],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(coqui_agreed_envvar)

    ld.add_action(tts_node_cmd)

    return ld
