# deepdrive_ai

## Dependencies

- [yasmin](https://github.com/uleroboticsgroup/yasmin)
- [audio_common](https://github.com/mgonzs13/audio_common)
- [llama_ros](https://github.com/mgonzs13/llama_ros)
- [whiser_ros](https://github.com/mgonzs13/whisper_ros)
- [tts_ros](https://github.com/mgonzs13/tts_ros)

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mattwilliamson/deepdrive_ai
$ cd ~/ros2_ws
$ colcon build
```

## Usage

```shell
$ ros2 launch deepdrive_ai_bringup chatbot.launch.py
```
