# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from huggingface_hub import hf_hub_download
from launch.conditions import IfCondition


def generate_launch_description():

    def run_whisper(context: LaunchContext, repo, file):
        repo = str(context.perform_substitution(repo))
        file = str(context.perform_substitution(file))

        # struct whisper_full_params {
        #     enum whisper_sampling_strategy strategy;

        #     int n_threads;
        #     int n_max_text_ctx;     // max tokens to use from past text as prompt for the decoder
        #     int offset_ms;          // start offset in ms
        #     int duration_ms;        // audio duration to process in ms

        #     bool translate;
        #     bool no_context;        // do not use past transcription (if any) as initial prompt for the decoder
        #     bool no_timestamps;     // do not generate timestamps
        #     bool single_segment;    // force single segment output (useful for streaming)
        #     bool print_special;     // print special tokens (e.g. <SOT>, <EOT>, <BEG>, etc.)
        #     bool print_progress;    // print progress information
        #     bool print_realtime;    // print results from within whisper.cpp (avoid it, use callback instead)
        #     bool print_timestamps;  // print timestamps for each text segment when printing realtime

        #     // [EXPERIMENTAL] token-level timestamps
        #     bool  token_timestamps; // enable token-level timestamps
        #     float thold_pt;         // timestamp token probability threshold (~0.01)
        #     float thold_ptsum;      // timestamp token sum probability threshold (~0.01)
        #     int   max_len;          // max segment length in characters
        #     bool  split_on_word;    // split on word rather than on token (when used with max_len)
        #     int   max_tokens;       // max tokens per segment (0 = no limit)

        #     // [EXPERIMENTAL] speed-up techniques
        #     // note: these can significantly reduce the quality of the output
        #     bool speed_up;          // speed-up the audio by 2x using Phase Vocoder
        #     bool debug_mode;        // enable debug_mode provides extra info (eg. Dump log_mel)
        #     int  audio_ctx;         // overwrite the audio context size (0 = use default)

        #     // [EXPERIMENTAL] [TDRZ] tinydiarize
        #     bool tdrz_enable;       // enable tinydiarize speaker turn detection

        #     // A regular expression that matches tokens to suppress
        #     const char * suppress_regex;

        #     // tokens to provide to the whisper decoder as initial prompt
        #     // these are prepended to any existing text context from a previous call
        #     // use whisper_tokenize() to convert text to tokens
        #     // maximum of whisper_n_text_ctx()/2 tokens are used (typically 224)
        #     const char * initial_prompt;
        #     const whisper_token * prompt_tokens;
        #     int prompt_n_tokens;

        #     // for auto-detection, set to nullptr, "" or "auto"
        #     const char * language;
        #     bool detect_language;

        #     // common decoding parameters:
        #     bool suppress_blank;    // ref: https://github.com/openai/whisper/blob/f82bc59f5ea234d4b97fb2860842ed38519f7e65/whisper/decoding.py#L89
        #     bool suppress_non_speech_tokens; // ref: https://github.com/openai/whisper/blob/7858aa9c08d98f75575035ecd6481f462d66ca27/whisper/tokenizer.py#L224-L253

        #     float temperature;      // initial decoding temperature, ref: https://ai.stackexchange.com/a/32478
        #     float max_initial_ts;   // ref: https://github.com/openai/whisper/blob/f82bc59f5ea234d4b97fb2860842ed38519f7e65/whisper/decoding.py#L97
        #     float length_penalty;   // ref: https://github.com/openai/whisper/blob/f82bc59f5ea234d4b97fb2860842ed38519f7e65/whisper/transcribe.py#L267

        #     // fallback parameters
        #     // ref: https://github.com/openai/whisper/blob/f82bc59f5ea234d4b97fb2860842ed38519f7e65/whisper/transcribe.py#L274-L278
        #     float temperature_inc;
        #     float entropy_thold;    // similar to OpenAI's "compression_ratio_threshold"
        #     float logprob_thold;
        #     float no_speech_thold;  // TODO: not implemented

        #     struct {
        #         int best_of;    // ref: https://github.com/openai/whisper/blob/f82bc59f5ea234d4b97fb2860842ed38519f7e65/whisper/transcribe.py#L264
        #     } greedy;

        #     struct {
        #         int beam_size;  // ref: https://github.com/openai/whisper/blob/f82bc59f5ea234d4b97fb2860842ed38519f7e65/whisper/transcribe.py#L265

        #         float patience; // TODO: not implemented, ref: https://arxiv.org/pdf/2204.05424.pdf
        #     } beam_search;


        return Node(
            package="whisper_ros",
            executable="whisper_node",
            name="whisper_node",
            namespace="whisper",
            parameters=[{
                "sampling_strategy": LaunchConfiguration("sampling_strategy", default="beam_search"),
                "model": LaunchConfiguration("model", default=hf_hub_download(repo_id=repo, filename=file, force_download=False)),
                "openvino_encode_device": LaunchConfiguration("openvino_encode_device", default="GPU"),

                "n_threads": LaunchConfiguration("n_threads", default=4),
                "n_max_text_ctx": LaunchConfiguration("n_max_text_ctx", default=16384),
                "offset_ms": LaunchConfiguration("offset_ms", default=0),
                "duration_ms": LaunchConfiguration("duration_ms", default=0),

                "translate": LaunchConfiguration("translate", default=False),
                "no_context": LaunchConfiguration("no_context", default=False),
                # "no_context": LaunchConfiguration("no_context", default=True),
                "no_timestamps": LaunchConfiguration("no_timestamps", default=False),
                "single_segment": LaunchConfiguration("single_segment", default=True),
                "print_special": LaunchConfiguration("print_special", default=False),
                "print_progress": LaunchConfiguration("print_progress", default=False),
                "print_realtime": LaunchConfiguration("print_realtime", default=False),
                "print_timestamps": LaunchConfiguration("print_timestamps", default=False),

                "token_timestamps": LaunchConfiguration("token_timestamps", default=False),
                "thold_pt": LaunchConfiguration("thold_pt", default=0.01),
                "thold_ptsum": LaunchConfiguration("thold_ptsum", default=0.01),
                "max_len": LaunchConfiguration("max_len", default=0),
                "split_on_word": LaunchConfiguration("split_on_word", default=False),
                "max_tokens": LaunchConfiguration("max_tokens", default=0),

                "speed_up": LaunchConfiguration("speed_up", default=False),
                "audio_ctx": LaunchConfiguration("audio_ctx", default=0),
                "tinydiarize": LaunchConfiguration("tinydiarize", default=False), # // enable tinydiarize speaker turn detection

                "language": LaunchConfiguration("language", default="en"),
                "detect_language": LaunchConfiguration("detect_language", default=False),

                "suppress_blank": LaunchConfiguration("suppress_blank", default=True),
                "suppress_non_speech_tokens": LaunchConfiguration("suppress_non_speech_tokens", default=True),

                "temperature": LaunchConfiguration("temperature", default=0.00),
                "max_initial_ts": LaunchConfiguration("max_initial_ts", default=1.00),
                "length_penalty": LaunchConfiguration("length_penalty", default=-1.00),

                "temperature_inc": LaunchConfiguration("temperature_inc", default=0.40),
                "entropy_thold": LaunchConfiguration("entropy_thold", default=2.40),
                "logprob_thold": LaunchConfiguration("logprob_thold", default=-1.00),
                "no_speech_thold": LaunchConfiguration("no_speech_thold", default=0.60),

                "greedy_best_of": LaunchConfiguration("greedy_best_of", default=5),
                "beam_search_beam_size": LaunchConfiguration("beam_search_beam_size", default=5),
                "beam_search_patience": LaunchConfiguration("beam_search_patience", default=-1.00),

                "n_processors": LaunchConfiguration("n_processors", default=1),
                "use_gpu": LaunchConfiguration("use_gpu", default=True),
                "gpu_device": LaunchConfiguration("gpu_device", default=0),
            }]
        ),

    model_repo = LaunchConfiguration("model_repo")
    model_repo_cmd = DeclareLaunchArgument(
        "model_repo",
        default_value="distil-whisper/distil-large-v3-ggml",
        description="Hugging Face model repo")

    model_filename = LaunchConfiguration("model_filename")
    model_filename_cmd = DeclareLaunchArgument(
        "model_filename",
        default_value="ggml-distil-large-v3.bin",
        description="Hugging Face model filename")

    return LaunchDescription([
        model_repo_cmd,
        model_filename_cmd,
        OpaqueFunction(function=run_whisper, args=[
                       model_repo, model_filename]),

        Node(
            package="whisper_ros",
            executable="silero_vad_node",
            name="silero_vad_node",
            namespace="whisper",
            parameters=[{
                "enabled": LaunchConfiguration("enabled", default=False),
            }],
            remappings=[("audio", "/audio/in")]
        ),

        Node(
            package="whisper_ros",
            executable="whisper_manager_node",
            name="whisper_manager_node",
            namespace="whisper",
        ),

        Node(
            package="audio_common",
            executable="audio_capturer_node",
            name="capturer_node",
            namespace="audio",
            parameters=[{
                "format": LaunchConfiguration("channels", default=1),
                "channels": LaunchConfiguration("channels", default=1),
                "rate": LaunchConfiguration("rate", default=16000),
                "chunk": LaunchConfiguration("chunk", default=4096*2),
            }],
            remappings=[("audio", "in")],
            condition=IfCondition(PythonExpression(
                [LaunchConfiguration("launch_audio_capturer", default=True)]))
        )
    ])
