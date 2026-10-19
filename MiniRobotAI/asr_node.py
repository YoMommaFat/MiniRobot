import os
import json
import time
import rclpy
import threading
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import Bool
from rai.communication.ros2 import ROS2Context
from rai_s2s.asr.agents.asr_agent import SpeechRecognitionAgent
from rai_s2s.asr.models import SileroVAD
from rai_s2s.asr.models.open_ai_whisper import OpenAIWhisper
from rai_s2s.sound_device import SoundDeviceConfig

# Remove ROS 2 UNIX timestamps from terminal output globally
os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{name}] [{severity}]: {message}"


# VAD Wrapper that can block voice detection
class MuteableVAD:
    def __init__(self, real_vad):
        self.real_vad = real_vad
        self.is_muted = False
        self._was_speech = False
    
    def __call__(self, audio_data, input_parameters):
        if self.is_muted:
            if self._was_speech:
                get_logger('asr_node').info("Robot started speaking, recording aborted.")
                self._was_speech = False
            return False, {}
            
        is_speech, info = self.real_vad(audio_data, input_parameters)
        
        # State change detection for visual feedback
        if is_speech and not self._was_speech:
            get_logger('asr_node').info("Voice detected! Continuous recording started...")
            self._was_speech = True
        elif not is_speech and self._was_speech:
            self._was_speech = False
            
        return is_speech, info
        
    def reset(self):
        self._was_speech = False
        self.real_vad.reset()

# ROS 2 Node listening to TTS "speaking" signals
class MuteListenerNode(Node):
    def __init__(self, muteable_vad):
        super().__init__('asr_mute_listener')
        self.muteable_vad = muteable_vad
        self.sub = self.create_subscription(Bool, '/robot_speaking', self.listener_callback, 10)
        
    def listener_callback(self, msg):
        self.muteable_vad.is_muted = msg.data
        status = "MUTED (Robot speaking)" if msg.data else "LISTENING (Robot finished)"
        self.get_logger().info(f"Microphone status: {status}")

@ROS2Context()
def main():
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.json")
    with open(config_path, "r", encoding="utf-8") as f:
        asr_cfg = json.load(f).get("asr", {})

    sample_rate = asr_cfg.get("sample_rate", 16000)
    model_name = asr_cfg.get("model_name", "whisper-large-v3")
    language = asr_cfg.get("language", "hu")
    vad_threshold = asr_cfg.get("vad_threshold", 0.4)
    grace_period = asr_cfg.get("grace_period", 1.5)
    device_name = asr_cfg.get("device_name", "default")

    mic_config = SoundDeviceConfig(
        stream=True,
        channels=1,
        device_name=device_name,
        block_size=1280,
        consumer_sampling_rate=sample_rate,
        dtype="int16",
        device_number=None,
        is_input=True,
        is_output=False,
    )
    
    # CLOUD GROQ API (Lightning fast)
    get_logger('asr_node').info(f"Connecting to Groq API ({model_name}, lang={language})...")
    transcription_model = OpenAIWhisper(model_name=model_name, sample_rate=sample_rate, language=language)
    
    vad_model = SileroVAD(sample_rate, vad_threshold)
    
    muteable_vad = MuteableVAD(vad_model)

    agent = SpeechRecognitionAgent(
        microphone_config=mic_config,
        ros2_name="minirobot_asr",
        transcription_model=transcription_model,
        vad=muteable_vad,
        grace_period=grace_period
    )
    
    # Start the mute listener Node on a separate thread
    mute_node = MuteListenerNode(muteable_vad)
    spin_thread = threading.Thread(target=rclpy.spin, args=(mute_node,))
    spin_thread.daemon = True
    spin_thread.start()
    
    get_logger('asr_node').info("Ears armed! Waiting for speech...")
    
    try:
        agent.run() # Note: this is a non-blocking call, it only starts background threads!
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        get_logger('asr_node').info("Ctrl+C detected. Force killing all threads...")
        os._exit(0)

if __name__ == '__main__':
    main()
