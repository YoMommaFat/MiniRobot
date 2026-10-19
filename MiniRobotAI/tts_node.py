import os
import json
import rclpy
import subprocess
import re
import threading
import queue
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rai_interfaces.msg import HRIMessage

# Remove ROS 2 UNIX timestamps from terminal output globally
os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{name}] [{severity}]: {message}"

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.json")
        with open(config_path, "r", encoding="utf-8") as f:
            tts_cfg = json.load(f).get("tts", {})

        self.piper_model = tts_cfg.get("piper_model", "hu_HU-berta-medium.onnx")
        self.audio_device = tts_cfg.get("audio_device", "pulse")
        self.sample_rate = str(tts_cfg.get("sample_rate", 22050))

        self.subscription = self.create_subscription(
            HRIMessage,
            '/to_human',
            self.listener_callback,
            10)
        self.speaking_pub = self.create_publisher(Bool, '/robot_speaking', 10)
        self.emotion_pub = self.create_publisher(String, '/cmd_face', 10)
        self.say_pub = self.create_publisher(String, '/say', 10)
        self.buffer = ""
        self.is_llm_finished = True # Track if the LLM has completed its stream
        
        self.get_logger().info(f'Loading Piper ONNX model ({self.piper_model}) into memory (RAM)...')
        # Persistent, continuously running Piper process
        self.piper_proc = subprocess.Popen(
            ['piper', '--model', self.piper_model, '--output_raw'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE
        )
        
        self.audio_queue = queue.Queue()
        
        # Reader thread: reads raw audio from Piper
        self.read_thread = threading.Thread(target=self.piper_reader, daemon=True)
        self.read_thread.start()
        
        # Player thread: dynamically opens/closes aplay
        self.play_thread = threading.Thread(target=self.audio_player, daemon=True)
        self.play_thread.start()

        self.get_logger().info('Local TTS (Piper) node started. Waiting for text...')

    def piper_reader(self):
        while True:
            chunk = self.piper_proc.stdout.read(4096)
            if chunk:
                self.audio_queue.put(chunk)
            else:
                break

    def audio_player(self):
        aplay_proc = None
        while True:
            try:
                # Wait for audio chunks (0.2s timeout)
                chunk = self.audio_queue.get(timeout=0.2)
                
                # If aplay is not running, start it (and mute the microphone)
                if aplay_proc is None:
                    msg = Bool()
                    msg.data = True
                    self.speaking_pub.publish(msg)
                    
                    aplay_proc = subprocess.Popen(
                        ['aplay', '-D', self.audio_device, '-r', self.sample_rate, '-f', 'S16_LE', '-t', 'raw', '-c', '1'],
                        stdin=subprocess.PIPE
                    )
                    
                aplay_proc.stdin.write(chunk)
                aplay_proc.stdin.flush()
                
            except queue.Empty:
                # If no data for 0.2s, sentence is finished.
                # Close aplay to release the audio card!
                if aplay_proc is not None:
                    aplay_proc.stdin.close()
                    aplay_proc.wait()
                    aplay_proc = None
                    
                    # we wait 2.0s mid-generation. BUT if we know the LLM is done (seq_end), we don't need to wait (0.0s).
                    try:
                        timeout_val = 0.0 if self.is_llm_finished else 2.0
                        if timeout_val > 0:
                            chunk = self.audio_queue.get(timeout=timeout_val)
                            # If we get here, more audio arrived! Just put it back and loop.
                            self.audio_queue.put(chunk)
                            continue
                    except queue.Empty:
                        pass
                        
                    # Timeout passed or 0.0s reached. The LLM has definitely finished answering.
                    msg = Bool()
                    msg.data = False
                    self.speaking_pub.publish(msg)
    def speak(self, text):
        # Notify the face immediately
        say_msg = String()
        say_msg.data = text
        self.say_pub.publish(say_msg)

        # Send text to the persistent Piper process in memory
        self.piper_proc.stdin.write((text + "\n").encode('utf-8'))
        self.piper_proc.stdin.flush()

    def listener_callback(self, msg):
        # Ha egy új válasz kezdődik, átállítjuk a státuszt
        if self.is_llm_finished and msg.text:
            self.is_llm_finished = False
            
        text = msg.text
        if not text and not msg.seq_end:
            return
            
        self.buffer += text
        
        # Extract and publish all complete emotion tags, remove them from buffer
        matches = list(re.finditer(r'\[([A-Z_]+)\]', self.buffer))
        for match in matches:
            emotion = match.group(1)
            emo_msg = String()
            emo_msg.data = emotion
            self.emotion_pub.publish(emo_msg)
            
        # Strip all complete tags from the buffer
        if matches:
            self.buffer = re.sub(r'\[[A-Z_]+\]\s*', '', self.buffer)
            
        # Check if there is an incomplete tag at the end of the buffer
        if '[' in self.buffer and ']' not in self.buffer[self.buffer.rfind('['):]:
            # Full tag has not arrived from stream yet, wait
            return
        
        # Check if there is a sentence-ending punctuation in the buffer
        has_punct = any(p in self.buffer for p in ['.', '!', '?', '\n'])
        
        if has_punct or msg.seq_end:
            if msg.seq_end:
                # Read out the remaining buffer at the very end
                speak_text = self.buffer.strip()
                self.buffer = ""
                self.is_llm_finished = True # A LLM hivatalosan befejezte a teljes választ!
            else:
                # Find a safe cut point (the last sentence-ending punctuation)
                last_punct = max(self.buffer.rfind(p) for p in ['.', '!', '?', '\n'])
                speak_text = self.buffer[:last_punct+1].strip()
                # Leave the remainder (the unfinished word) in the buffer for the next call
                self.buffer = self.buffer[last_punct+1:]
                
            if speak_text:
                self.get_logger().info(f'Readout: {speak_text}')
                self.speak(speak_text)

def main(args=None):
    rclpy.init(args=args)
    tts_node = TTSNode()
    try:
        rclpy.spin(tts_node)
    except KeyboardInterrupt:
        pass
    finally:
        tts_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
