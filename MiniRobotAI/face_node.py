import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import asyncio
import websockets
import json
import threading
import os

# Remove ROS 2 UNIX timestamps from terminal output globally
os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{name}] [{severity}]: {message}"

class WebFaceNode(Node):
    def __init__(self):
        super().__init__('face_node')
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_config.json")
        with open(config_path, "r", encoding="utf-8") as f:
            face_cfg = json.load(f).get("face", {})

        self.ws_host = face_cfg.get("ws_host", "0.0.0.0")
        self.ws_port = face_cfg.get("ws_port", 8765)

        self.emotion_sub = self.create_subscription(String, '/cmd_face', self.emotion_callback, 10)
        self.speaking_sub = self.create_subscription(Bool, '/robot_speaking', self.speaking_callback, 10)
        
        # Store connected browsers
        self.ws_clients = set()
        
        # The WebSocket server runs on a separate thread to not block ROS 2
        self.ws_thread = threading.Thread(target=self.start_ws_server, daemon=True)
        self.ws_thread.start()
        
        self.get_logger().info(f"WebFace Node started! Waiting for Chromium browser connection on port {self.ws_port}...")
        
    def emotion_callback(self, msg):
        self.get_logger().info(f"Emotion command received: {msg.data}")
        self.broadcast({"type": "emotion", "value": msg.data})

    def speaking_callback(self, msg):
        # Sent by TTS. True: Started speaking, False: Finished
        self.broadcast({"type": "speaking", "value": msg.data})
        
    def broadcast(self, data):
        # Send message to the browser (in JSON format)
        message = json.dumps(data)
        if not self.ws_clients:
            return
            
        async def send_to_all():
            disconnected = set()
            for client in self.ws_clients:
                try:
                    await client.send(message)
                except Exception:
                    disconnected.add(client)
            
            # Remove closed connections
            for client in disconnected:
                self.ws_clients.remove(client)
                
        # Sending must be scheduled in the asyncio event loop
        if hasattr(self, 'loop') and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(send_to_all(), self.loop)
            
    async def ws_handler(self, websocket):
        self.get_logger().info("Browser display connected successfully!")
        self.ws_clients.add(websocket)
        try:
            # Keep the connection alive
            async for message in websocket:
                pass
        except Exception:
            pass
        finally:
            self.ws_clients.remove(websocket)
            self.get_logger().info("Browser display disconnected from server.")

    async def _run_ws(self):
        self.loop = asyncio.get_running_loop()
        async with websockets.serve(self.ws_handler, self.ws_host, self.ws_port):
            await asyncio.Future()  # Infinite loop
            
    def start_ws_server(self):
        asyncio.run(self._run_ws())

def main(args=None):
    rclpy.init(args=args)
    node = WebFaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
