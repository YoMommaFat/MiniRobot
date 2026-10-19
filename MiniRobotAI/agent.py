import os
import json

# Remove ROS 2 UNIX timestamps from terminal output globally
os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{name}] [{severity}]: {message}"

# CRITICAL FIX: Monkeypatch LangChain BaseMessage to make .text() callable again.
# EZT NE TÖRÖLD KI! Enélkül a generálás végén a hri_connector elszáll TypeError-ral,
# így sosem küldi el a seq_end=True jelet a szájnak, ami emiatt extra 2 másodpercet vár a semmire!
from langchain_core.messages import BaseMessage
BaseMessage.text = lambda self: self.content

from langchain_core.runnables import RunnableConfig
from rai.agents import BaseAgent, wait_for_shutdown
from rai.agents.langchain import (
    HRICallbackHandler,
    ReActAgentState,
    create_react_runnable,
)

# CRITICAL FIX 2: Gemini néha listaként küldi a tokent (multimodal output), amit a régi 
# hri_connector Pydantic validátora nem bír feldolgozni (csak stringet vár).
original_on_llm_new_token = HRICallbackHandler.on_llm_new_token
def safe_on_llm_new_token(self, token, **kwargs):
    if isinstance(token, list):
        token = "".join([item.get('text', '') for item in token if isinstance(item, dict)])
    elif not isinstance(token, str):
        token = str(token)
    return original_on_llm_new_token(self, token, **kwargs)
HRICallbackHandler.on_llm_new_token = safe_on_llm_new_token
from rai.communication.ros2 import (
    ROS2Context,
    ROS2HRIConnector,
    ROS2HRIMessage,
)
from rai.messages.multimodal import SystemMultimodalMessage
from rclpy.logging import get_logger


class MiniRobotAgent(BaseAgent):
    def __init__(self):
        super().__init__()
        
        # ROS 2 communication bridge for text messages
        self.hri_connector = ROS2HRIConnector()

        # Load base system prompt from robot_prompt.json
        prompt_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_prompt.json")
        with open(prompt_path, "r", encoding="utf-8") as f:
            prompt_data = json.load(f)
        raw_prompt = prompt_data.get("system_prompt", "")
        prompt_text = "\n".join(raw_prompt) if isinstance(raw_prompt, list) else str(raw_prompt)

        system_prompt = SystemMultimodalMessage(content=prompt_text)

        # Initialize AI logic
        self.agent = create_react_runnable(
            tools=[],
            system_prompt=system_prompt,
        )
        
        self.state = ReActAgentState(messages=[])

        # Output topic: where the robot sends its answers
        self.callback_handler = HRICallbackHandler(
            connectors={"/to_human": self.hri_connector},
        )

        # Input topic: where the robot receives questions (recognized speech)
        self.hri_connector.register_callback("/from_human", self.input_callback)
        
        # Hardcoded publisher for THINKING state and TTS end signal
        import rclpy
        from std_msgs.msg import String
        from rai_interfaces.msg import HRIMessage
        self.emotion_node = rclpy.create_node('agent_emotion_pub')
        self.emotion_pub = self.emotion_node.create_publisher(String, '/cmd_face', 10)
        self.tts_pub = self.emotion_node.create_publisher(HRIMessage, '/to_human', 10)
        
        get_logger('agent').info("Agent node started successfully. Waiting for questions on /from_human!")

    def input_callback(self, message: ROS2HRIMessage):
        text = message.text.strip() if message.text else ""
        if not text:
            # Csak üres zajt hallott a fül (a Whisper nem talált benne beszédet)
            return
            
        get_logger('agent').info(f"USER: {text}")
        
        # Manually set face to THINKING while waiting for LLM
        from std_msgs.msg import String
        msg = String()
        msg.data = "THINKING"
        self.emotion_pub.publish(msg)
        
        self.state["messages"].append(message.to_langchain())
        # CRITICAL FIX: Menteni kell a visszatérő state-et, különben a robot elfelejti a saját válaszait!
        try:
            self.state = self.agent.invoke(
                self.state, config=RunnableConfig(callbacks=[self.callback_handler])
            )
        finally:
            # BIZTOSÍTÉK: Ha a régi callback handler elnyeli a stream végét, mi manuálisan 
            # ráküldjük a Szájra (tts_node), hogy vége a mondatnak, így azonnal becsukja a száját!
            from rai_interfaces.msg import HRIMessage
            end_msg = HRIMessage()
            end_msg.text = ""
            end_msg.seq_end = True
            self.tts_pub.publish(end_msg)

    def run(self):
        pass

    def stop(self):
        self.hri_connector.shutdown()


@ROS2Context()
def main():
    agent = MiniRobotAgent()
    wait_for_shutdown([agent])

if __name__ == "__main__":
    main()
