# MiniRobot

Kompakt, differenciálhajtású oktató-, telekonferencia- és kutatórobot platform **ROS 2**, **RobotecAI (RAI)** multimodális nagy nyelvi modell (LLM) integrációval és **FreeRTOS** alapú beágyazott mikrokontroller vezérléssel.

![MiniRobot](MiniRobotAI/documentation/images/minirobot_exterior_render.png)

---

## Mappastruktúra

* **[`MiniRobot/`](MiniRobot/)** – **Arduino Mega (ATmega2560-16AU) Firmware:**
  FreeRTOS alapú valós idejű motorvezérlés, enkóder olvasás, hardveres szakadék- és ütközésvédelem (*Motor Assist*), ADC tápellátás-felügyelet és kétirányú JSON soros kommunikáció.
* **[`MiniRobotAI/`](MiniRobotAI/)** – **Raspberry Pi 5 / ROS 2 & RAI Ágens Rendszer:**
  * [`agent.py`](MiniRobotAI/agent.py) – RAI + LangChain ReAct központi LLM ágens kognitív állapotvezérléssel
  * [`robot_prompt.json`](MiniRobotAI/robot_prompt.json) – A robot személyiségét és kognitív állapotkódjait meghatározó rendszer-prompt
  * [`robot_config.json`](MiniRobotAI/robot_config.json) – A beszédfelismerő (ASR), beszédszintetizátor (TTS) és WebSocket kijelző paraméterei
  * [`asr_node.py`](MiniRobotAI/asr_node.py) – Beszédfelismerés (Silero VAD + Whisper ASR) automatikus némítással beszéd közben
  * [`tts_node.py`](MiniRobotAI/tts_node.py) – Alacsony késleltetésű, memóriában futó lokális magyar beszédszintézis (Piper ONNX)
  * [`face_node.py`](MiniRobotAI/face_node.py) – WebSocket szerver a robot érzelmi/kognitív állapotainak megjelenítéséhez
  * [`minirobot_bringup/`](MiniRobotAI/minirobot_bringup/) – ROS 2 hardveres híd (`arduino_bridge.py`) a soros JSON protokoll és a ROS 2 topikok között
  * [`web_ui/`](MiniRobotAI/web_ui/) – Kombinált interaktív robotarc, valós idejű szenzor-diagnosztikai műszerfal és távirányító felület
* **[`MiniRobotNR/`](MiniRobotNR/)** – Korábbi Node-RED alapú vezérlési flow-k mentése.

---

## Dokumentáció

A részletes műszaki leírások és útmutatók a [`MiniRobotAI/documentation/`](MiniRobotAI/documentation/) mappában találhatók:

1. **[`01_installation_guide.md`](MiniRobotAI/documentation/01_installation_guide.md) – Teljes Telepítési Útmutató**
   * **I. Rész:** Általános szoftverkörnyezet (Ubuntu 24.04, ROS 2, RAI keretrendszer, `.env` API kulcsok, Piper TTS) – bármely PC-n és Raspberry Pi-n használható.
   * **II. Rész:** MiniRobot-specifikus Raspberry Pi 5 hardverbeállítások (Waveshare WM8960 Audio HAT Kernel 6.8 javítás, 800×480 SPI érintőképernyő, GPIO UART, RDP/Samba).
2. **[`02_hardware_and_ros_architecture.md`](MiniRobotAI/documentation/02_hardware_and_ros_architecture.md) – Hardver, Mikrokontroller API és ROS 2 Híd Specifikáció**
   * FreeRTOS taskok, soros JSON protokoll, ADC áram- és feszültségmérő konverziós képletek, MegaCore AVR pinout, valamint az `arduino_bridge.py` és a Web UI architektúrája.
3. **[`03_ai_hri_and_research_background.md`](MiniRobotAI/documentation/03_ai_hri_and_research_background.md) – Elméleti Háttér, HRI Kognitív Állapotok és Kutatási Koncepció**
   * ROS 2 Embodied AI keretrendszerek összehasonlítása (RAI, ROSA, ROS-LLM, ROSGPT, TCC-IRoNL), Ekman alapérzelmek vs. HRI kognitív állapotok, valamint a VLSB (*Vision-Language-Social Behavior*) kutatási keretrendszer.
4. **[`04_daily_run_guide.md`](MiniRobotAI/documentation/04_daily_run_guide.md) – Napi Indítási és Használati Útmutató**
   * A 6 alappillér manuális indítása, illetve az egyparancsos `start_tmux.sh` háttérfolyamat-kezelés és kiosk kijelző indítása.
