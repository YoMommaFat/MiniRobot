#!/bin/bash

# Ellenőrizzük, hogy a tmux telepítve van-e
if ! command -v tmux > /dev/null 2>&1; then
    echo "Hiba: a 'tmux' nincs telepítve!"
    echo "Futtasd le ezt a Pi-n: sudo apt update && sudo apt install tmux"
    exit 1
fi

SESSION="minirobot"

# Ha már fut egy ilyen nevű session, kilőjük
tmux kill-session -t $SESSION 2>/dev/null

# Új session indítása a háttérben
tmux new-session -d -s $SESSION

# Alap parancs, ami betölti a környezetet és a .env fájlt
INIT_CMD="source /opt/ros/jazzy/setup.bash && source ~/rai_venv/bin/activate && cd ~/minirobot && set -a && source .env && set +a"

# Képernyő felosztása 4 részre (2x2 mátrix)
tmux split-window -h -t $SESSION:0
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.2

# 1. Panel (Bal fent) - AGY
tmux send-keys -t $SESSION:0.0 "$INIT_CMD" C-m
tmux send-keys -t $SESSION:0.0 "clear" C-m
tmux send-keys -t $SESSION:0.0 "python3 agent.py" C-m

# 2. Panel (Bal lent) - TTS
tmux send-keys -t $SESSION:0.1 "$INIT_CMD" C-m
tmux send-keys -t $SESSION:0.1 "clear" C-m
tmux send-keys -t $SESSION:0.1 "python3 tts_node.py" C-m

# 3. Panel (Jobb fent) - ASR
tmux send-keys -t $SESSION:0.2 "$INIT_CMD" C-m
tmux send-keys -t $SESSION:0.2 "clear" C-m
tmux send-keys -t $SESSION:0.2 "python3 asr_node.py" C-m

# 4. Panel (Jobb lent) - ARC
tmux send-keys -t $SESSION:0.3 "$INIT_CMD" C-m
tmux send-keys -t $SESSION:0.3 "clear" C-m
tmux send-keys -t $SESSION:0.3 "python3 face_node.py" C-m

# 5. Ablak (Új fül) - Webes Kijelző (Chromium)
tmux new-window -t $SESSION:1 -n "WebUI"
# Várunk 2 másodpercet, hogy a face_node biztosan elinduljon a 8765-ös porton
#tmux send-keys -t $SESSION:1 "sleep 2 && chromium-browser --app=file:///home/robot/minirobot/web_ui/index.html --kiosk" C-m
tmux send-keys -t $SESSION:1 "sleep 2 && chromium-browser --app=file:///home/robot/minirobot/web_ui/index.html" C-m

# 6. Ablak (Új fül) - ROS Híd és Arduino Kapcsolat
tmux new-window -t $SESSION:2 -n "ROS_Bridge"
tmux split-window -h -t $SESSION:2
# Bal panel: rosbridge_server (hivatalos ROS 2 WebSocket API)
tmux send-keys -t $SESSION:2.0 "source /opt/ros/jazzy/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m
# Jobb panel: arduino_bridge.py (Hardver kommunikáció)
tmux send-keys -t $SESSION:2.1 "source /opt/ros/jazzy/setup.bash && cd ~/minirobot && python3 minirobot_bringup/minirobot_bringup/arduino_bridge.py" C-m

# Jobb lent panel: kamera (camera_node.py)
tmux split-window -v -t $SESSION:2.1
tmux send-keys -t $SESSION:2.2 "source /opt/ros/jazzy/setup.bash && cd ~/minirobot && python3 camera_node.py" C-m

# Visszaváltunk az első (0.) ablakra, ahol a 4 log panel van
tmux select-window -t $SESSION:0

# Csatlakozás a sessionhöz, hogy rögtön lássuk az eredményt
tmux attach -t $SESSION
