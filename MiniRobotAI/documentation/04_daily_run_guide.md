# MiniRobot – Rendszer Indítási Útmutató (Napi használat)

Ez a leírás összefoglalja, hogyan kell a nulláról, a gép bekapcsolása után elindítani a teljes mesterséges intelligencia rendszert (a ROS 2 hálózatot, az LLM Agyat, a TTS Szájat, az ASR Fület, a Grafikus Arcot és a Hardveres Hidat).

---

## 1. Előkészületek

1. **Csatlakozás a géphez:** Lépj be a Raspberry Pi-re Távoli Asztal (RDP) segítségével, vagy nyiss SSH kapcsolatokat a Windows terminálból.
2. **Környezet (VENV), API kulcsok (`.env`) és Mappa:** Első használat előtt másold le a példa konfigurációt (`cp .env.example .env`), és írd bele a saját API kulcsaidat. A gép bekapcsolása után minden újonnan megnyitott terminál ablakban **egy az egyben** futtasd le az alábbi blokkot! Ez betölti a ROS 2-t, a virtuális környezetet, belép a kódok mappájába, és felolvassa a `.env` fájlból az API kulcsokat:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/rai_venv/bin/activate
   cd ~/minirobot
   set -a && source .env && set +a
   ```

---

## 2. A Rendszer Indítása (A 6 pillér)

A moduláris ROS 2 hálózat miatt a robot „szervei” különálló terminál ablakokban futnak. **Nyiss 6 terminált, és indítsd el őket az alábbi sorrendben:**

### 1. Terminál: Az Agy (LLM)
Ez a script kapcsolódik a felhőhöz, és ez irányítja a logikát.
```bash
python3 agent.py
```
*(Várd meg, amíg kiírja, hogy `Agent node started successfully. Waiting for questions on /from_human!`).*

### 2. Terminál: A Száj (Piper TTS)
Ez a lokális neurális hangszintetizátor, ami a kapott szövegeket Berta hangján, késleltetés nélkül felolvassa, és egy némító jelet is küld a mikrofonnak, hogy a robot ne hallja a saját hangját.
```bash
python3 tts_node.py
```
*(Várd meg a `Local TTS (Piper) node started. Waiting for text...` üzenetet.)*

### 3. Terminál: A Fülek (Whisper ASR + Silero VAD)
Ez a modul figyeli a mikrofont. A szüneteket (csendet) hangérzékelő (VAD) figyeli, és elküldi a levágott hangokat a felhős/gyorsított Whisper AI-nak leiratozásra.
```bash
python3 asr_node.py
```
*(Amint megjelenik az `Ears armed! Waiting for speech...` felirat, a rendszer hangalapú része üzemkész.)*

### 4. Terminál: Az Arc (Backend)
Ez a Python modul fogadja az agytól az érzelemkódokat és vezérli a webes kijelzőt a 8765-ös WebSocket porton.
```bash
python3 face_node.py
```

### 5. Terminál: A ROS Webes Híd (`rosbridge`)
Ez a hivatalos szerver nyitja meg a ROS 2 hálózatot (a 9090-es WebSocket porton) a webes Műszerfal számára, hogy megjeleníthesse az adatokat és küldhesse a mozgásparancsokat.
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 6. Terminál: A Hardver Híd (Arduino)
Ez a script kommunikál a fizikai robottal a soros porton, és kiküldi a szenzoradatokat a ROS hálózatba.
```bash
python3 minirobot_bringup/minirobot_bringup/arduino_bridge.py
```

---

## 3. Használat

Amint mind a 6 terminál fut:
1. Csak kezdj el beszélni a mikrofonba magyarul.
2. Ha befejezted a mondatot, tarts egy nagyon pici szünetet.
3. A 3. terminálban látni fogod, hogy felismerte a hangot, majd az 1. terminálban az AI generálja a választ.
4. Mialatt a robot beszél, az arc mozgatja a száját, a 3. terminál pedig kiírja, hogy `MUTED (Robot speaking)`, hogy a mikrofon ne hallja a saját hangját.
5. Ha benyomod a fizikai ütközőket, vagy felemeled a robotot, a Webes Műszerfalon látni fogod a vizuális visszajelzést.

---

## 4. Leállítás
Ha végrehajtottad a napi teszteket, mind a hat terminálban a **`Ctrl + C`** megnyomásával állíthatod le a rendszereket. Ha valamelyik folyamat beragadna, használd a **`Ctrl + Z`**, majd a `kill -9 %1` parancsokat.

---

## 5. Automatizált indítás és Haladó parancsok

Ha nem szeretnél kézzel megnyitni 6 terminált minden alkalommal, az alábbi parancsokkal automatizálhatod a folyamatokat.

### A teljes rendszer automatikus indítása (háttérben)
A moduláris ROS 2 hálózat (Agy, Száj, Fül, Arc, ROS Híd, Arduino Híd) elindítható egyetlen szkripttel a háttérben, a `tmux` terminál multiplexer segítségével:
```bash
cd ~/minirobot
./start_tmux.sh
```
*(Ha bele akarsz nézni a futó háttérfolyamatokba, használd a `tmux attach -t minirobot` parancsot).*

**Navigáció a `tmux` fülek (ablakok) között:**
A szkript 3 fület hoz létre (`0`: AI, `1`: WebUI, `2`: ROS_Bridge). Váltani köztük a **`Ctrl+b`** (egyszerre lenyomva, majd elengedve), utána pedig a **`0`**, **`1`**, vagy **`2`** gomb megnyomásával tudsz.

### A Kijelző (Arc) Megjelenítése Teljes Képernyőn
Mivel az arc már webalapú, a Chromium böngészővel kell megnyitni kiosk (teljes képernyős, keret nélküli) módban. Futtasd egy új terminálban vagy parancsikonból:
```bash
chromium-browser --app=file:///home/robot/minirobot/web_ui/index.html --kiosk
```
*(Kilépés a teljes képernyős Chromiumból: `Alt + F4`).*

### Rendszer Kikapcsolása (`tmux kill`)
Ha a robot „szervei” a háttérben (`tmux`) futnak, a teljes MiniRobot backend bezárásához és leállításához ezt a parancsot kell használnod:
```bash
tmux kill-session -t minirobot
```
Ezután a robot teljesen leáll, a kamera, a mikrofon és a soros portok felszabadulnak.

### Erőforrások és CPU Figyelése
A processzor (CPU) és a memória (RAM) valós idejű terhelését a beépített parancsokkal monitorozhatod egy új terminál ablakban:
```bash
htop
```
*(Kilépés: `q` vagy `F10`).*
*Tipp: Ha telepítve van a rendszereden, a `btop` parancs sokkal szebb, grafikus hardver-monitorozó felületet ad.*
