# MiniRobot – Teljes Telepítési Útmutató (Szoftverkörnyezet és Raspberry Pi Hardver)

Ez a dokumentum lépésről lépésre bemutatja a MiniRobot szoftveres ökoszisztémájának (ROS 2 Jazzy, RobotecAI RAI keretrendszer, LLM/ASR/TTS modulok) telepítését, valamint a fizikai roboton található egyedi Raspberry Pi 5 hardverelemek (Waveshare WM8960 hangkártya, 800×480 SPI érintőképernyő, GPIO UART, RDP/Samba) beállítását.

> **FONTOS – Különbség a normál PC/Laptop és a fizikai MiniRobot (Raspberry Pi) telepítése között:**
> A dokumentum **két fő részre** oszlik:
> * **I. RÉSZ – Általános Szoftverkörnyezet (Minden gépen kötelező):** A ROS 2, a RAI keretrendszer, a Python virtuális környezet (`rai_venv`), az API kulcsok (`.env`) és a lokális Piper TTS telepítése. Ha egy **asztali PC-n vagy laptopon** (Ubuntu 24.04 alatt) szeretnéd fejleszteni vagy tesztelni az AI ágenst és a Web UI-t, **kizárólag az I. részt kell elvégezned**, mivel a PC-n a hangkártya és a kijelző gyárilag (plug-and-play) működik!
> * **II. RÉSZ – MiniRobot Hardver-specifikus Beállítások (Csak a robot Raspberry Pi-jén szükséges):** Az egyedi I2S hangkártya (WM8960 Audio HAT) kernelmoduljának javítása és fordítása, az SPI érintőképernyő, a fizikai UART soros port (Arduino kapcsolat) és a headless távoli elérés (RDP/Samba) konfigurálása.

---

# I. RÉSZ: Általános Szoftverkörnyezet (PC, Laptop és Raspberry Pi)

**Célkörnyezet:** Ubuntu 24.04 LTS (Noble Numbat, 64-bit) és **ROS 2 Jazzy Jalisco**.

---

## 1. ROS 2 Jazzy és Függőségek Telepítése

Ha még nincs fent a gépen a ROS 2 Jazzy, telepítsd az alábbi parancsokkal:

```bash
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-jazzy-desktop ros-jazzy-rosbridge-suite -y
```

### Python és multimédia rendszercsomagok telepítése
Az arc, a képfeldolgozás és a hangkezelés futtatásához szükséges rendszercsomagok:
```bash
sudo apt update
sudo apt install python3-venv python3-pygame python3-numpy python3-opencv ffmpeg libportaudio2 -y
```

### Gyakori hibák teszteléskor
* **"event not found":** Ha a `/say` topikba küldesz üzenetet terminálból és felkiáltójelet használsz, tegyél elé egy visszaperjelet: `data: 'Szia\!'`.
* **Késleltetés:** A `ros2 topic pub --once` parancs 1-2 másodpercig tart, amíg felépíti a kapcsolatot. A végleges kódban (folyamatos kapcsolatnál) ez ezredmásodpercekre csökken.

---

## 2. RAI (Robot Agent Interface) Keretrendszer Telepítése

Ez a fejezet a RobotecAI által fejlesztett RAI keretrendszer telepítését írja le ROS 2 Jazzy környezethez optimalizálva, a hivatalos telepítő buktatóit (Poetry és `venv` izolációs problémák) kikerülve.

### 2.1 A Telepítési Koncepció (Miért így csináljuk?)

* **A `venv` és az Ubuntu 24.04 problémája:** A modern Ubuntu verziók biztonsági okokból tiltják a globális `pip install` használatát. Bár ezt meg lehetne kerülni a `--break-system-packages` kapcsolóval, a legtisztább megoldás egy virtuális környezet (`venv`) használata. Azonban a ROS 2 beépített Python csomagjait (pl. `rclpy`) a `venv` alapból nem látja. Ezt a `venv` létrehozásakor a `--system-site-packages` kapcsolóval kell orvosolnunk.
* **A Poetry probléma (Miért térünk el a hivatalos doksitól?):** A RAI hivatalos dokumentációja a "Developer Environment" telepítéséhez a **Poetry** nevű csomagkezelőt javasolja. Azonban a Poetry agresszíven létrehoz egy saját, 100%-ig szigetelt virtuális környezetet, amely elvágja a ROS 2 kapcsolatot. Ehelyett mi **letöltjük a forráskódot (`git clone`), de a telepítéshez a beépített `pip`-et használjuk (`pip install -e .`)** a saját, jól beállított ROS-kompatibilis `venv`-ünkön belül.

### 2.2 Lépésről Lépésre Telepítés

#### 1. A virtuális környezet (`rai_venv`) létrehozása
A home könyvtárban hozzuk létre, így független marad a kód mappájától. **Előtte kötelező betölteni a ROS 2-t!**
```bash
cd ~
source /opt/ros/jazzy/setup.bash
python3 -m venv rai_venv --system-site-packages
```

#### 2. A `venv` aktiválása
Ezt minden alkalommal ki kell adni, ha új terminált nyitsz (hacsak nem tetted be a `.bashrc`-be a 6. fejezet alapján):
```bash
source ~/rai_venv/bin/activate
```
*(Ha sikerült, a parancssor elején megjelenik a `(rai_venv)` felirat.)*

#### 3. A forráskód letöltése és telepítése (`rai-core`)
Töltsük le a RAI fejlesztői kódját és telepítsük szerkeszthető módban:
```bash
cd ~
git clone https://github.com/RobotecAI/rai.git
cd rai
pip install -e .
```

#### 4. Konfiguráció generálása
Generáljuk le a RAI alapértelmezett beállítási fájljait:
```bash
rai-config-init
```

#### 5. ROS 2 RAI interfészek telepítése
Ahhoz, hogy a RAI hivatalos üzenettípusait (`HRIMessage`) a rendszer felismerje a ROS hálózaton:
```bash
sudo apt install ros-jazzy-rai-interfaces -y
```

---

## 3. Beszédértés (ASR) – `rai_s2s` modul és függőségi hibák javítása

A mikrofonkezelés, a Whisper ASR és a VAD (hangérzékelés) a `rai_s2s` modulban található, amit külön kell telepíteni a forráskódból, továbbá a WebSocket alapú kijelzőhöz és a soros kommunikációhoz szükséges csomagokat is felrakjuk:
```bash
source ~/rai_venv/bin/activate
pip install -e ~/rai/src/rai_s2s
pip install websockets pyserial faster-whisper
```

> **FONTOS FIX (`numpy` és `scipy` verziók):** A `rai_s2s` telepítése felülírja a `numpy` és a `scipy` csomagokat egy újabb (2.0+) verzióra, amely összezavarja a ROS 2 / `rai-core` futását. Ezt a telepítés után **kötelező** visszaléptetni a `venv`-en belül:
> ```bash
> pip install "numpy<2"
> pip install scipy==1.14.0
> ```

---

## 4. API Kulcsok (`.env`) és a `config.toml` beállítása

A `config.toml` fájlban **TILOS** az `api_key` paraméter megadása, mert az sémavalidációs hibát (`Pydantic ValidationError`) okoz. Ehelyett a kulcsokat környezeti változóként, a `.env` fájlból töltjük be.

1. Lépj be a projekt `MiniRobotAI` (a Pi-n `~/minirobot`) mappájába, és másold le a minta `.env.example` fájlt:
   ```bash
   cd ~/minirobot
   cp .env.example .env
   nano .env
   ```
2. Írd be a saját Google Gemini (LLM) és Groq (felhős Whisper ASR) kulcsaidat:
   ```env
   GOOGLE_API_KEY="AIzaSy..."
   OPENAI_API_KEY="gsk_..."
   OPENAI_BASE_URL="https://api.groq.com/openai/v1"
   ```
3. A `config.toml`-ben a modellt ingyenes Google AI Studio fiók esetén érdemes a gyors Flash modellre állítani:
   ```toml
   [google]
   simple_model = "gemini-3-flash-preview"
   complex_model = "gemini-3-flash-preview"
   embeddings_model = "text-embedding-004"
   ```

---

## 5. Lokális TTS (Piper) Telepítése

Mivel a RAI beépített TTS-e fizetős felhőt (ElevenLabs) használ, egy saját, késleltetésmentes lokális Python csomóponttal (`tts_node.py`) váltjuk ki, amely a memóriában (RAM) tartja a magyar neurális hangmodellt.

1. Telepítsd a neurális TTS motort a `rai_venv`-be:
   ```bash
   source ~/rai_venv/bin/activate
   pip install piper-tts
   ```
2. Töltsd le a magyar nyelvi modellt (`berta` vagy `anna`) a futtatási mappába (`~/minirobot`):
   ```bash
   cd ~/minirobot
   wget https://huggingface.co/rhasspy/piper-voices/resolve/main/hu/hu_HU/berta/medium/hu_HU-berta-medium.onnx
   wget https://huggingface.co/rhasspy/piper-voices/resolve/main/hu/hu_HU/berta/medium/hu_HU-berta-medium.onnx.json
   ```
   *(Tipp: Az ALSA `Device Busy` hibák elkerüléséhez a `tts_node.py` a lejátszást a PulseAudio-ra bízza: `aplay -D pulse`).*

---

## 6. Automatikus környezetbetöltés (`~/.bashrc`)

Ahhoz, hogy minden terminál megnyitásakor automatikusan betöltődjön a ROS 2 Jazzy és a RAI virtuális környezete (`rai_venv`), add hozzá az alábbi sorokat a `~/.bashrc` fájl legvégéhez:

```bash
# ROS 2 Jazzy és a RAI venv betöltése
source /opt/ros/jazzy/setup.bash
source ~/rai_venv/bin/activate
```
Ezt egyetlen paranccsal is hozzáfűzheted:
```bash
echo -e "\nsource /opt/ros/jazzy/setup.bash\nsource ~/rai_venv/bin/activate" >> ~/.bashrc
```

---

# II. RÉSZ: MiniRobot Hardver-specifikus Beállítások (Kizárólag Raspberry Pi 5-höz!)

> **MEGJEGYZÉS – Más PC-n / laptopon ez a rész kihagyható!**
> Az alábbi fejezetek kifejezetten a fizikai MiniRobotba épített **Raspberry Pi 5** fedélzeti számítógéphez, annak egyedi **Waveshare WM8960 Audio HAT** hangkártyájához, a **800×480-as SPI érintőképernyőjéhez** és a **GPIO UART** soros portjához tartoznak. Egy hagyományos PC-n vagy laptopon az operációs rendszer saját hangkártya- és kijelzőkezelése alapból működik.

---

## 7. Operációs Rendszer, Távoli Elérés (RDP), Auto-login és Samba

### 7.1 Operációs Rendszer Telepítése a Pi-re
1. Használd a **Raspberry Pi Imager** szoftvert.
2. Válaszd ki az `Other general-purpose OS` -> `Ubuntu` -> `Ubuntu 24.04 LTS (64-bit)` opciót.
3. A **Beállításoknál** (fogaskerék): Gépnév: `robot`, Felhasználó: `robot`.

### 7.2 Auto-login és Kulcstartó (Keyring) feloldása RDP-hez
Ahhoz, hogy a robot bekapcsolás után monitor és billentyűzet nélkül is azonnal elérhető legyen Távoli Asztallal (RDP), két dolgot kell beállítani:

#### A. Auto-login bekapcsolása
1. Nyisd meg: `sudo nano /etc/gdm3/custom.conf`
2. A `[daemon]` szekcióban:
   ```ini
   AutomaticLoginEnable = true
   AutomaticLogin = robot
   ```

#### B. Kulcstartó (Keyring) felszabadítása (LÉNYEGES!)
Az Auto-login miatt a kulcstartó lezárva marad, ami minden újraindítás után megváltoztatja/blokkolja az RDP jelszót.
1. Nyisd meg a Pi grafikus felületén: **Passwords and Keys** (Seahorse).
2. Jobb klikk a **Login** kulcstartóra -> **Change Password**.
3. Add meg a régi jelszót, az **újat pedig hagyd üresen**. Hagyd jóvá a figyelmeztetést.
4. Ezután a **Settings > System > Remote Desktop** menüben a lakat eltűnik, és a távoli elérés újraindítás után is stabil marad.

### 7.3 Fájlmegosztás és Szinkronizáció (Samba)
Hogy a fejlesztői Windows PC és a Pi között ne kelljen kézzel fájlokat másolgatni, csatoljuk fel a Pi-t hálózati meghajtóként.

#### Beállítás a Pi-n:
1. Telepítés: `sudo apt update && sudo apt install samba -y`
2. Jelszó beállítása a hálózathoz: `sudo smbpasswd -a robot`
3. Tűzfal engedélyezése (ha aktív az `ufw`): `sudo ufw allow samba`
4. Konfiguráció: `sudo nano /etc/samba/smb.conf` – add hozzá a legvégéhez:
   ```ini
   [robot]
      path = /home/robot
      read only = no
      writable = yes
      force user = robot
   ```
5. Újraindítás: `sudo systemctl restart smbd`

#### Beállítás Windowson:
1. **Ez a gép** -> Jobb klikk -> **Hálózati meghajtó csatlakoztatása**.
2. Mappa: `\\Pi_IP_címe\robot` (pl. `\\192.168.1.9\robot`)
3. Felhasználó: `robot`, Jelszó: amit az `smbpasswd` parancsnál megadtál.

---

## 8. Robot Kijelző és Érintőképernyő (800×480 HDMI + XPT2046 SPI)

### 8.1 Boot Konfiguráció
Szerkeszd a `/boot/firmware/config.txt` fájlt (`sudo nano /boot/firmware/config.txt`):
```ini
# Felbontás (800x480)
hdmi_force_hotplug=1
hdmi_group=2
hdmi_mode=87
hdmi_cvt=800 480 60 6 0 0 0

# Driverek
dtparam=spi=on
dtparam=i2c_arm=on
dtparam=i2s=on
dtoverlay=ads7846,cs=1,penirq=25,penirq_pull=2,speed=50000,keep_vref_on=0
dtoverlay=wm8960-soundcard
dtparam=audio=off
```

### 8.2 Érintés kalibrálása
Fájl: `sudo nano /etc/udev/rules.d/99-touchscreen.rules`
```bash
ENV{ID_INPUT_TOUCHSCREEN}=="1", ENV{LIBINPUT_CALIBRATION_MATRIX}="1 0 0 0 1 0"
```

---

## 9. Waveshare WM8960 Audio HAT Hangkártya Telepítése (Raspberry Pi 5 + Ubuntu 24.04)

**Környezet:** Raspberry Pi 5, Ubuntu 24.04 LTS (Kernel 6.8), Waveshare WM8960 Audio HAT.
*(Más PC-n nincs szükség erre a fejezetre!)*

Mivel az Ubuntu 24.04 gyári overlay-e nem tudja kezelni a Pi 5 órajelét (MCLK), a gyártó kódját kell használnunk és manuálisan javítanunk a 6.8-as kernelhez.

### 9.1 Előkészületek és a Driver letöltése
Telepítsd a fordításhoz szükséges eszközöket és a kernel fejléceket, majd töltsd le a Waveshare repót:
```bash
sudo apt update
sudo apt install -y git dkms build-essential linux-headers-raspi
cd ~
git clone https://github.com/waveshareteam/WM8960-Audio-HAT
cd WM8960-Audio-HAT
```

### 9.2 A C-kód Manuális Javítása (Kernel 6.8 Kompatibilitás)
A letöltött kód alapból nem fordul le a 6.8-as kernelen (`DKMS bad exit status 2`). Két hibát kell kézzel javítani a `wm8960-soundcard.c` fájlban:
```bash
nano wm8960-soundcard.c
```

**1. Hiba (Redundáns deklaráció):**
Keresd meg a 42. sor környékét, és töröld ki vagy kommenteld ki ezt a részt:
```c
// static inline void snd_soc_dai_link_set_capabilities(struct snd_soc_dai_link *link)
// {
//     /* Empty implementation for older kernels */
// }
```

**2. Hiba (Típuseltérés a `.remove` callbacknél):**
Keresd meg a `simple_util_remove` függvényt, és írd át `void`-ról `int`-re, majd adj hozzá egy `return 0;`-t:
```c
static int simple_util_remove(struct platform_device *pdev)
{
    // ... meglévő kód ...
    return 0;
}
```
Lentebb a `platform_driver` struktúrában írd át a `.remove`-ot `.remove_new`-ra:
```c
    .probe = simple_util_probe,
    .remove_new = simple_util_remove,
};
```
Mentsd el a fájlt (`Ctrl+O`, `Enter`, `Ctrl+X`).

### 9.3 Telepítés futtatása és Boot konfiguráció
Hozz létre szimbolikus linket az Ubuntu-s konfigurációs útvonalhoz, majd indítsd el a fordítást:
```bash
sudo ln -s /boot/firmware/config.txt /boot/config.txt
sudo ./install.sh
```
*(Ha a kódjavítást jól csináltad, a szkript hiba nélkül lefut, és a végén kiírja: `Enjoy!`)*

Ellenőrizd a `/boot/firmware/config.txt` fájlban, hogy az alaplapi hang le legyen tiltva (`dtparam=audio=off`), különben összeakad az I2S órajellel:
```ini
dtparam=i2c_arm=on
dtparam=i2s=on
dtoverlay=wm8960-soundcard
dtparam=audio=off
```

### 9.4 Teljes Áramtalanítás (Hard Reset)
A telepítés után **NEM ELÉG** a szoftveres újraindítás (`reboot`)! A WM8960 chip hajlamos beragadni egy hardveres némított vagy mono állapotba.
1. Állítsd le a Pi-t: `sudo shutdown now`
2. **Kapcsold le / húzd ki a teljes tápot 10 másodpercre.**
3. Kapcsold vissza és indítsd el.

### 9.5 Némítás Feloldása (`alsamixer`) és Teszt
1. Futtasd: `alsamixer` -> **F6** -> válaszd a `wm8960-soundcard` eszközt.
2. A nyilakkal keresd meg a **"Left Output Mixer PCM"** és **"Right Output Mixer PCM"** oszlopokat.
3. Nyomj rájuk **M** betűt, hogy `MM` helyett `OO` (zöld, feloldott) legyen.
4. Húzd fel a **Speaker** és **Headphone** hangerőt.
5. Mentsd el véglegesen:
   ```bash
   sudo alsactl store
   ```
6. Futtasd a sztereó tesztet:
   ```bash
   speaker-test -c 2 -t wav -l 1
   ```

---

## 10. Raspberry Pi Hardveres UART (Soros Port) Konfiguráció az Arduinóhoz

A MiniRobot mikrovezérlőjének `USART2` (JSON kommunikáció) portja közvetlenül a Raspberry Pi **GPIO 14 (TXD)** és **GPIO 15 (RXD)** lábaira van kötve. Hogy ez működjön Ubuntu alatt, az alábbi beállításokat kell elvégezni:

### 10.1 A Hardveres UART bekapcsolása
1. Futtasd: `sudo nano /boot/firmware/config.txt`
2. Add hozzá a sort: `enable_uart=1`

### 10.2 A Linux bejelentkezési konzol (Login Shell) letiltása
A Linux gyárilag erre a soros portra küldi ki a bootolási logokat, ami felborítja az Arduino JSON kommunikációját.
1. Telepítsd és indítsd el a `raspi-config` eszközt:
   ```bash
   sudo apt install raspi-config -y
   sudo raspi-config
   ```
2. Lépj az **Interface Options -> Serial Port** menübe:
   * *Would you like a login shell to be accessible over serial?* -> **NO**
   * *Would you like the serial port hardware to be enabled?* -> **YES**
   *(Manuális módszer: a `/boot/firmware/cmdline.txt` fájlból töröld ki a `console=serial0,115200` részt).*
3. Indítsd újra a gépet!

### 10.3 Jogosultságok megadása (`dialout` csoport)
Ubuntun a soros portokat (`/dev/ttyAMA0` / `/dev/serial0`) alapból csak `root` használhatja. Hogy a `robot` felhasználó is hozzáférjen `sudo` nélkül:
```bash
sudo usermod -aG dialout $USER
```
*(Az érvényesítéshez ki-, majd újra be kell jelentkezni!)*
