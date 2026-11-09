# MiniRobot – Odamenés egy tárgyhoz (`nod_towards`) és kamera node

Ez az ág két dolgot ad a rendszerhez:

1. **`nod_towards` eszköz az AI ágensnek.** Ha azt mondod a robotnak, hogy *„Menj oda a piros labdához!”*, a kamera képe alapján a tárgy felé fordul, és tesz felé néhány rövid lépést (összesen kb. 15 cm-t). Ha valami túl közel van előtte, a távolságmérők megállítják.
2. **Kamera node (`camera_node.py`).** A kamera képeit ROS topikon küldi; ezeket olvassa az eszköz. A repóban eddig nem volt ilyen node.

Az ágens modellje továbbra is a `config.toml`-ból jön (alapból Gemini), az eszköz bármilyen képet látó modellel működik.

---

## Mi változott

| Fájl | Változás |
|---|---|
| `nod_tool.py` | **Új.** A `nod_towards` eszköz. |
| `camera_node.py` | **Új.** A kamera képei: `/camera/image_raw` (nyers, bgr8) és `/camera/image_raw/compressed` (JPEG). |
| `check_emotion_tags.py` | **Új.** Mérőszkript: mennyire tartja be egy helyi (Ollama) modell az állapotkódokat. A robothoz nem kell. |
| `agent.py` | Bekötöttük az eszközt. Ha a válaszban nincs érvényes állapotkód, az arc READY-re vált (különben THINKING-en ragadna). Az előzményekből csak az utolsó 6 kérdés–válasz marad. |
| `robot_prompt.json` | Két új sor: mikor hívja az eszközt, és milyen kóddal kezdje utána a választ. |
| `robot_config.json` | Új `camera` szakasz (a beállítások magyarázata a `_..._info` mezőkben). |
| `start_tmux.sh` | A kamera node a ROS_Bridge ablak jobb alsó paneljében indul. |
| `config.toml` | *(Külön commit.)* `gemini-3.8-flash` és `gemini-embedding-2` – lásd lent. Ha nem kell, ez az egy commit visszavonható. |

---

## Hogyan működik az odamenés

1. Az arc **OBSERVING**-re vált.
2. Az eszköz lefényképezi a látványt, és megkérdezi a képelemző modellt (`config.toml` → `simple_model`), hol van a tárgy: balra, középen, jobbra, vagy nincs a képen.
3. Ha oldalt van, a robot helyben fordul felé (először 20°-ot), és újra megnézi. Ha túlfordult, feleannyit fordul vissza. Legfeljebb 4 fordulás.
4. Ha a tárgy középen van, **3 × 5 cm-t** megy előre. Minden lépés előtt friss távolságmérést vár, és megáll, ha valami túl közel van.
5. A végén az arc **SUCCESS**, **SORRY** vagy **ALERT** lesz, és az eszköz megmondja a modellnek, melyik kóddal kezdje a válaszát.

A robot **nem mozdul**, ha nem jön kép a kamerából, vagy a távolságmérők nem küldenek adatot (ALERT). Ha nem látja a tárgyat, nem megy közelebb (SORRY).

---

## Első indítás előtt

### 1. Kamera

```bash
rpicam-hello --list-cameras
```

- **Listázza a kamerát:** minden kész, a node így működik.
- **„command not found”:** Pi 5-ön Ubuntu 24.04 alatt a Raspberry Pi saját kameraszoftverét (libcamera + rpicam-apps) forrásból kell lefordítani, az apt-os csomag nem támogatja a Pi 5-öt. Leírás: [Raspberry Pi camera software](https://www.raspberrypi.com/documentation/accessories/camera-software.html). Ha utána `dmaHeap allocation failure` hibát kapsz, a felhasználónak nincs jogosultsága; erre egy udev szabály a megoldás ([Pi 5 + Ubuntu 24.04 kamera](https://tanat44.github.io/posts/2026/2026-05-16-pi5-camera-ubuntu)).
- **USB webkamera:** a `robot_config.json` `camera` szakaszában állítsd a `"backend"`-et `"opencv"`-re. A kamerák listája: `v4l2-ctl --list-devices`.
- **Fejjel lefelé szerelt kamera:** `"rotate_180": true`.

### 2. Távolság-küszöb (kötelező)

A `nod_tool.py` elején a `DIST_CLOSE_RAW = 450` csak becslés. Mérd ki:

1. Tarts egy dobozt 10 cm-re a robot elé.
2. `ros2 topic echo /arduino_sensors` – a `"type": "dist"` üzenetek első három értéke a három első távolságmérő (DF_L, DF_M, DF_R).
3. Ezt az értéket írd a `DIST_CLOSE_RAW`-ba. Ha a szenzor közelebbi tárgyra **kisebb** számot ad, állítsd a `DIST_HIGHER_IS_CLOSER`-t `False`-ra.

### 3. Fordulás és lépéshossz

Odometria még nincs, ezért a fordulás és az előremenés időzített. Ha a robot kevesebbet fordul a kelleténél, növeld a `TURN_SCALE`-t; ha rövidebbet megy, a `DRIVE_SCALE`-t (mindkettő alapból 1.0). A lassú forduláshoz kis PWM tartozik (kb. ±20); ha a kerekek meg sem mozdulnak, növeld a `TURN_SPEED`-et.

### 4. Biztonság

Az `arduino_bridge.py`-ban nincs időkorlát a mozgásparancsokra: ha az ágens mozgás közben leáll, a robot az utolsó parancsot hajtja tovább. Az eszköz a végén mindig küld egy megállást, de **az első próbát a kerekek felemelt állapotában** végezd. Javasolt egy kb. 0,5 s-os időkorlátot tenni az `arduino_bridge.py`-ba.

---

## Indítás

A szokásos módon másold a `MiniRobotAI` tartalmát a robot `~/minirobot` mappájába, és indítsd:

```bash
./start_tmux.sh
```

A kamera a **ROS_Bridge** ablak jobb alsó paneljében fut. Ellenőrzés (kb. 10 kép/s):

```bash
ros2 topic hz /camera/image_raw
```

Kézi indítás: `source /opt/ros/jazzy/setup.bash && cd ~/minirobot && python3 camera_node.py`

---

## Kipróbálás

Tegyél egy jól látható tárgyat (pl. piros labdát) a robot elé kb. fél méterre, a kamera látószögén belül, és mondd: **„Menj oda a piros labdához!”**

| Helyzet | Várt viselkedés |
|---|---|
| A tárgy előtte vagy kicsit oldalt | OBSERVING arc → fordul → 3 rövid lépés → SUCCESS arc, a válasz `[SUCCESS]`-szel kezdődik |
| A tárgy már nagyon közel | Nem megy közelebb, SUCCESS |
| A tárgy nem látszik | Nem mozdul, SORRY |
| Nincs kamerakép vagy távolságadat | Nem mozdul, ALERT |

---

## Beállítások (`nod_tool.py` eleje)

| Név | Alap | Jelentés |
|---|---|---|
| `CAMERA_TOPIC` | `/camera/image_raw` | Innen olvassa a képet. |
| `TURN_SPEED` | 1.5 | Fordulási sebesség, rad/s. |
| `TURN_SCALE`, `DRIVE_SCALE` | 1.0 | Szorzók, ha a robot kevesebbet fordul / megy a kelleténél. |
| `TURN_STEP_DEG` | 20 | Az első fordulás fokban. |
| `MAX_TURNS` | 4 | Legfeljebb ennyiszer fordul. |
| `DRIVE_SPEED` | 0.10 | Előremenés, m/s. |
| `STEPS`, `STEP_M` | 3, 0.05 | Lépések száma és hossza (m). |
| `DIST_CLOSE_RAW` | 450 | Távolság-küszöb (nyers érték) – **mérd ki**. |
| `DIST_HIGHER_IS_CLOSER` | `True` | Nagyobb érték = közelebbi tárgy? |
| `DIST_MAX_AGE_S` | 1.5 | Ennél régebbi távolságadat = nincs adat, nem mozdul. |

A kamera beállításai a `robot_config.json` `camera` szakaszában vannak, mindegyikhez magyarázattal.

---

## Modell

- **Gemini (alap):** a `config.toml`-ban eddig a `gemini-3-flash-preview` szerepelt. Ez egy 2025 decemberi előzetes modell, amelyet a Google már elavultként jelöl. A `text-embedding-004`-et 2026. január 14-én leállították. A külön commit a `gemini-3.8-flash`-re (stabil, képet és eszközhívást is kezel) és a `gemini-embedding-2`-re cseréli őket.
- **Fix verziószámot használj**, ne a `gemini-flash-latest` álnevet: az álnév figyelmeztetés nélkül másik modellre válthat, és a LangChain Gemini-csatolója a Gemini 3 eszközhívásaihoz szükséges javítást csak akkor alkalmazza, ha a modell nevében benne van a verziószám.
- **Hibaüzenet `thought_signature`-rel az első eszközhíváskor:** túl régi a LangChain Gemini-csatolója. A `rai_venv`-ben: `pip install -U langchain-google-genai`
- **Helyi modell (Ollama):** a `check_emotion_tags.py` megméri, hogyan tartja egy modell az állapotkódokat. A PC-n, az Ollama mellett futtasd (ROS nem kell):

  ```bash
  python3 check_emotion_tags.py --prompt robot_prompt.json --model qwen3-vl:4b-instruct --runs 3
  ```

---

## Tesztelés

**Tesztelve** (valódi ROS és robot nélkül, helyettesítőkkel):

- Az odamenés egy szimulált 2D világban: a robot a `/cmd_vel` alapján mozog ±20–40% időzítési hibával, kamerakép és 2 Hz-es távolságmérők. 11 helyzet × 2 futás: az oldalt (25–29°) lévő tárgyak felé fordulva 11°-on belül állt meg, a közeli tárgy előtt 9–11 cm-re megállt, kép vagy távolságadat nélkül nem mozdult.
- A kamera node egy hamis `rpicam-vid`-del (ugyanazokat a kapcsolókat fogadja, mint az igazi) és egy videófájllal: kb. 10 kép/s, helyes színek, újraindítás kamerahiba és lefagyás után, forgatás, átméretezés.
- A JPEG-képfolyam darabolása (33 eset), az ágens arc-tartaléka és az előzmények rövidítése.

**Nem tesztelt:** valódi ROS és RAI, valódi modell (Gemini vagy más), a MiniRobot hardvere és a Pi 5 kamerája.
