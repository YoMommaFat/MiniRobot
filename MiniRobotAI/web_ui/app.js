const root = document.documentElement;
const eyeL = document.getElementById('eye-l');
const eyeR = document.getElementById('eye-r');
const mouth = document.getElementById('mouth');
const browL = document.getElementById('brow-l');
const browR = document.getElementById('brow-r');

let isSpeaking = false;

const emotions = {
    "READY": { color: "cyan", eyes: "normal", mouth: "normal", brow: "hidden" },
    "WINK": { color: "cyan", eyes: ["closed", "normal"], mouth: "normal", brow: "hidden" },
    "CONFUSED": { color: "cyan", eyes: ["squint", "normal"], mouth: "normal", brow: "visible confused" },
    "ALERT": { color: "red", eyes: "normal", mouth: "normal", brow: "visible alert" },
    "OBSERVING": { color: "cyan", eyes: "squint", mouth: "hidden", brow: "hidden" },
    "AFFECTION": { color: "hotpink", eyes: "heart", mouth: "smile", brow: "hidden" },
    "CURIOSITY": { color: "cyan", eyes: "normal", mouth: "circle", brow: "hidden" },
    "THINKING": { color: "cyan", eyes: "closed", mouth: "normal", brow: "hidden" },
    "SUCCESS": { color: "lime", eyes: "happy", mouth: "smile", brow: "hidden" },
    "SORRY": { color: "cyan", eyes: "normal", mouth: "sad", brow: "visible sad" }
};

function setEmotion(name) {
    if (!emotions[name]) return;
    const e = emotions[name];
    
    root.style.setProperty('--face-color', e.color);
    
    eyeL.className = `eye left ${Array.isArray(e.eyes) ? e.eyes[0] : e.eyes}`;
    eyeR.className = `eye right ${Array.isArray(e.eyes) ? e.eyes[1] : e.eyes}`;
    mouth.className = `mouth ${e.mouth}`;
    
    if (isSpeaking && e.mouth !== 'hidden') {
        mouth.classList.add('speaking');
    }

    browL.className = `brow left ${e.brow}`;
    browR.className = `brow right ${e.brow}`;
}

// Speaking animation state (movement is handled by CSS hardware acceleration)
function setSpeaking(speaking) {
    isSpeaking = speaking;
    if (speaking) {
        if (!mouth.classList.contains('hidden')) {
            mouth.classList.add('speaking');
        }
    } else {
        mouth.classList.remove('speaking');
    }
}

window.setEmotion = setEmotion;
window.setSpeaking = setSpeaking;
// --- WEBSOCKET LOGIC (Bridge to ROS 2 Python Node) ---
function connectWebSocket() {
    const host = window.location.hostname || 'localhost';
    const ws = new WebSocket('ws://' + host + ':8765');

    ws.onopen = () => {
        console.log("Successfully connected to ROS 2 Face Node!");
        // A teszt gombokat mostantól a 🎭 ikon vezérli, nem rejtjük el automatikusan!
        setEmotion('READY');
    };

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            console.log("Command received from robot:", data);
            
            if (data.type === 'emotion') {
                setEmotion(data.value);
            } else if (data.type === 'speaking') {
                setSpeaking(data.value === true || data.value === 'true');
            }
        } catch (e) {
            console.error("Error parsing WebSocket message:", e);
        }
    };

    ws.onclose = () => {
        console.log("Connection lost. Reconnecting in 2 seconds...");
        // Reconnect automatically if connection is lost
        setTimeout(connectWebSocket, 2000);
    };

    ws.onerror = (error) => {
        console.error("WebSocket error:", error);
    };
}

// Start WebSocket connection attempt on load
connectWebSocket();

// ==========================================
// MÜSZERFAL & ROSBRIDGE (DEBUG UI LOGIKA)
// ==========================================

let debugActive = false;
let emotionsActive = false;
let rosbridge = null;
let joyManager = null;

// Érzelmek panel ki/be kapcsolása (a 🎭 gomb hívja meg)
function toggleEmotions() {
    emotionsActive = !emotionsActive;
    
    // Ha megnyitjuk az érzelmeket, zárjuk be a műszerfalat!
    if (emotionsActive && debugActive) {
        toggleDebug();
    }
    
    const controls = document.getElementById('controls');
    if (emotionsActive) {
        controls.style.display = 'flex';
    } else {
        controls.style.display = 'none';
    }
}

// Műszerfal ki/be kapcsolása (a ⚙️ gomb hívja meg)
function toggleDebug() {
    debugActive = !debugActive;
    
    // Ha megnyitjuk a műszerfalat, zárjuk be az érzelmeket!
    if (debugActive && emotionsActive) {
        toggleEmotions();
    }
    
    if (debugActive) {
        document.body.classList.add('debug-active');
        initRosBridge();
    } else {
        document.body.classList.remove('debug-active');
    }
}

// Az arc beszéd animációjának szinkronizálása a LED-el
const originalSetSpeaking = setSpeaking;
window.setSpeaking = function(speaking) {
    originalSetSpeaking(speaking);
    const ttsDot = document.getElementById('ai-tts');
    if(ttsDot) { 
        if(speaking) ttsDot.classList.add('active'); 
        else ttsDot.classList.remove('active'); 
    }
};

// Meglévő Emotion hívás kiegészítése a Gondolkodás LED-hez
const originalSetEmotion = setEmotion;
window.setEmotion = function(name) {
    originalSetEmotion(name);
    const llmDot = document.getElementById('ai-llm');
    if(llmDot) {
        if(name === 'THINKING') llmDot.classList.add('thinking');
        else llmDot.classList.remove('thinking');
    }
};

function initRosBridge() {
    if (rosbridge) {
        setupJoystick(); // Ha már megvan, csak rajzoljuk újra, ha kell
        return; 
    }

    // Windows alatt (file:/// megnyitás) a hostname üres string, ami miatt a websocket URL hibás lesz (ws://:9090),
    // és a ROSLIB azonnal, fatális hibával leállítja a JavaScript futását, mielőtt a joystickot elérné!
    const host = window.location.hostname || 'localhost';
    const rosUrl = 'ws://' + host + ':9090';
    
    try {
        rosbridge = new ROSLIB.Ros({ url: rosUrl });
    } catch (e) {
        console.error("Hiba a ROS hálózat indításakor:", e);
        // Ne álljon le a futás, menjünk tovább!
    }
    
    // FONTOS: A Joystickot mindenképp megrajzoljuk, még akkor is, ha magát a hálózati kapcsolatot esetleg nem tudja felépíteni!
    setupJoystick();

    if (rosbridge) {
        rosbridge.on('connection', () => {
            console.log('Sikeres csatlakozás a ROS 2 rosbridge-hez!');
            setupRosSubscribers();
        });

        rosbridge.on('error', (error) => {
            console.warn('Nem fut a rosbridge_server? Error: ', error);
        });

        rosbridge.on('close', () => {
            rosbridge = null;
        });
    }
}

function setupRosSubscribers() {
    // AKKUMULÁTOR FIGYELÉS
    const battSub = new ROSLIB.Topic({
        ros: rosbridge, name: '/battery_state', messageType: 'sensor_msgs/BatteryState'
    });
    
    battSub.subscribe((msg) => {
        const v = msg.voltage;
        document.getElementById('batt-val').innerText = v.toFixed(1) + ' V';
        
        let percent = ((v - 6.6) / (8.4 - 6.6)) * 100;
        let hue;
        
        if (v < 6.6) {
            hue = 0; // Piros
            percent = 5; // Hogy látszódjon a piros csík
        } else if (v > 8.4) {
            hue = 0; // Piros (túltöltés)
            percent = 100;
        } else {
            // 0-100% között a hue 0 (piros) és 120 (zöld) között változik, középen (60) sárga
            hue = (percent / 100) * 120;
        }
        
        const bar = document.getElementById('battery-fill');
        if (bar) {
            bar.style.width = percent + '%';
            bar.style.backgroundColor = `hsl(${hue}, 100%, 50%)`;
        }
    });

    // ODOMETRIA
    const odomSub = new ROSLIB.Topic({ 
        ros: rosbridge, name: '/odom', messageType: 'nav_msgs/Odometry' 
    });
    
    odomSub.subscribe((msg) => {
        document.getElementById('enc-l').innerText = msg.pose.pose.position.x.toFixed(2);
        document.getElementById('enc-r').innerText = msg.pose.pose.position.y.toFixed(2);
    });

    // ARDUINO NYERS SZENZOROK (Cliff, Bump, Dist)
    const sensorSub = new ROSLIB.Topic({ 
        ros: rosbridge, name: '/arduino_sensors', messageType: 'std_msgs/String' 
    });
    
    sensorSub.subscribe((msg) => {
        try {
            const data = JSON.parse(msg.data);
            if (data.type === 'cliff') {
                const [CFL, CFM, CFR, CRL, CRM, CRR] = data.val;
                // Arduino kód alapján: CLIFF_THR = 32. Ha alatta van, akkor szakadék van (true).
                document.getElementById('sens-cfl').classList.toggle('active', CFL < 32);
                document.getElementById('sens-cfm').classList.toggle('active', CFM < 32);
                document.getElementById('sens-cfr').classList.toggle('active', CFR < 32);
                document.getElementById('sens-crl').classList.toggle('active', CRL < 32);
                document.getElementById('sens-crm').classList.toggle('active', CRM < 32);
                document.getElementById('sens-crr').classList.toggle('active', CRR < 32);
            } 
            else if (data.type === 'bump') {
                const [BPL, BPR] = data.val;
                // Feltételezzük, hogy 0 (LOW) jelenti a lenyomott állapotot.
                document.getElementById('sens-bpl').classList.toggle('active', BPL === 0);
                document.getElementById('sens-bpr').classList.toggle('active', BPR === 0);
            }
            else if (data.type === 'pwr') {
                const [aU_BAT, aRS_OP, aI_ARD, aI_RPI, aML_VP, aMR_VP] = data.val;
                
                // Konverzió: 2.56V referencia, 10-bit ADC (1023)
                // 1A = 1V ADC feszültség mindkét áramnál
                // Szorzó (Amperhez): 2.56 / 1023 = 0.002502 A / ADC lépés
                const current_factor = (2.56 / 1023.0);
                
                document.getElementById('cur-rpi').innerText = (aI_RPI * current_factor).toFixed(2) + " A";
                document.getElementById('cur-ard').innerText = (aI_ARD * current_factor).toFixed(2) + " A";
            }
        } catch (e) {
            console.warn("Hibás szenzor JSON:", e);
        }
    });
}

function setupJoystick() {
    // Ha már létezik egy joystick példány (de eltűnt a display: none miatt), töröljük!
    if (joyManager) {
        joyManager.destroy();
        joyManager = null;
    }
    
    // Adunk egy pillanatot (150ms) a böngészőnek, hogy a "display: none" biztosan eltűnjön 
    // és a DOM méretei frissüljenek, mielőtt a nipple.js kiszámolja a pozíciókat.
    setTimeout(() => {
        const zone = document.getElementById('joystick-zone');
        if (!zone) return;

        // Joystick vizuális generálása
        joyManager = nipplejs.create({ 
            zone: zone, mode: 'static', position: { left: '50%', top: '50%' }, color: '#0f0', size: 100
        });
        
        let cmdVel = null;
        
        joyManager.on('move', (evt, data) => {
            // Ha nincs hálózat, a joystick vizuálisan mozog, de nem küldünk semmit
            if (!rosbridge || !rosbridge.isConnected) return;
            
            // Csak akkor hozzuk létre a topicot, ha már van élő kapcsolat
            if (!cmdVel) {
                cmdVel = new ROSLIB.Topic({ 
                    ros: rosbridge, name: '/cmd_vel', messageType: 'geometry_msgs/Twist' 
                });
            }
            
            const max_speed = 0.35; // Max sebesség m/s
            const max_turn = 1.0;   // Max fordulás rad/s
            
            let lin = Math.sin(data.angle.radian) * (data.distance/50) * max_speed;
            let ang = -Math.cos(data.angle.radian) * (data.distance/50) * max_turn;
            
            const twist = new ROSLIB.Message({
                linear: { x: lin, y: 0.0, z: 0.0 },
                angular: { x: 0.0, y: 0.0, z: ang }
            });
            cmdVel.publish(twist);
        });
        
        joyManager.on('end', () => {
            if (!rosbridge || !rosbridge.isConnected || !cmdVel) return;
            cmdVel.publish(new ROSLIB.Message({
                linear: { x: 0.0, y: 0.0, z: 0.0 },
                angular: { x: 0.0, y: 0.0, z: 0.0 }
            }));
        });
    }, 550);
}

function sendPanic() {
    alert("VÉSZLEÁLLÍTÁS (PDN) parancs generálva! (Később ráköthetjük a ROS topicra)");
}
