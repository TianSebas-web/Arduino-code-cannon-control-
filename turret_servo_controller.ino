#include <Arduino.h>
#include <Servo.h>

// --- CONFIGURACIÓN DE PINES DEL ARDUINO NANO ---
const int PAN_MAIN_PIN = 9;   // Servo 360 (ASUMIMOS CONTROLABLE POR POSICIÓN 0-180)
const int TILT_PIN = 10;  // Servo 180 (TILT)
const int ROLL_FIRE_PIN = 11; // Servo 360 (Disparo/Turret - Controlado por VELOCIDAD/TIEMPO)
const int PAN_LASER_PIN = 6;  // Servo 90 (PAN del Láser)
const int LASER_PIN = 8;  // Láser ON/OFF

// Servo Objects
Servo panMainServo;
Servo tiltServo;
Servo rollServo;
Servo panLaserServo; 

// !!! PULSE WIDTHS y LÍMITES !!!
const int MIN_PULSE = 500;  
const int MAX_PULSE = 2400; 

// LÍMITES FÍSICOS
const int TILT_MIN = 30;  // Límite seguro inferior para TILT 180
const int TILT_MAX = 150; // Límite seguro superior para TILT 180

// VALORES DE DISPARO (Ajustados para el servo 360 de Turret)
const int ROLL_MOVE_SPEED = 120; // Velocidad para girar el tambor (ej: 120 es rápido en 360 modificado)
const int ROLL_STOP_SPEED = 90;  // Velocidad de parada (Centro)
const int ROLL_PRECISION_MS = 150; // Tiempo para un dardo (AJUSTAR ESTO)

// Angulos de entrada de Python (rango -90 a 90)
float panAngle = 0.0;
float tiltAngle = 0.0;

// --- PROTOTIPOS DE FUNCIONES ---
void fire();
void homeServos();
void handleSerialCommand(String command);


// --- IMPLEMENTACIÓN DE FUNCIONES ---

void fire() { 
    // Mueve el servo de DISPARO (ROLL 360) a una velocidad y lo detiene.
    rollServo.write(ROLL_MOVE_SPEED); // Gira a velocidad alta (ej. 120)
    delay(ROLL_PRECISION_MS); 
    rollServo.write(ROLL_STOP_SPEED); // Detiene el giro (90)
    delay(50);
    Serial.println("FIRING DART");
}

void homeServos(){
    // Centro todos los servos a posición de reposo
    panMainServo.write(90);
    rollServo.write(ROLL_STOP_SPEED); 
    tiltServo.write(90); 
    panLaserServo.write(90); // INICIALIZACIÓN DEL LÁSER A 90 GRADOS
    
    Serial.println("HOMING");
}

void handleSerialCommand(String command) {
    // 1. Parsing: PXX.XTXX.XF1L1
    int pIndex = command.indexOf('P');
    int tIndex = command.indexOf('T');
    int fIndex = command.indexOf('F');
    int lIndex = command.indexOf('L');
    
    float localPan = 0.0;
    float localTilt = 0.0;
    bool fireDart = false;
    bool laserOn = false;

    if (pIndex != -1 && tIndex != -1) { localPan = command.substring(pIndex + 1, tIndex).toFloat(); }
    if (tIndex != -1 && fIndex != -1) { localTilt = command.substring(tIndex + 1, fIndex).toFloat(); }
    if (fIndex != -1 && lIndex != -1) { fireDart = (command.substring(fIndex + 1, fIndex + 2) == "1"); }
    if (lIndex != -1) { laserOn = (command.substring(lIndex + 1, lIndex + 2) == "1"); }

    // --- 2. EXECUTE COMMANDS ---

    // A. PAN PRINCIPAL (360° - POSICIÓN ASUMIDA): Mapeo a 180 grados
    int panMainPos = map(localPan, -90, 90, 0, 180);
    panMainServo.write(panMainPos);

    // B. TILT (180°): Mapeo al rango de seguridad
    int tiltPos = map(localTilt, -90, 90, TILT_MIN, TILT_MAX);
    tiltServo.write(tiltPos);

    // C. PAN LÁSER (90°): Mapeo al centro y pequeño margen (ej. 45 a 135)
    // Usaremos el mismo PanMainPos para mantener la alineación
    panLaserServo.write(panMainPos); 

    // D. LÁSER ON/OFF
    digitalWrite(LASER_PIN, laserOn ? HIGH : LOW);
    
    // E. DISPARO
    if (fireDart) { fire(); }
}


// --- SETUP Y LOOP ---

void setup() { 
    Serial.begin(115200); 

    // 1. Attach Servos con Pulse Widths
    panMainServo.attach(PAN_MAIN_PIN, MIN_PULSE, MAX_PULSE);    
    tiltServo.attach(TILT_PIN, MIN_PULSE, MAX_PULSE); 
    rollServo.attach(ROLL_FIRE_PIN, MIN_PULSE, MAX_PULSE); 
    panLaserServo.attach(PAN_LASER_PIN, MIN_PULSE, MAX_PULSE); 

    // 2. Configurar el PIN DEL LÁSER (Pin 8)
    pinMode(LASER_PIN, OUTPUT); 
    digitalWrite(LASER_PIN, LOW); 

    // 3. Posición inicial (Centro)
    homeServos(); 
    
    Serial.println("Turret Ready for Serial Control (P:360, T:180, R:360, L:90).");
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n'); 
        handleSerialCommand(command); 
    }
    delay(5);
}
