import sys
from time import sleep

# --- CONFIGURACIÓN SERIAL Y HARDWARE ---
# AJUSTAR: El puerto COM donde aparece tu Arduino Nano.
# Ejemplo Windows: 'COM5' | Ejemplo Raspberry Pi: '/dev/ttyACM0'
ARDUINO_PORT = 'COM7' 
BAUD_RATE = 115200

# CONFIGURACIÓN DE LA CÁMARA
CAMERA_ID = 2
CAMERA_FOV_H_DEG = 60.0 # Campo de visión horizontal estimado

# GANANCIAS DE CONTROL PROPORCIONAL (KP)
# Estos valores suavizan el movimiento de los servos. AJUSTAR según la respuesta física.
KP_PAN = 0.5 
KP_TILT = 0.5

# LÓGICA DE ACTIVACIÓN: 
# Área mínima de píxeles para que un globo sea considerado un objetivo (proxy de cercanía).
MIN_FIRE_AREA = 10000 


# --- INICIALIZACIÓN DE HARDWARE Y MODELO ---

# Inicializar Serial
try:
    ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    sleep(2)
    print(f"Serial connection established on {ARDUINO_PORT}.")
except serial.SerialException as e:
    print(f"ERROR: Could not connect to Arduino on {ARDUINO_PORT}. Details: {e}")
    ser = None 
    
# Inicializar Cámara
cap = cv2.VideoCapture(CAMERA_ID)
if not cap.isOpened():
    print("Error: Could not open camera.")
    if ser: ser.close()
    sys.exit()

W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
CENTER_X = W // 2
CENTER_Y = H // 2

# CÁLCULOS DE CALIBRACIÓN GEOMÉTRICA
FOV_H_RAD = np.deg2rad(CAMERA_FOV_H_DEG)
FOCAL_LENGTH_PIXELS = (W / 2) / np.tan(FOV_H_RAD / 2) 
PIXELS_PER_DEGREE = W / CAMERA_FOV_H_DEG

# Cargar Modelo YOLO
try:
    # RUTA ABSOLUTA que verificamos previamente
    model = YOLO('C:/Users/Laura/OneDrive/Documentos/vision comp/Balloon/runs/detect/balloon_detector/weights/best.pt')  
    model.args['imgsz'] = 320 # Velocidad máxima para la Raspberry Pi/PC
    print("YOLO Model loaded.")
except Exception as e:
    print(f"YOLO Load Error: {e}. Check path to 'best.pt'")
    if ser: ser.close()
    sys.exit()


# --- CONTROL SERIAL (LÁSER Y DISPARO) ---
def send_aiming_command(pan_angle_deg, tilt_angle_deg, fire_dart=False, laser_on=False):
    """
    Envía comandos P/T/F/L al Arduino vía serial.
    Formato: PXX.XTXX.XF1L1\n
    """
    if ser is None:
        return

    # Asegura que los ángulos estén dentro del rango de -90 a 90
    pan_angle_deg = max(-90.0, min(90.0, pan_angle_deg))
    tilt_angle_deg = max(-90.0, min(90.0, tilt_angle_deg))
    
    fire_cmd = 1 if fire_dart else 0
    laser_cmd = 1 if laser_on else 0
    
    # Formato: PXX.XTXX.XF1L1\n
    command = f"P{pan_angle_deg:.1f}T{tilt_angle_deg:.1f}F{fire_cmd}L{laser_cmd}\n"
    
    try:
        ser.write(command.encode('utf-8'))
    except Exception as e:
        print(f"Serial communication error: {e}")

# --- BÚSQUEDA Y PRIORIZACIÓN DE OBJETOS ---
def get_best_target(results):
    """
    Selecciona el objetivo más cercano (más grande) basado en el área del cuadro delimitador.
    """
    targets = []
    
    if results and results[0].boxes:
        for box in results[0].boxes:
            conf = box.conf.item()
            if conf > 0.5: 
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                area = (x2 - x1) * (y2 - y1)
                
                targets.append({
                    'box': box,
                    'area': area,
                    'conf': conf
                })

    if not targets:
        return None, 0

    # Priorizar por área (el más grande/cercano primero)
    targets.sort(key=lambda t: t['area'], reverse=True)
    best_target = targets[0]
    
    return best_target['box'], best_target['area']

# --- MAIN LOOP ---
print("\nStarting camera tracker. Press 'q' to exit.")

try:
    while True:
        ret, frame = cap.read()
        if not ret: break

        # 1. Detección (YOLO)
        results = model(frame, verbose=False)
        
        # 2. Priorización de Objetivo
        best_box, distance_proxy = get_best_target(results)
        
        target_pan_angle = 0.0
        target_tilt_angle = 0.0
        should_fire = False
        laser_on = False 

        if best_box is not None:
            x1, y1, x2, y2 = [int(v) for v in best_box.xyxy[0]]
            target_pixel_x = (x1 + x2) // 2
            target_pixel_y = (y1 + y2) // 2
            
            # --- CÁLCULO DE ÁNGULOS Y CONTROL PROPORCIONAL ---
            
            # 1. Calcular ERROR DE ÁNGULO (Pan)
            pixel_deviation_x = target_pixel_x - CENTER_X 
            error_pan_deg = pixel_deviation_x / PIXELS_PER_DEGREE
            
            # Aplicar Control Proporcional (KP) para suavizar la puntería
            target_pan_angle = error_pan_deg * KP_PAN

            # 2. Calcular ERROR DE ÁNGULO (Tilt)
            vertical_pixel_distance = CENTER_Y - target_pixel_y 
            error_tilt_deg = np.rad2deg(np.arctan(vertical_pixel_distance / FOCAL_LENGTH_PIXELS))
            
            # Aplicar Control Proporcional (KP)
            target_tilt_angle = error_tilt_deg * KP_TILT

            # --- LÓGICA DE ACTIVACIÓN ---
            
            # Encender el láser si detectamos un globo válido (proxy de cercanía)
            if distance_proxy > MIN_FIRE_AREA * 0.5: 
                laser_on = True 
            
            # Disparar solo si el globo es grande Y está centrado
            if distance_proxy > MIN_FIRE_AREA: 
                # Centrado: Error muy bajo (menor a 2 grados)
                if abs(error_pan_deg) < 2.0 and abs(error_tilt_deg) < 2.0:
                    should_fire = True
            
            # Dibujar la caja delimitadora
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"Balloon: Area {distance_proxy} @ P:{target_pan_angle:.1f} T:{target_tilt_angle:.1f} FIRE:{should_fire}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(frame, (CENTER_X, CENTER_Y), 5, (0, 0, 255), -1)

        # 3. Control Serial: Envía el comando al Arduino
        send_aiming_command(target_pan_angle, target_tilt_angle, should_fire, laser_on)
        
        cv2.imshow('YOLO Camera Tracker', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'): break

finally:
    print("\nShutting down...")
    if ser:
        # Apagar el láser y centrar antes de cerrar
        send_aiming_command(0.0, 0.0, False, False)
        sleep(0.5)
        ser.close()
    cap.release()
    cv2.destroyAllWindows()
    print("Cleanup complete.")