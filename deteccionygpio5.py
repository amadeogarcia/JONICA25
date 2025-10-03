from picamera2 import Picamera2
from ultralytics import YOLO
import RPi.GPIO as GPIO
import time
import threading
from queue import Queue

# -----------------------------
# Configurar GPIO
# -----------------------------
GPIO.setmode(GPIO.BCM)
STOP_PIN = 26  # GPIO26 -> pin físico 37
GO_PIN   = 16  # GPIO16 -> pin físico 36
GPIO.setup(STOP_PIN, GPIO.OUT)
GPIO.setup(GO_PIN, GPIO.OUT)
GPIO.output(STOP_PIN, 0)
GPIO.output(GO_PIN, 0)

# -----------------------------
# Configurar YOLO (NCNN)
# -----------------------------
model = YOLO("yolov8npareysemaforo_ncnn_model", task="detect")

# -----------------------------
# Configurar Picamera2
# -----------------------------
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(
    main={"format": "BGR888", "size": (640, 640)}
)
picam2.configure(camera_config)
picam2.start()

# -----------------------------
# Cola para frames
# -----------------------------
frame_queue = Queue(maxsize=2)
detections = None
running = True

# -----------------------------
# Variables de control PARE
# -----------------------------
pare_activo = False
pare_tiempo_inicio = 0
pare_duracion = 5  # segundos

# -----------------------------
# Hilo de captura
# -----------------------------
def capture_thread():
    while running:
        img = picam2.capture_array()
        if not frame_queue.full():
            frame_queue.put(img)

# -----------------------------
# Hilo de inferencia YOLO
# -----------------------------
def yolo_thread():
    global detections
    while running:
        if not frame_queue.empty():
            while not frame_queue.empty():
                input_frame = frame_queue.get()
            results = model.track(input_frame, verbose=False)
            if len(results) > 0:
                detections = results[0].boxes
            else:
                detections = None

# -----------------------------
# Iniciar hilos
# -----------------------------
threading.Thread(target=capture_thread, daemon=True).start()
threading.Thread(target=yolo_thread, daemon=True).start()

# -----------------------------
# Bucle principal
# -----------------------------
try:
    while running:
        # Por defecto: apagar GO (STOP se maneja con timer)
        GPIO.output(GO_PIN, 0)

        if detections is not None:
            for box in detections:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                nombre = model.names[cls_id].lower()

                # --- Señal PARE con timer ---
                if nombre == "pare" and conf > 0.3:
                    if not pare_activo:  # solo si no estaba activo
                        pare_activo = True
                        pare_tiempo_inicio = time.time()
                        GPIO.output(STOP_PIN, 1)
                        print("Señal PARE detectada! Pin activado por 5s. conf: ")
                        print(conf)

                # --- Señal Semáforo Verde (en tiempo real) ---
                if nombre == "semaforoverde" and conf > 0.4:
                    GPIO.output(GO_PIN, 1)
                    print("Semáforo VERDE detectado! Pin activado. conf:")
                    print(conf)

        # -----------------------------
        # Control del tiempo del PARE
        # -----------------------------
        if pare_activo:
            if time.time() - pare_tiempo_inicio >= pare_duracion:
                GPIO.output(STOP_PIN, 0)  # apagar pin
                pare_activo = False
                print("? Tiempo de PARE finalizado. Se reinicia la detección.")

        time.sleep(0.05)  # pequeño delay para no saturar la CPU

finally:
    running = False
    picam2.stop()
    GPIO.output(STOP_PIN, 0)
    GPIO.output(GO_PIN, 0)
    GPIO.cleanup()
    print("Programa finalizado y GPIO limpio.")
