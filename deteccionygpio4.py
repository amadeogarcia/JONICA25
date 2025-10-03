from picamera2 import Picamera2
from ultralytics import YOLO
from PIL import Image, ImageDraw
import threading
import pygame
from queue import Queue
import RPi.GPIO as GPIO
import time

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

# -----------------------------
# Iniciar hilos
# -----------------------------
threading.Thread(target=capture_thread, daemon=True).start()
threading.Thread(target=yolo_thread, daemon=True).start()

# -----------------------------
# Inicializar pygame
# -----------------------------
pygame.init()
screen = pygame.display.set_mode((640, 640))
pygame.display.set_caption("YOLO Tracking sin OpenCV")

# -----------------------------
# Bucle principal
# -----------------------------
try:
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Tomar el frame más reciente
        display_frame = None
        while not frame_queue.empty():
            display_frame = frame_queue.get()

        if display_frame is None:
            continue

        # Convertir a PIL y rotar 180°
        img_pil = Image.fromarray(display_frame)
        img_pil = img_pil.rotate(180)
        draw = ImageDraw.Draw(img_pil)

        # Por defecto: apagar GO (el STOP se controla con timer)
        GPIO.output(GO_PIN, 0)

        # Dibujar detecciones y controlar GPIO
        if detections is not None:
            for box in detections:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()

                # Rotar bounding box 180° según tamaño del frame
                frame_W, frame_H = display_frame.shape[1], display_frame.shape[0]
                x1, y1, x2, y2 = frame_W - x2, frame_H - y2, frame_W - x1, frame_H - y1

                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                label = f"{model.names[cls_id]} {conf:.2f}"
                draw.rectangle([x1, y1, x2, y2], outline="red", width=2)
                draw.text((x1, y1 - 10), label, fill="red")

                # --- Señal PARE con timer ---
                if model.names[cls_id].lower() == "pare" and conf > 0.3:
                    if not pare_activo:  # solo si no estaba activo
                        pare_activo = True
                        pare_tiempo_inicio = time.time()
                        GPIO.output(STOP_PIN, 1)
                        print("Señal PARE detectada! Pin activado por 5s, conf: ")
                        print(conf)

                # --- Señal Semáforo Verde (sin timer) ---
                if model.names[cls_id].lower() == "semaforoverde" and conf > 0.3:
                    GPIO.output(GO_PIN, 1)
                    print("Semáforo VERDE detectado! Pin activado, conf: ")
                    print(conf)

        # -----------------------------
        # Control del tiempo del PARE
        # -----------------------------
        if pare_activo:
            if time.time() - pare_tiempo_inicio >= pare_duracion:
                GPIO.output(STOP_PIN, 0)  # apagar pin
                pare_activo = False
                print("Tiempo de PARE finalizado. Se reinicia la detección.")

        # Mostrar en pygame
        frame_surface = pygame.image.fromstring(img_pil.tobytes(), img_pil.size, img_pil.mode)
        screen.blit(frame_surface, (0, 0))
        pygame.display.update()

finally:
    running = False
    picam2.stop()
    pygame.quit()
    GPIO.output(STOP_PIN, 0)
    GPIO.output(GO_PIN, 0)
    GPIO.cleanup()
    print("Programa finalizado y GPIO limpio.")
