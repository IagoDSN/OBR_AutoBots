import cv2 as cv
import numpy as np
from multiprocessing import Process, Value, Lock
from multiprocessing.shared_memory import SharedMemory
import time
from picamera2 import Picamera2

DEBUG = True
CAMERA_NUM = 0

FRAME_WIDTH = 320
FRAME_HEIGHT = 200
FRAME_SHAPE = (FRAME_HEIGHT, FRAME_WIDTH, 3)
FRAME_NBYTES = int(np.prod(FRAME_SHAPE))

BLACK_THRESH = 60
MIN_CONTOUR_AREA = 80

ROI_ESQUERDA_CIMA = (0,   60,  0, 50)
ROI_CIMA          = (60, 260,  0, 50)
ROI_DIREITA_CIMA  = (260, 320, 0, 50)

LINE_LOST = 0
LINE_FOUND = 1

KP = 0.6
BASE_SPEED = 60.0


def mascara_preto(frame_rgb):
    return np.all(frame_rgb < BLACK_THRESH, axis=2).astype(np.uint8) * 255


def detectar_centro(frame, roi):
    x1, x2, y1, y2 = roi
    recorte = frame[y1:y2, x1:x2]
    mask = mascara_preto(recorte)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    maior = max(contours, key=cv.contourArea)
    if cv.contourArea(maior) < MIN_CONTOUR_AREA:
        return None

    M = cv.moments(maior)
    if M["m00"] == 0:
        return None

    cx_local = int(M["m10"] / M["m00"])
    cy_local = int(M["m01"] / M["m00"])
    if mask[0, cx_local] == 255:
        cy_local = 0

    return (cx_local + x1, cy_local + y1)


def escolher_alvo(centro_cima, centro_esq, centro_dir, center_x):
    """- preto so na esquerda (+ centro) -> ramal esquerda, mira nela
    - preto so na direita  (+ centro) -> ramal direita, mira nela
    - preto nos dois lados ao mesmo tempo -> cruzamento, segue reto
    - nenhum dos lados -> comportamento normal (centro)
    """
    cx_cima = centro_cima[0] if centro_cima is not None else None
    cx_esq = centro_esq[0] if centro_esq is not None else None
    cx_dir = centro_dir[0] if centro_dir is not None else None

    tem_cima = cx_cima is not None
    tem_esq = cx_esq is not None
    tem_dir = cx_dir is not None

    if tem_esq and tem_dir:
        return cx_cima if tem_cima else center_x
    if tem_cima and tem_esq:
        return cx_esq
    if tem_cima and tem_dir:
        return cx_dir
    if tem_cima:
        return cx_cima
    return None


def calcular_comando_motor(status, erro, center_x):
    if status == LINE_LOST:
        return 0.0, 0.0

    correcao = KP * (erro / center_x)
    vel_esq = BASE_SPEED + correcao * BASE_SPEED
    vel_dir = BASE_SPEED - correcao * BASE_SPEED

    vel_esq = max(-100.0, min(100.0, vel_esq))
    vel_dir = max(-100.0, min(100.0, vel_dir))
    return vel_esq, vel_dir


def capturar_e_processar(shm_name, frame_lock, novo_frame_flag, camera_ok,
                          line_status, line_angle, cx_alvo_v):
    shm = SharedMemory(name=shm_name)
    frame_buf = np.ndarray(FRAME_SHAPE, dtype=np.uint8, buffer=shm.buf)

    try:
        picam2 = Picamera2(camera_num=CAMERA_NUM)
        config = picam2.create_video_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(0.3)
    except Exception as e:
        print(f"ERRO ao iniciar a Picamera2: {e}")
        camera_ok.value = 0
        shm.close()
        return

    camera_ok.value = 1
    center_x = FRAME_WIDTH // 2

    try:
        while True:
            try:
                frame_bgr = picam2.capture_array()
            except Exception as e:
                print(f"ERRO ao capturar frame da Picamera2: {e}")
                camera_ok.value = 0
                line_status.value = LINE_LOST
                break

            h, w = frame_bgr.shape[:2]
            if (w, h) != (FRAME_WIDTH, FRAME_HEIGHT):
                frame_bgr = cv.resize(frame_bgr, (FRAME_WIDTH, FRAME_HEIGHT))

            frame_rgb = cv.cvtColor(frame_bgr, cv.COLOR_BGR2RGB)

            with frame_lock:
                frame_buf[:] = frame_rgb
                novo_frame_flag.value = 1

            centro_cima = detectar_centro(frame_rgb, ROI_CIMA)
            centro_esq = detectar_centro(frame_rgb, ROI_ESQUERDA_CIMA)
            centro_dir = detectar_centro(frame_rgb, ROI_DIREITA_CIMA)

            alvo_x = escolher_alvo(centro_cima, centro_esq, centro_dir, center_x)

            if alvo_x is not None:
                line_status.value = LINE_FOUND
                line_angle.value = alvo_x - center_x
                cx_alvo_v.value = alvo_x
            else:
                line_status.value = LINE_LOST
    finally:
        picam2.stop()
        shm.close()