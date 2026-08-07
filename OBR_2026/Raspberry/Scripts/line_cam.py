import cv2 as cv
import numpy as np
import time
from picamera2 import Picamera2
from multiprocessing.shared_memory import SharedMemory

import mp_manager as mgr
from constants import FRAME_WIDTH, FRAME_HEIGHT, FRAME_SHAPE, LINE_LOST, LINE_FOUND

DEBUG = True
CAMERA_NUM = 0

BLACK_THRESH = 60
MIN_CONTOUR_AREA = 80

ROI_ESQUERDA_CIMA = (0,   60,  0, 80)
ROI_CIMA          = (60, 260,  0, 80)
ROI_DIREITA_CIMA  = (260, 320, 0, 80)


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


def capturar_e_processar():
    shm = SharedMemory(name=mgr.shm.name)
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
        mgr.camera_ok.value = 0
        shm.close()
        return

    mgr.camera_ok.value = 1
    center_x = FRAME_WIDTH // 2

    try:
        while not mgr.terminate.is_set():
            try:
                frame_bgr = picam2.capture_array()
            except Exception as e:
                print(f"ERRO ao capturar frame da Picamera2: {e}")
                mgr.camera_ok.value = 0
                mgr.line_status.value = LINE_LOST
                break

            h, w = frame_bgr.shape[:2]
            if (w, h) != (FRAME_WIDTH, FRAME_HEIGHT):
                frame_bgr = cv.resize(frame_bgr, (FRAME_WIDTH, FRAME_HEIGHT))

            frame_rgb = cv.cvtColor(frame_bgr, cv.COLOR_BGR2RGB)

            with mgr.frame_lock:
                frame_buf[:] = frame_rgb
                mgr.novo_frame_flag.value = 1

            centro_cima = detectar_centro(frame_rgb, ROI_CIMA)
            centro_esq = detectar_centro(frame_rgb, ROI_ESQUERDA_CIMA)
            centro_dir = detectar_centro(frame_rgb, ROI_DIREITA_CIMA)

            alvo_x = escolher_alvo(centro_cima, centro_esq, centro_dir, center_x)

            if alvo_x is not None:
                mgr.line_status.value = LINE_FOUND
                mgr.line_angle.value = alvo_x - center_x
                mgr.cx_alvo_v.value = alvo_x
            else:
                mgr.line_status.value = LINE_LOST
    finally:
        mgr.camera_ok.value = 0
        picam2.stop()
        shm.close()
