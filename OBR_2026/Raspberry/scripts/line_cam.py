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
MIN_CONTOUR_AREA = 2500
MIN_CONTOUR_AREA_VERDE = 800

ROI_ESQ = (0, 150, 0, 200)
ROI_DIR = (170, 320, 0, 200)

cx_anterior = None  # último centro conhecido da linha preta (usado quando só há verde)


def mascara_preto(frame_rgb):
    return np.all(frame_rgb < BLACK_THRESH, axis=2).astype(np.uint8) * 255


def mascara_verde(frame_rgb):
    lower_green = np.array([0, 100, 0])
    upper_green = np.array([100, 255, 100])
    return cv.inRange(frame_rgb, lower_green, upper_green)


def detectar_centro_preto(frame_rgb, roi):
    x1, x2, y1, y2 = roi
    recorte = frame_rgb[y1:y2, x1:x2]
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


def detectar_centro_verde(frame_rgb, roi):
    x1, x2, y1, y2 = roi
    recorte = frame_rgb[y1:y2, x1:x2]
    mask = mascara_verde(recorte)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    maior = max(contours, key=cv.contourArea)
    if cv.contourArea(maior) < MIN_CONTOUR_AREA_VERDE:
        return None

    M = cv.moments(maior)
    if M["m00"] == 0:
        return None

    cx_local = int(M["m10"] / M["m00"])
    cy_local = int(M["m01"] / M["m00"])
    if mask[0, cx_local] == 255:
        cy_local = 0

    return (cx_local + x1, cy_local + y1)


def processar_linha_com_verde(frame_rgb):
    global cx_anterior

    # 1) VERDE nos dois lados
    centro_verde_esq = detectar_centro_verde(frame_rgb, ROI_ESQ)
    centro_verde_dir = detectar_centro_verde(frame_rgb, ROI_DIR)

    # 2) PRETO nos dois lados
    centro_preto_esq = detectar_centro_preto(frame_rgb, ROI_ESQ)
    centro_preto_dir = detectar_centro_preto(frame_rgb, ROI_DIR)

    tem_preto_esq = centro_preto_esq is not None
    tem_preto_dir = centro_preto_dir is not None

    if tem_preto_esq and tem_preto_dir:
        cx_preto = (centro_preto_esq[0] + centro_preto_dir[0]) // 2
        cy_preto = min(centro_preto_esq[1], centro_preto_dir[1])
    elif tem_preto_esq:
        cx_preto, cy_preto = centro_preto_esq
    elif tem_preto_dir:
        cx_preto, cy_preto = centro_preto_dir
    else:
        cx_preto, cy_preto = None, None

    if cx_preto is not None:
        cx_anterior = cx_preto
        status_linha = LINE_FOUND
    elif centro_verde_esq is not None or centro_verde_dir is not None:
        cx_preto = cx_anterior
        status_linha = LINE_FOUND if cx_anterior is not None else LINE_LOST
    else:
        status_linha = LINE_LOST

    # só considera o verde válido se estiver ABAIXO do preto (mais perto do robô)
    if cy_preto is not None:
        verde_esq_valido = centro_verde_esq is not None and centro_verde_esq[1] > cy_preto
        verde_dir_valido = centro_verde_dir is not None and centro_verde_dir[1] > cy_preto
    else:
        verde_esq_valido = centro_verde_esq is not None
        verde_dir_valido = centro_verde_dir is not None

    if verde_esq_valido and verde_dir_valido:
        acao = "CURVA_180"
    elif verde_esq_valido:
        acao = "VIRAR_ESQUERDA"
    elif verde_dir_valido:
        acao = "VIRAR_DIREITA"
    else:
        acao = "SEGUIR_RETO"

    return acao, cx_preto, status_linha, centro_preto_esq, centro_preto_dir, centro_verde_esq, centro_verde_dir


def _desenhar_debug(frame_rgb, roi, centro, cor):
    """Desenha o retângulo da ROI e o centro detectado (se houver) direto no frame."""
    x1, x2, y1, y2 = roi
    cv.rectangle(frame_rgb, (x1, y1), (x2, y2), cor, 1)
    if centro is not None:
        cv.circle(frame_rgb, centro, 4, cor, -1)


def _iniciar_debug_window():
    """Tenta abrir a janela de debug. Se não houver display (modo headless),
    desativa o debug em vez de travar o processo com erro de X11."""
    try:
        cv.namedWindow("line_cam debug", cv.WINDOW_NORMAL)
        return True
    except cv.error as e:
        print(f"[line_cam] DEBUG desativado (sem display disponível): {e}")
        return False


def capturar_e_processar():
    shm = SharedMemory(name=mgr.shm.name)
    frame_buf = np.ndarray(FRAME_SHAPE, dtype=np.uint8, buffer=shm.buf)

    debug_ativo = DEBUG and _iniciar_debug_window()

    try:
        picam2 = Picamera2(camera_num=CAMERA_NUM)
        config = picam2.create_video_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(0.01)
    except Exception as e:
        print(f"ERRO ao iniciar a Picamera2: {e}")
        mgr.camera_ok.value = 0
        shm.close()
        return

    mgr.camera_ok.value = 1
    center_x = FRAME_WIDTH // 2

    try:
        while not mgr.terminate.is_set():
            # 1) DETECTAR FRAME
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

            # 2) VERDE + 3) PRETO (ambos calculados dentro de processar_linha_com_verde,
            # nessa ordem: verde primeiro, preto depois)
            (acao, cx_alvo, status_linha,
             centro_preto_esq, centro_preto_dir,
             centro_verde_esq, centro_verde_dir) = processar_linha_com_verde(frame_rgb)

            mgr.line_status.value = status_linha
            if cx_alvo is not None:
                mgr.line_angle.value = cx_alvo - center_x
                mgr.cx_alvo_v.value = cx_alvo

            if debug_ativo:
                frame_debug = frame_rgb.copy()
                _desenhar_debug(frame_debug, ROI_ESQ, centro_preto_esq, (255, 0, 0))
                _desenhar_debug(frame_debug, ROI_DIR, centro_preto_dir, (0, 0, 255))
                _desenhar_debug(frame_debug, ROI_ESQ, centro_verde_esq, (0, 255, 0))
                _desenhar_debug(frame_debug, ROI_DIR, centro_verde_dir, (0, 255, 0))
                if cx_alvo is not None:
                    cv.line(frame_debug, (center_x, 0), (center_x, FRAME_HEIGHT), (255, 255, 0), 1)
                    cv.circle(frame_debug, (cx_alvo, 10), 5, (0, 255, 255), -1)

                status_txt = f"{'LINE_FOUND' if status_linha == LINE_FOUND else 'LINE_LOST'} | {acao}"
                cv.putText(frame_debug, status_txt, (5, FRAME_HEIGHT - 8),
                           cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                cv.imshow("line_cam debug", cv.cvtColor(frame_debug, cv.COLOR_RGB2BGR))
                if cv.waitKey(1) & 0xFF == ord('q'):
                    debug_ativo = False
                    cv.destroyAllWindows()
    finally:
        mgr.camera_ok.value = 0
        if debug_ativo:
            cv.destroyAllWindows()
        picam2.stop()
        shm.close()