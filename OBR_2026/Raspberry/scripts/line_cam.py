"""
Versao FUNCIONAL (nao so debug) do pipeline de seguidor de linha usando
camera USB, com a mesma arquitetura de producao do line_cam.py: processo
dedicado de captura, frame compartilhado via shared_memory, e status/erro
expostos via multiprocessing.Value para o processo de controle (control.py) ler.
"""
import cv2 as cv
import numpy as np
from multiprocessing import Process, Value, Lock
from multiprocessing.shared_memory import SharedMemory
import time

CAMERA_INDEX = 0  # troque se tiver mais de uma camera USB conectada

FRAME_WIDTH = 320
FRAME_HEIGHT = 200
FRAME_SHAPE = (FRAME_HEIGHT, FRAME_WIDTH, 3)
FRAME_NBYTES = int(np.prod(FRAME_SHAPE))  # uint8 -> 1 byte por elemento

BLACK_THRESH = 60  # pixel eh "preto" se R, G e B estiverem todos abaixo disso
MIN_CONTOUR_AREA = 80

# (x1, x2, y1, y2) -- as 3 tiles ocupam a faixa y=0..50 lado a lado
ROI_ESQUERDA_CIMA = (0,   60,  0, 50)
ROI_CIMA          = (60, 260,  0, 50)
ROI_DIREITA_CIMA  = (260, 320, 0, 50)

LINE_LOST = 0
LINE_FOUND = 1


def mascara_preto(frame_rgb):
    """255 onde os 3 canais (R,G,B) estao abaixo do threshold -> pixel preto."""
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


def capturar_e_processar(shm_name, frame_lock, novo_frame_flag, camera_ok,
                          line_status, line_angle, cx_alvo_v):
    """Processo da camera: abre a webcam USB, captura, escreve na
    shared_memory e calcula o erro/status pra quem estiver controlando
    os motores (control.py)."""
    shm = SharedMemory(name=shm_name)
    frame_buf = np.ndarray(FRAME_SHAPE, dtype=np.uint8, buffer=shm.buf)

    cap = cv.VideoCapture(CAMERA_INDEX)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        camera_ok.value = 0  # avisa o processo pai que a camera nao abriu
        shm.close()
        return

    camera_ok.value = 1
    center_x = FRAME_WIDTH // 2

    try:
        while True:
            ok, frame_bgr = cap.read()
            if not ok:
                camera_ok.value = 0  # avisa que a camera caiu no meio da execucao
                line_status.value = LINE_LOST
                break

            frame = cv.resize(frame_bgr, (FRAME_WIDTH, FRAME_HEIGHT))
            frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

            with frame_lock:
                frame_buf[:] = frame_rgb
                novo_frame_flag.value = 1

            centro_cima = detectar_centro(frame_rgb, ROI_CIMA)
            centro_esq = detectar_centro(frame_rgb, ROI_ESQUERDA_CIMA)
            centro_dir = detectar_centro(frame_rgb, ROI_DIREITA_CIMA)

            alvo_x = escolher_alvo(centro_cima, centro_esq, centro_dir, center_x)

            if alvo_x is not None:
                line_status.value = LINE_FOUND
                line_angle.value = alvo_x - center_x  # erro em pixels ate o meio da tela
                cx_alvo_v.value = alvo_x
            else:
                line_status.value = LINE_LOST
                # mantem o ultimo line_angle.value ate a suavizacao temporal decidir
    finally:
        cap.release()
        shm.close()


if __name__ == "__main__":
    shm = SharedMemory(create=True, size=FRAME_NBYTES)
    frame_lock = Lock()
    novo_frame_flag = Value('i', 0)
    camera_ok = Value('i', 0)

    line_status = Value('i', LINE_LOST)
    line_angle = Value('d', 0.0)  # erro em pixels (nao angulo em graus)
    cx_alvo_v = Value('i', -1)

    p = Process(
        target=capturar_e_processar,
        args=(shm.name, frame_lock, novo_frame_flag, camera_ok,
              line_status, line_angle, cx_alvo_v),
        daemon=True
    )
    p.start()

    # espera o processo filho tentar abrir a camera antes de seguir
    for _ in range(50):  # ate ~2.5s
        if camera_ok.value != 0:
            break
        time.sleep(0.05)

    if camera_ok.value == 0:
        print(f"ERRO: nao consegui abrir a camera USB no indice {CAMERA_INDEX}")
        print("Tente CAMERA_INDEX = 1, 2... se tiver mais de uma camera.")
        p.terminate()
        p.join()
        shm.close()
        shm.unlink()
        raise SystemExit(1)

    frame_view = np.ndarray(FRAME_SHAPE, dtype=np.uint8, buffer=shm.buf)

    try:
        while True:
            if camera_ok.value == 0:
                print("ERRO: a camera desconectou / parou de responder")
                break

            if novo_frame_flag.value:
                with frame_lock:
                    frame_local = frame_view.copy()
                    novo_frame_flag.value = 0
                cv.imshow("Linha", cv.cvtColor(frame_local, cv.COLOR_RGB2BGR))
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break

            print(f"status={line_status.value} erro={line_angle.value:.1f} "
                  f"cx_alvo={cx_alvo_v.value}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        p.terminate()
        p.join()
        shm.close()
        shm.unlink()
        cv.destroyAllWindows()
