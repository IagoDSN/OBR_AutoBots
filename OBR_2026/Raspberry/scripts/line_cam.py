
import cv2 as cv
import numpy as np
from multiprocessing import Process, Value, Lock
from multiprocessing.shared_memory import SharedMemory
import time
from picamera2 import Picamera2

# ---- liga/desliga a parte visual e os prints -------------------------
DEBUG = True  # True = mostra janela + prints | False = so roda o controle

CAMERA_NUM = 0  # indice da camera na Picamera2 (0 = unica/primeira camera CSI)

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

# ---- parametros do controle de motor (ajuste/tune aqui) --------------
KP = 0.6          # ganho proporcional -- TODO: tunar igual ja fez no control.py
BASE_SPEED = 60.0  # velocidade base em % (0-100)


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


def calcular_comando_motor(status, erro, center_x):
   
    if status == LINE_LOST:
        return 0.0, 0.0

    correcao = KP * (erro / center_x)  # normaliza o erro pela metade da largura
    vel_esq = BASE_SPEED + correcao * BASE_SPEED
    vel_dir = BASE_SPEED - correcao * BASE_SPEED

    vel_esq = max(-100.0, min(100.0, vel_esq))
    vel_dir = max(-100.0, min(100.0, vel_dir))
    return vel_esq, vel_dir


def capturar_e_processar(shm_name, frame_lock, novo_frame_flag, camera_ok,
                          line_status, line_angle, cx_alvo_v):
    """Processo da camera: abre a Picamera2, captura, escreve na
    shared_memory e calcula o erro/status pra quem estiver controlando
    os motores (control.py)."""
    shm = SharedMemory(name=shm_name)
    frame_buf = np.ndarray(FRAME_SHAPE, dtype=np.uint8, buffer=shm.buf)

    try:
    except ImportError as e:
        print(f"ERRO: picamera2 nao esta instalado/disponivel ({e})")
        camera_ok.value = 0
        shm.close()
        return

    try:
        picam2 = Picamera2(camera_num=CAMERA_NUM)
        config = picam2.create_video_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
        )
        picam2.configure(config)
        picam2.start()
        time.sleep(0.3)  # da um tempinho pro sensor estabilizar exposicao/AWB
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
                # Apesar do nome, o formato "RGB888" da Picamera2 entrega os
                # bytes em ordem B,G,R (quirk conhecido do libcamera) -- por
                # isso tratamos igual sairia de um cv.VideoCapture (BGR) e
                # convertemos pra RGB do mesmo jeito.
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
                line_angle.value = alvo_x - center_x  # erro em pixels ate o meio da tela
                cx_alvo_v.value = alvo_x
            else:
                line_status.value = LINE_LOST
                # mantem o ultimo line_angle.value ate a suavizacao temporal decidir
    finally:
        picam2.stop()
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
        print("ERRO: nao consegui iniciar a Picamera2.")
        print("Verifique se a fita flex esta bem conectada, se 'picamera2' esta")
        print("instalado e se nenhum outro processo esta usando a camera.")
        p.terminate()
        p.join()
        shm.close()
        shm.unlink()
        raise SystemExit(1)

    frame_view = np.ndarray(FRAME_SHAPE, dtype=np.uint8, buffer=shm.buf)
    center_x = FRAME_WIDTH // 2

    try:
        while True:
            if camera_ok.value == 0:
                print("ERRO: a camera desconectou / parou de responder")
                enviar_para_motor(0.0, 0.0)  # para os motores por seguranca
                break

            if novo_frame_flag.value:
                with frame_lock:
                    frame_local = frame_view.copy()
                    novo_frame_flag.value = 0
                if DEBUG:
                    cv.imshow("Linha", cv.cvtColor(frame_local, cv.COLOR_RGB2BGR))
                    if cv.waitKey(1) & 0xFF == ord('q'):
                        break

            # ---- aqui eh onde o erro vira comando de motor -----------
            status = line_status.value
            erro = line_angle.value
            vel_esq, vel_dir = calcular_comando_motor(status, erro, center_x)

            if DEBUG:
                print(f"status={status} erro={erro:.1f} cx_alvo={cx_alvo_v.value} "
                      f"vel_esq={vel_esq:.1f} vel_dir={vel_dir:.1f}")

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        enviar_para_motor(0.0, 0.0)  # garante que os motores param ao sair
        p.terminate()
        p.join()
        shm.close()
        shm.unlink()
        if DEBUG:
            cv.destroyAllWindows()
