import cv2
from picamera2 import Picamera2
from libcamera import controls
import numpy as np
import time

# NÃO cria a câmera aqui no import do módulo. Se este módulo for importado
# no processo principal ANTES do fork() do multiprocessing, o processo
# filho herdaria um handle de câmera já aberto -- e isso trava/quebra.
# A câmera só pode ser aberta dentro do processo que vai efetivamente usá-la.
camera = None

camera_x = 448
camera_y = 252
ponto_central_camera_x = camera_x // 2

preto_min = np.array([0, 0, 0])
preto_max = np.array([90, 90, 90])

_ultimo_erro = 0
velocidade_anterior = 0.0


def iniciar_camera():
    """Deve ser chamada UMA VEZ, dentro do processo que vai usar a câmera
    (o processo_camera.py), nunca no processo principal antes do fork."""
    global camera

    camera = Picamera2()
    mode = camera.sensor_modes[0]
    camera.configure(camera.create_video_configuration(
        sensor={'output_size': mode['size'], 'bit_depth': mode['bit_depth']}
    ))
    camera.start()
    camera.set_controls({
        "AfMode": controls.AfModeEnum.Manual,
        "LensPosition": 6.5,
        "FrameDurationLimits": (1000000 // 50, 1000000 // 50)  # 50 FPS
    })
    time.sleep(0.1)


def parar_camera():
    if camera is not None:
        camera.stop()


def capturar_frame():
    try:
        raw_capture = camera.capture_array()
    except Exception as e:
        print(f"Erro ao capturar frame: {e}")
        return None

    if raw_capture is None:
        return None

    raw_capture = cv2.resize(raw_capture, (camera_x, camera_y))
    rgb_img = cv2.cvtColor(raw_capture, cv2.COLOR_RGBA2RGB)
    return rgb_img


def _centro_maior_contorno(mask, offset_x=0, offset_y=0):
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours = [c for c in contours if cv2.contourArea(c) > 100]

    if len(contours) == 0:
        return None

    maior_contorno = max(contours, key=cv2.contourArea)
    M = cv2.moments(maior_contorno)

    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"]) + offset_x
    cy = int(M["m01"] / M["m00"]) + offset_y
    return (cx, cy)


def detectar_linha_perto(rgb_img):
    roi = rgb_img[232:252, 0:448]
    mask = cv2.inRange(roi, preto_min, preto_max)
    return _centro_maior_contorno(mask, offset_x=0, offset_y=232)


def detectar_linha_longe(rgb_img):
    roi = rgb_img[0:100, 0:448]
    mask = cv2.inRange(roi, preto_min, preto_max)
    return _centro_maior_contorno(mask, offset_x=0, offset_y=0)


def detectar_linha_esq(rgb_img):
    roi_esq = rgb_img[0:252, 0:112]
    mask = cv2.inRange(roi_esq, preto_min, preto_max)
    centro = _centro_maior_contorno(mask, offset_x=0, offset_y=0)

    if centro is None:
        return None

    cx_esq, cy_esq = centro
    pixel = rgb_img[cy_esq, 0]  # coluna 0 = borda esquerda da imagem
    dentro_da_faixa = np.all(pixel >= preto_min) and np.all(pixel <= preto_max)

    if dentro_da_faixa:
        cx_esq = 0

    return (cx_esq, cy_esq)


def detectar_linha_dir(rgb_img):
    roi_dir = rgb_img[0:252, 336:448]
    mask = cv2.inRange(roi_dir, preto_min, preto_max)
    centro = _centro_maior_contorno(mask, offset_x=336, offset_y=0)

    if centro is None:
        return None

    cx_dir, cy_dir = centro
    # BUG ORIGINAL: lia rgb_img[cy_dir, 0] (borda ESQUERDA) para checar a
    # borda DIREITA. Tem que ler a última coluna (camera_x - 1).
    pixel = rgb_img[cy_dir, camera_x - 1]
    dentro_da_faixa = np.all(pixel >= preto_min) and np.all(pixel <= preto_max)

    if dentro_da_faixa:
        cx_dir = camera_x

    return (cx_dir, cy_dir)


def calcular_potencia(erro):
    """Retorna a velocidade (0.0 - 1.0) que o motor deve usar, com base
    na magnitude do erro. Quem aplica isso no motor de fato é o control.py.
    """
    global velocidade_anterior

    erro_abs = abs(erro)
    if erro_abs <= 5:
        velocidade = 0.9
    elif erro_abs <= 15:
        velocidade = 0.30
    elif erro_abs <= 30:
        velocidade = 0.45
    elif erro_abs <= 50:
        velocidade = 0.60
    elif erro_abs <= 80:
        velocidade = 0.75
    else:
        velocidade = 0.9

    velocidade_anterior = velocidade
    return velocidade


# Guarda os pontos/centros detectados no ÚLTIMO frame processado, só pra
# quem quiser desenhar overlay de debug (desenhar_debug). Não interfere em
# nada da lógica de controle -- é só "memória" pra visualização.
_debug_info = {
    "centro_perto": None,
    "centro_longe": None,
    "centro_esq": None,
    "centro_dir": None,
}


def deteccao_erro(rgb_img):
    global _ultimo_erro

    centro_perto = detectar_linha_perto(rgb_img)
    centro_longe = detectar_linha_longe(rgb_img)
    centro_esq = detectar_linha_esq(rgb_img)
    centro_dir = detectar_linha_dir(rgb_img)

    _debug_info["centro_perto"] = centro_perto
    _debug_info["centro_longe"] = centro_longe
    _debug_info["centro_esq"] = centro_esq
    _debug_info["centro_dir"] = centro_dir

    achou_perto = centro_perto is not None
    achou_longe = centro_longe is not None
    achou_esq = centro_esq is not None
    achou_dir = centro_dir is not None

    if not achou_perto:
        return None, 0, 0.0

    cx_perto, _ = centro_perto

    if achou_longe:
        cx_referencia, _ = centro_longe
    elif achou_esq and achou_dir:
        direcao_ambigua = "reto" if abs(_ultimo_erro) <= 5 else ("direita" if _ultimo_erro > 0 else "esquerda")
        return direcao_ambigua, _ultimo_erro, calcular_potencia(_ultimo_erro)
    elif achou_esq:
        # só achou na lateral esquerda -> linha está saindo/curvando pra esquerda
        cx_referencia, _ = centro_esq
    elif achou_dir:
        # só achou na lateral direita -> linha está saindo/curvando pra direita
        cx_referencia, _ = centro_dir
    else:
        # nada encontrado em longe/esq/dir -> usa só o ROI de perto
        cx_referencia = cx_perto

    diferenca_x = cx_perto - cx_referencia
    limite = 40  # em pixels, ajuste conforme calibração

    if abs(diferenca_x) > limite:
        erro = cx_referencia - ponto_central_camera_x
    else:
        erro = cx_perto - ponto_central_camera_x

    _ultimo_erro = erro

    if erro > 5:
        direcao = "direita"
    elif erro < -5:
        direcao = "esquerda"
    else:
        direcao = "reto"

    potencia = calcular_potencia(erro)

    return direcao, erro, potencia


def desenhar_debug(rgb_img, direcao, erro, potencia):
    """Desenha por cima de uma CÓPIA do frame: as ROIs usadas na detecção,
    os centros encontrados em cada uma, e um texto com direção/erro/potência.
    Não usa nada de multiprocessing -- é só desenho, o cv2.imshow fica por
    conta de quem chamar isso (processo_camera.py)."""
    img_debug = rgb_img.copy()

    # ROIs (mesmas coordenadas usadas nas funções detectar_linha_*)
    cv2.rectangle(img_debug, (0, 232), (448, 252), (0, 255, 255), 1)   # perto
    cv2.rectangle(img_debug, (0, 0), (448, 100), (255, 255, 0), 1)     # longe
    cv2.rectangle(img_debug, (0, 0), (112, 252), (255, 0, 255), 1)     # esquerda
    cv2.rectangle(img_debug, (336, 0), (448, 252), (255, 0, 255), 1)   # direita

    cores = {
        "centro_perto": (0, 0, 255),
        "centro_longe": (0, 255, 0),
        "centro_esq": (255, 0, 0),
        "centro_dir": (255, 0, 0),
    }
    for nome, cor in cores.items():
        centro = _debug_info.get(nome)
        if centro is not None:
            cv2.circle(img_debug, centro, 5, cor, -1)

    # linha central de referência da câmera
    cv2.line(img_debug, (ponto_central_camera_x, 0), (ponto_central_camera_x, camera_y), (200, 200, 200), 1)

    texto = f"dir={direcao} erro={erro} pot={potencia:.2f}"
    cv2.putText(img_debug, texto, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    return img_debug
