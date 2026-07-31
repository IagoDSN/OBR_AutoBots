import cv2
from picamera2 import Picamera2
from libcamera import controls
import numpy as np
import time


camera = None

camera_x = 448
camera_y = 252
ponto_central_camera_x = camera_x // 2

preto_min = np.array([0, 0, 0])
preto_max = np.array([90, 90, 90])

_ultimo_erro = 0
velocidade_anterior = 0.0


def iniciar_camera():
    
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
   
    pixel = rgb_img[cy_dir, camera_x - 1]
    dentro_da_faixa = np.all(pixel >= preto_min) and np.all(pixel <= preto_max)

    if dentro_da_faixa:
        cx_dir = camera_x

    return (cx_dir, cy_dir)


def calcular_potencia(erro):
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


def deteccao_erro(rgb_img):
    global _ultimo_erro

    centro_perto = detectar_linha_perto(rgb_img)
    centro_longe = detectar_linha_longe(rgb_img)
    centro_esq = detectar_linha_esq(rgb_img)
    centro_dir = detectar_linha_dir(rgb_img)

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
        
        cx_referencia, _ = centro_esq
    elif achou_dir:
        
        cx_referencia, _ = centro_dir
    else:
        
        cx_referencia = cx_perto

    diferenca_x = cx_perto - cx_referencia
    limite = 40  

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
