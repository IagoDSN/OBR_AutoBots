from __future__ import annotations

import logging
import queue
import threading
from dataclasses import dataclass

import cv2
import numpy as np
from gpiozero import OutputDevice, PWMOutputDevice
from libcamera import Transform
from picamera2 import Picamera2

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("line_follower")


# ----------------------------------------------------------------------------
# 1. CONFIGURAÇÃO — mude parâmetros aqui, sem precisar mexer no resto do código
# ----------------------------------------------------------------------------
@dataclass(frozen=True)
class Config:
    # Pinos - Motor A (Esquerdo)
    IN1_PIN: int = 17
    IN2_PIN: int = 27
    ENA_PIN: int = 18  # PWM

    # Pinos - Motor B (Direito)
    IN3_PIN: int = 22
    IN4_PIN: int = 23
    ENB_PIN: int = 19  # PWM

    # Câmera
    FRAME_WIDTH: int = 320
    FRAME_HEIGHT: int = 180
    ROTATE_180: bool = True

    # Visão computacional (faixa HSV considerada "linha preta")
    HSV_LOWER: tuple = (0, 0, 0)
    HSV_UPPER: tuple = (180, 255, 50)
    ERODE_ITER: int = 5
    DILATE_ITER: int = 9
    OFF_BOTTOM_Y_THRESHOLD: int = 358  # ponto considerado "saindo pela base da imagem"

    # Controle PD
    KP: float = 0.75  # Ganho Proporcional
    KD: float = 0.0   # Ganho Derivativo. Comece baixo (0.05-0.3) e suba aos
                       # poucos: deve reduzir oscilação sem criar tremores.
    SETPOINT_X: int = 160  # centro da imagem (FRAME_WIDTH / 2)
    BASE_SPEED: int = 60   # 0 a 100

    SHOW_PREVIEW: bool = True  # desligue se rodar sem monitor (headless) —
                                # também evita alocar o buffer de preview
