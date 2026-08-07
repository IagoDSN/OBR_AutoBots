import time

from gpiozero import Device
from gpiozero.pins.lgpio import LGPIOFactory
Device.pin_factory = LGPIOFactory()

from motores import PonteHBTS7960
import mp_manager as mgr
from constants import FRAME_WIDTH, LINE_LOST

RPWM_ESQ, LPWM_ESQ, REN_ESQ, LEN_ESQ = 12, 13, 5, 6
RPWM_DIR, LPWM_DIR, REN_DIR, LEN_DIR = 18, 19, 20, 21

KP = 0.6
BASE_SPEED = 60.0
CENTER_X = FRAME_WIDTH // 2


def calcular_comando_motor(status, erro, center_x=CENTER_X, kp=KP, base=BASE_SPEED):
    if status == LINE_LOST:
        return base, base

    correcao = kp * (erro / center_x)
    vel_esq = base + correcao * base
    vel_dir = base - correcao * base

    vel_esq = max(-100.0, min(100.0, vel_esq))
    vel_dir = max(-100.0, min(100.0, vel_dir))
    return vel_esq, vel_dir


def loop_controle():
    
    try:
        motor_esq = PonteHBTS7960(RPWM_ESQ, LPWM_ESQ, REN_ESQ, LEN_ESQ)
        motor_dir = PonteHBTS7960(RPWM_DIR, LPWM_DIR, REN_DIR, LEN_DIR)
    except Exception as e:
        print(f"[control] ERRO ao inicializar os motores (GPIO): {e}")
        return

    print("[control] Motores inicializados, loop de controle iniciado.")

    try:
        while not mgr.terminate.is_set():
            if not mgr.camera_ok.value:
                motor_esq.set_velocidade(0)
                motor_dir.set_velocidade(0)
                time.sleep(0.05)
                continue

            vel_esq, vel_dir = calcular_comando_motor(
                mgr.line_status.value, mgr.line_angle.value
            )
            motor_esq.set_velocidade(vel_esq)
            motor_dir.set_velocidade(vel_dir)
            time.sleep(0.02)
    finally:
        motor_esq.parar()
        motor_dir.parar()
        motor_esq.fechar()
        motor_dir.fechar()
        print("[control] Motores parados e GPIO liberado.")
