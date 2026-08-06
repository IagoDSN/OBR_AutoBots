import time
from gpiozero import Motor, DigitalOutputDevice
from line_cam import calcular_comando_motor  
cc = calcular_comando_motor 
# --- Pinos (BCM) — confira contra sua fiação real ---
# Motor direito
motor_dir = Motor(forward=18, backward=19)
en_dir = DigitalOutputDevice(23, initial_value=True)

# Motor esquerdo
motor_esq = Motor(forward=20, backward=21)
en_esq = DigitalOutputDevice(24, initial_value=True)

KP = 0.6                 # ajusta depois de testar
VELOCIDADE_BASE = 0.6    # gpiozero usa -1.0 a 1.0, não 0-255


def aplicar_comando(erro):
    vel_esq, vel_dir = calcular_comando_motor(erro)
    motor_esq.value = cc.vel_esq
    motor_dir.value = cc.vel_dir

def parar():
    motor_esq.stop()
    motor_dir.stop()

if __name__ == "__main__":
    try:
        while True:
            erro = 0
            aplicar_comando(erro)
            time.sleep(0.01)  # loop rápido, mas não 100% CPU

    except KeyboardInterrupt:
        pass
    finally:
        parar()

