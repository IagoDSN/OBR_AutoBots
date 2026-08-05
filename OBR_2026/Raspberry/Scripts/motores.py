import pigpio
import time

from debug import

debug = db

# GPIO do Raspberry Pi 5
RPWM = 18
LPWM = 19

REN = 23
LEN = 24


pi = pigpio.pi()


# habilita ponte H
pi.write(REN, 1)
pi.write(LEN, 1)
def cal_erro_P(db.erro):


def motor_frente(velocidade):
    # velocidade 0-255
    pi.set_PWM_dutycycle(RPWM, velocidade)
    pi.set_PWM_dutycycle(LPWM, 0)


def motores_parar(erro):
    pi.set_PWM_dutycicle(RPWM, 0)
    pi.set_PWM_dutycicle(LPWM, 0)

