# Pi 5 precisa do backend lgpio; forçar ANTES de criar Motor/DigitalOutputDevice
from gpiozero import Device
from gpiozero.pins.lgpio import LGPIOFactory
Device.pin_factory = LGPIOFactory()

from gpiozero import Motor, DigitalOutputDevice


class PonteHBTS7960:
    """Um motor controlado por uma ponte BTS7960.

    RPWM/LPWM -> velocidade e sentido (via gpiozero.Motor)
    R_EN/L_EN -> habilitação da ponte; ficam em HIGH e servem
                 como trava de emergência
    """

    def __init__(self, pino_rpwm, pino_lpwm, pino_ren, pino_len):
        self._motor = Motor(forward=pino_rpwm, backward=pino_lpwm, pwm=True)
        self._ren = DigitalOutputDevice(pino_ren, initial_value=True)
        self._len = DigitalOutputDevice(pino_len, initial_value=True)

    def set_velocidade(self, velocidade_pct):
        """velocidade_pct: -100 (ré máxima) a 100 (frente máxima)"""
        valor = max(-100.0, min(100.0, velocidade_pct)) / 100.0
        self._motor.value = valor

    def parar(self):
        self._motor.stop()

    def desabilitar(self):
        self._ren.off()
        self._len.off()

    def habilitar(self):
        self._ren.on()
        self._len.on()

    def fechar(self):
        self._motor.close()
        self._ren.close()
        self._len.close()