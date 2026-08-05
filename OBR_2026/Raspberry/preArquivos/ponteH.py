"""
PonteH.py
Controle de duas BTS7960 (4 motores) para Raspberry Pi 5

Ligação BTS7960 ESQUERDA
-----------------------------------
RPWM -> GPIO18
LPWM -> GPIO19
R_EN -> GPIO23
L_EN -> GPIO24
VCC  -> 5V Raspberry
GND  -> GND Raspberry

Ligação BTS7960 DIREITA
-----------------------------------
RPWM -> GPIO12
LPWM -> GPIO13
R_EN -> GPIO16
L_EN -> GPIO20
VCC  -> 5V Raspberry
GND  -> GND Raspberry

Alimentação
-----------------------------------
Bateria + -> B+ das duas BTS7960
Bateria - -> B- das duas BTS7960

Motor esquerdo -> M+ M- da BTS7960 esquerda
Motor direito  -> M+ M- da BTS7960 direita

IMPORTANTE:
O GND da bateria deve estar ligado ao GND do Raspberry.
"""

from gpiozero import PWMOutputDevice, DigitalOutputDevice


class PonteH:

    def __init__(self):

        # =============================
        # Ponte esquerda
        # =============================

        self.rpwm_e = PWMOutputDevice(18, frequency=1000)
        self.lpwm_e = PWMOutputDevice(19, frequency=1000)

        self.ren_e = DigitalOutputDevice(23)
        self.len_e = DigitalOutputDevice(24)

        # =============================
        # Ponte direita
        # =============================

        self.rpwm_d = PWMOutputDevice(12, frequency=1000)
        self.lpwm_d = PWMOutputDevice(13, frequency=1000)

        self.ren_d = DigitalOutputDevice(16)
        self.len_d = DigitalOutputDevice(20)

        self.iniciar()

    ####################################################

    def iniciar(self):

        self.ren_e.on()
        self.len_e.on()

        self.ren_d.on()
        self.len_d.on()

        self.parar()

    ####################################################

    def parar(self):

        self.rpwm_e.value = 0
        self.lpwm_e.value = 0

        self.rpwm_d.value = 0
        self.lpwm_d.value = 0

    ####################################################

    def _motor_esquerdo(self, velocidade):

        velocidade = max(-1.0, min(1.0, velocidade))

        if velocidade > 0:

            self.lpwm_e.value = 0
            self.rpwm_e.value = velocidade

        elif velocidade < 0:

            self.rpwm_e.value = 0
            self.lpwm_e.value = -velocidade

        else:

            self.rpwm_e.value = 0
            self.lpwm_e.value = 0

    ####################################################

    def _motor_direito(self, velocidade):

        velocidade = max(-1.0, min(1.0, velocidade))

        if velocidade > 0:

            self.lpwm_d.value = 0
            self.rpwm_d.value = velocidade

        elif velocidade < 0:

            self.rpwm_d.value = 0
            self.lpwm_d.value = -velocidade

        else:

            self.rpwm_d.value = 0
            self.lpwm_d.value = 0

    ####################################################
    # FUNÇÃO CHAMADA PELO PID
    ####################################################

    def controlar(self, vel_esquerda, vel_direita):
        """
        vel_esquerda : -1.0 até 1.0
        vel_direita  : -1.0 até 1.0

        Exemplos

        controlar(0.6,0.6)
        controlar(0.8,0.5)
        controlar(0.5,0.8)
        controlar(-0.4,-0.4)
        """

        self._motor_esquerdo(vel_esquerda)
        self._motor_direito(vel_direita)

    ####################################################
    # AUXILIAR PARA PID
    ####################################################

    def aplicar_pid(self, velocidade_base, correcao):
        """
        velocidade_base : 0.0 a 1.0

        correcao:
            positiva  -> vira direita
            negativa  -> vira esquerda
        """

        esquerda = velocidade_base + correcao
        direita = velocidade_base - correcao

        esquerda = max(-1.0, min(1.0, esquerda))
        direita = max(-1.0, min(1.0, direita))

        self.controlar(esquerda, direita)