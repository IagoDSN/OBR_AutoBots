from gpiozero import Motor

# Mesma lógica da câmera: os pinos GPIO só são reservados quando
# iniciar_motores() for chamada dentro do processo_motor.py, não no import
# deste módulo no processo principal.
motor_esquerdo = None
motor_direito = None


def iniciar_motores():
    global motor_esquerdo, motor_direito
    # motor_esquerdo -> usa IN1/IN2/ENA
    motor_esquerdo = Motor(forward=17, backward=27, enable=18, pwm=True)
    # motor_direito -> usa IN3/IN4/ENB
    motor_direito = Motor(forward=22, backward=23, enable=13, pwm=True)


def frente(velocidade):
    motor_esquerdo.forward(velocidade)
    motor_direito.forward(velocidade)


def girar_direita(velocidade):
    motor_esquerdo.forward(velocidade)
    motor_direito.backward(velocidade)


def girar_esquerda(velocidade):
    motor_esquerdo.backward(velocidade)
    motor_direito.forward(velocidade)


def parar():
    motor_esquerdo.stop()
    motor_direito.stop()


def aplicar_comando(direcao, velocidade):
    """Recebe direcao ("direita"/"esquerda"/"reto") e velocidade (0.0-1.0)
    calculadas pelo line_cam.py e aciona os motores."""
    if direcao == "direita":
        girar_direita(velocidade)
    elif direcao == "esquerda":
        girar_esquerda(velocidade)
    elif direcao == "reto":
        frente(velocidade)
    else:
        parar()
