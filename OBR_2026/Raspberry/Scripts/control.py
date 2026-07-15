import time
from gpiozero import LED, PWMLED
from mp_manager import *

# ==========================================
# Configuração dos Pinos da Ponte H (L298N)
# ==========================================

# Motor A (Lado Esquerdo)
IN1 = 17
IN2 = 27
ENA = 18

# Motor B (Lado Direito)
IN3 = 22
IN4 = 23
ENB = 19

# Correções de hardware caso um motor gire mais rápido que o outro
left_correction = 1
right_correction = 1
max_turn_angle = 110

def steer(angle=190., speed=0.8):
    """
    Controla os motores baseado no ângulo (-180 a 180) e velocidade (0 a 1).
    190 é o código para PARAR.
    200 é o código para RÉ.
    """
    speed_left.value = 0
    speed_right.value = 0

    # Para os motores
    if angle == 190:
        forward_right.off()
        backward_right.off()
        forward_left.off()
        backward_left.off()
        speed_left.value = 0
        speed_right.value = 0

    # Movimento para trás (Ré)
    elif angle == 200:
        forward_right.on()
        backward_right.off()
        forward_left.on()
        backward_left.off()
        speed_left.value = max(speed * left_correction, 0)
        speed_right.value = max(speed * right_correction, 0)

    # Movimento para frente e curvas
    elif angle in range(-180, 181):
        forward_right.off()
        backward_right.on()
        forward_left.off()
        backward_left.on()

        # Curva para a Direita
        if angle >= 0:
            if angle > max_turn_angle: # Curva muito fechada (gira no próprio eixo)
                forward_right.on()
                backward_right.off()
                forward_left.off()
                backward_left.on()
                speed_left.value = min(speed * left_correction * 1.2, 1)
                speed_right.value = min(speed * right_correction * 1.2, 1)
            else: # Curva suave
                speed_left.value = min(speed * left_correction, 1)
                speed_right.value = min(speed * right_correction * ((max_turn_angle - angle) / (max_turn_angle - 1)), 1)

        # Curva para a Esquerda
        else:
            if angle < -max_turn_angle: # Curva muito fechada (gira no próprio eixo)
                forward_right.off()
                backward_right.on()
                forward_left.on()
                backward_left.off()
                speed_left.value = min(speed * left_correction * 1.2, 1)
                speed_right.value = min(speed * right_correction * 1.2, 1)
            else: # Curva suave
                speed_left.value = min(speed * left_correction * ((max_turn_angle + angle) / (max_turn_angle - 1)), 1)
                speed_right.value = min(speed * right_correction, 1)

def control_loop():
    global forward_left, backward_left, speed_left
    global forward_right, backward_right, speed_right

    print("[CONTROL] Iniciando configuração dos motores...")

    # Instanciando GPIOs - Lado Esquerdo (Motor A)
    forward_left = LED(IN1)
    backward_left = LED(IN2)
    speed_left = PWMLED(ENA, frequency=1000)

    # Instanciando GPIOs - Lado Direito (Motor B)
    forward_right = LED(IN3)
    backward_right = LED(IN4)
    speed_right = PWMLED(ENB, frequency=1000)

    time.sleep(.5)
    print("[CONTROL] Motores prontos. Aguardando comandos da câmera.")

    # Loop principal de controle
    while not terminate.value:
        # Se a variável 'run' (do mp_manager) estiver verdadeira, segue a linha
        if run.value:
            # Lê o ângulo atual calculado pela câmera
            current_angle = line_angle.value
            
            # Define uma velocidade base (ex: 70%). Você pode criar uma lógica dinâmica depois.
            base_speed = 0.7 
            
            # Se a linha foi detectada, dirige. Caso contrário, para.
            if line_detected.value:
                steer(current_angle, base_speed)
            else:
                steer(190) # Para
        else:
            steer(190) # Para
            
        time.sleep(0.01) # Pequeno atraso para aliviar o processador