import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import threading  # Biblioteca para rodar processos em segundo plano
from imutils.video import VideoStream  # Captura de câmera em thread separada

# ==========================================
# CONFIGURAÇÕES DE HARDWARE (GPIO)
# ==========================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pinos do Ultrassônico
TRIG = 5
ECHO = 6

# Motor Esquerdo (Motor A)
IN1 = 17
IN2 = 27
ENA = 18

# Motor Direito (Motor B)
IN3 = 22
IN4 = 23
ENB = 19

# Configura todos os pinos como saída/entrada
GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Inicializa o PWM nos pinos de Enable (Frequência de 100Hz)
pwm_esq = GPIO.PWM(ENA, 100)
pwm_dir = GPIO.PWM(ENB, 100)
pwm_esq.start(0)
pwm_dir.start(0)

# ==========================================
# VARIÁVEIS GLOBAIS E DE CONTROLE
# ==========================================
DISTANCIA_OBSTACULO = 6.0
GPIO.output(TRIG, GPIO.LOW)

# Variável global que a Thread do ultrassônico vai atualizar
distancia_global = 999.0

LARGURA = 160
ALTURA = 120
CENTRO_X = LARGURA // 2  

vs = VideoStream(src=0).start()
time.sleep(2.0)  

vs.stream.stream.set(cv2.CAP_PROP_FRAME_WIDTH, LARGURA)
vs.stream.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTURA)

def ler_frame():
    frame = vs.read()
    if frame is None:
        return None
    if frame.shape[1] != LARGURA or frame.shape[0] != ALTURA:
        frame = cv2.resize(frame, (LARGURA, ALTURA))
    return frame

# Variáveis do Controlador PID
kp = 0.75  # Proporcional 
ki = 0.01  # Integral (Atenção: talvez precise de reajuste por causa do dt)
kd = 0.30  # Derivativo (Atenção: talvez precise de reajuste por causa do dt)
ap = 1.0   # Ganho do ângulo da linha
erro_anterior = 0
soma_erro = 0

vel_base = 60
obs_perto = 20
obs_longe = 60
lado = 1  # 1 = esquerda, 0 = direita

kernel = np.ones((3, 3), np.uint8)

# Limites HSV globais
HSV_PRETO_BAIXO = np.array([0, 0, 0])
HSV_PRETO_ALTO = np.array([75, 75, 75])

HSV_VERDE_BAIXO = np.array([35, 65, 50])
HSV_VERDE_ALTO = np.array([85, 200, 200])

HSV_VERMELHO_BAIXO1 = np.array([0, 100, 100])
HSV_VERMELHO_ALTO1 = np.array([10, 255, 255])
HSV_VERMELHO_BAIXO2 = np.array([160, 100, 100])
HSV_VERMELHO_ALTO2 = np.array([180, 255, 255])

# ROIs 
ROI_VERMELHO = (0, ALTURA, 50, 110)          
ROI_VERDE_ESQ = (90, 117, 0, 55)             
ROI_VERDE_DIR = (90, 117, 105, 160)          
ROI_VERDE_CENTRO = (90, 117, 58, 103)        
ROI_GATILHO = (72, 96, 15, 145)              
LIMITE_PIXELS_GATILHO = 1200                 
LIMITE_ERRO_LINHA = 8                        

# ==========================================
# FUNÇÕES DE MOVIMENTAÇÃO
# ==========================================
def motores_frente():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(vel_base)
    pwm_dir.ChangeDutyCycle(vel_base)

def motores_tras():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_esq.ChangeDutyCycle(vel_base)
    pwm_dir.ChangeDutyCycle(vel_base)

def motores_parar():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(0)
    pwm_dir.ChangeDutyCycle(0)

def motores_virar_dir():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_esq.ChangeDutyCycle(vel_base)
    pwm_dir.ChangeDutyCycle(vel_base)

def motores_virar_esq():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(vel_base)
    pwm_dir.ChangeDutyCycle(vel_base)

def obs_esq():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(obs_longe)
    pwm_dir.ChangeDutyCycle(obs_perto)

def obs_dir():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(obs_perto)
    pwm_dir.ChangeDutyCycle(obs_longe)


# ==========================================
# LEITURA ASSÍNCRONA DO ULTRASSÔNICO
# ==========================================
def ler_distancia_fisica():
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, GPIO.LOW)

    pulso_inicio = time.time()
    timeout = pulso_inicio
    while GPIO.input(ECHO) == 0:
        pulso_inicio = time.time()
        if pulso_inicio - timeout > 0.02:
            return 999.0

    pulso_fim = time.time()
    timeout = pulso_fim
    while GPIO.input(ECHO) == 1:
        pulso_fim = time.time()
        if pulso_fim - timeout > 0.02:
            return 999.0

    duracao_pulso = pulso_fim - pulso_inicio
    distancia = duracao_pulso * 17150
    return round(distancia, 1)

def thread_ultrassonico():
    global distancia_global
    while True:
        distancia_global = ler_distancia_fisica()
        time.sleep(0.05)


# ==========================================
# FUNÇÕES DE LÓGICA DE PISTA (VERDE E DESVIO)
# ==========================================
def encontrar_linha():
    while True:
        # --- NOVO: Parada de emergência dentro do loop infinito ---
        if cv2.waitKey(1) & 0xFF == ord("q"):
            raise KeyboardInterrupt  # Joga para o bloco try/except principal para desligar seguro

        img_atual = ler_frame()
        if img_atual is None:
            time.sleep(0.005)  # --- NOVO: Evita travar a CPU em 100% (busy-waiting) ---
            continue

        hsv = cv2.cvtColor(img_atual, cv2.COLOR_BGR2HSV)
        Blackline = cv2.inRange(hsv, HSV_PRETO_BAIXO, HSV_PRETO_ALTO)
        contours_blk, _ = cv2.findContours(Blackline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours_blk) > 0:
            blackbox = cv2.minAreaRect(contours_blk[0])
            (x_min, _), _, _ = blackbox
            erro = int(x_min - CENTRO_X)

            if abs(erro) < LIMITE_ERRO_LINHA:
                motores_frente()
                return True
        
        time.sleep(0.005)  # --- NOVO: Fôlego para o processador ---

def esq_verde():
    motores_frente()
    time.sleep(0.4)
    motores_virar_esq()
    time.sleep(0.5)
    encontrar_linha()

def dir_verde():
    motores_frente()
    time.sleep(0.4)
    motores_virar_dir()
    time.sleep(0.5)
    encontrar_linha()

def duplo_verde():
    motores_frente()
    time.sleep(0.4)

    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_esq.ChangeDutyCycle(vel_base)
    pwm_dir.ChangeDutyCycle(vel_base)
    time.sleep(0.8)
    encontrar_linha()


def desvio_obs(lado_desvio=1):
    motores_tras()
    time.sleep(0.4)

    if lado_desvio == 1:
        motores_virar_esq()
        time.sleep(1.6)

        # Lado Esquerdo
        while True:
            # --- NOVO: Parada de emergência dentro do loop infinito ---
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt

            img_atual = ler_frame()
            if img_atual is None:
                time.sleep(0.005)  # --- NOVO: Fôlego para o processador ---
                continue

            hsv = cv2.cvtColor(img_atual, cv2.COLOR_BGR2HSV)
            Blackline = cv2.inRange(hsv, HSV_PRETO_BAIXO, HSV_PRETO_ALTO)
            
            # --- NOVO: Aplicando a mesma erosão no primeiro giro para evitar ruídos ---
            Blackline = cv2.erode(Blackline, kernel, iterations=1)

            contours_blk, _ = cv2.findContours(Blackline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours_blk) > 0:
                break
            obs_esq()
            time.sleep(0.005)  # --- NOVO: Fôlego para o processador ---

        time.sleep(1.0)

        # Retomando
        while True:
            # --- NOVO: Parada de emergência dentro do loop infinito ---
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt

            img_atual = ler_frame()
            if img_atual is None:
                time.sleep(0.005)  # --- NOVO: Fôlego para o processador ---
                continue

            hsv = cv2.cvtColor(img_atual, cv2.COLOR_BGR2HSV)
            Blackline = cv2.inRange(hsv, HSV_PRETO_BAIXO, HSV_PRETO_ALTO)
            Blackline = cv2.erode(Blackline, kernel, iterations=1) 
            contours_blk, _ = cv2.findContours(Blackline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours_blk) > 0:
                blackbox = cv2.minAreaRect(contours_blk[0])
                (x_min, _), _, _ = blackbox
                erro = int(x_min - CENTRO_X)
                if abs(erro) < LIMITE_ERRO_LINHA:
                    motores_frente()
                    return True
            obs_dir()
            time.sleep(0.005)  # --- NOVO: Fôlego para o processador ---


# ==========================================
# LOOP PRINCIPAL (VISÃO + PID)
# ==========================================
def seguir_linha():
    global erro_anterior, soma_erro
    motores_frente()

    y1_v, y2_v, x1_v, x2_v = ROI_VERMELHO
    y1_ge, y2_ge, x1_ge, x2_ge = ROI_VERDE_ESQ
    y1_gd, y2_gd, x1_gd, x2_gd = ROI_VERDE_DIR
    y1_gc, y2_gc, x1_gc, x2_gc = ROI_VERDE_CENTRO
    y1_g, y2_g, x1_g, x2_g = ROI_GATILHO

    # --- NOVO: Inicia a contagem de tempo para o PID (dt) ---
    tempo_anterior = time.time()

    while True:
        # --- NOVO: Cálculo do tempo exato entre frames (dt) ---
        tempo_atual = time.time()
        dt = tempo_atual - tempo_anterior
        if dt <= 0:
            dt = 0.001  # Evita erro de divisão por zero caso seja rápido demais
        tempo_anterior = tempo_atual

        image = ler_frame()
        if image is None:
            time.sleep(0.005)  # --- NOVO: Fôlego se pular frame
            continue 

        # 1. VERIFICAÇÃO DE OBSTÁCULO 
        if distancia_global <= DISTANCIA_OBSTACULO:
            desvio_obs(lado)
            continue

        # 2. VERIFICAÇÃO DE FIM DE PISTA (VERMELHO)
        roi_vermelho_bgr = image[y1_v:y2_v, x1_v:x2_v]
        roi_vermelho_hsv = cv2.cvtColor(roi_vermelho_bgr, cv2.COLOR_BGR2HSV)
        v_mask1 = cv2.inRange(roi_vermelho_hsv, HSV_VERMELHO_BAIXO1, HSV_VERMELHO_ALTO1)
        v_mask2 = cv2.inRange(roi_vermelho_hsv, HSV_VERMELHO_BAIXO2, HSV_VERMELHO_ALTO2)
        vermelho = cv2.bitwise_or(v_mask1, v_mask2)

        vermelho_sign, _ = cv2.findContours(vermelho, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(vermelho_sign) > 0:
            motores_parar()
            break

        # 3. VERIFICAÇÃO DE CRUZAMENTOS VERDES
        parte_inferior_esq_bgr = image[y1_ge:y2_ge, x1_ge:x2_ge]
        parte_inferior_dir_bgr = image[y1_gd:y2_gd, x1_gd:x2_gd]

        parte_inferior_esq_hsv = cv2.cvtColor(parte_inferior_esq_bgr, cv2.COLOR_BGR2HSV)
        parte_inferior_dir_hsv = cv2.cvtColor(parte_inferior_dir_bgr, cv2.COLOR_BGR2HSV)

        green_esq = cv2.inRange(parte_inferior_esq_hsv, HSV_VERDE_BAIXO, HSV_VERDE_ALTO)
        green_dir = cv2.inRange(parte_inferior_dir_hsv, HSV_VERDE_BAIXO, HSV_VERDE_ALTO)

        green_esq = cv2.erode(green_esq, kernel, iterations=1) 
        green_dir = cv2.erode(green_dir, kernel, iterations=1) 

        contornos_esq, _ = cv2.findContours(green_esq, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contornos_dir, _ = cv2.findContours(green_dir, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contornos_esq) > 0 and len(contornos_dir) > 0:
            parte_inferior_centro_bgr = image[y1_gc:y2_gc, x1_gc:x2_gc]
            parte_inferior_centro_hsv = cv2.cvtColor(parte_inferior_centro_bgr, cv2.COLOR_BGR2HSV)
            green_centro = cv2.inRange(parte_inferior_centro_hsv, HSV_VERDE_BAIXO, HSV_VERDE_ALTO)
            green_centro = cv2.erode(green_centro, kernel, iterations=1)
            contornos_centro, _ = cv2.findContours(green_centro, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contornos_centro) == 0:
                duplo_verde()
                continue
            else:
                motores_frente()
                time.sleep(0.25)
                continue

        elif len(contornos_esq) > 0:
            esq_verde()
            continue

        elif len(contornos_dir) > 0:
            dir_verde()
            continue

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 4. GATILHO DE LINHA HORIZONTAL PRETA
        regiao_gatilho = hsv[y1_g:y2_g, x1_g:x2_g]
        Preto_gatilho = cv2.inRange(regiao_gatilho, HSV_PRETO_BAIXO, HSV_PRETO_ALTO)
        pixels_pretos = np.sum(Preto_gatilho == 255)

        if pixels_pretos > LIMITE_PIXELS_GATILHO:
            motores_frente()
            time.sleep(0.25)
            continue

        # 5. CÁLCULO E CONTROLE PID DA LINHA PRETA
        Blackline = cv2.inRange(hsv, HSV_PRETO_BAIXO, HSV_PRETO_ALTO)
        Blackline = cv2.erode(Blackline, kernel, iterations=1)

        contours_blk, _ = cv2.findContours(Blackline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk_len = len(contours_blk)

        if contours_blk_len > 0:
            if contours_blk_len == 1:
                blackbox = cv2.minAreaRect(contours_blk[0])
            else:
                candidates = []
                for con_num in range(contours_blk_len):
                    blackbox = cv2.minAreaRect(contours_blk[con_num])
                    (x_min, y_min), _, _ = blackbox
                    candidates.append((y_min, con_num))

                candidates = sorted(candidates)
                con_highest = candidates[0][1]
                blackbox = cv2.minAreaRect(contours_blk[con_highest])

            (x_min, y_min), (w_min, h_min), ang = blackbox

            # Tratamento do Ângulo
            if ang < -45:
                ang = 90 + ang
            if w_min < h_min and ang > 0:
                ang = (90 - ang) * -1
            if w_min > h_min and ang < 0:
                ang = 90 + ang

            error = int(x_min - CENTRO_X)
            ang = int(ang)

            # --- NOVO: CÁLCULO PID COM DT ---
            # 1. Proporcional: Corrige o erro atual
            P = error * kp

            # 2. Integral: Corrige o erro acumulado (multiplicando pelo dt)
            soma_erro += error * dt
            soma_erro = max(-500, min(500, soma_erro))  
            I = soma_erro * ki

            # 3. Derivativo: Suaviza a correção baseado na variação real no tempo
            D = ((error - erro_anterior) / dt) * kd

            # Cálculo final combinando PID + Ganho de Ângulo
            calculo_steering = int(P + I + D + (ang * ap))

            # Atualiza o erro anterior para a próxima iteração
            erro_anterior = error

            # Aplica nos motores
            vel_esq = vel_base + calculo_steering
            vel_dir = vel_base - calculo_steering

            vel_esq = max(0, min(100, vel_esq))
            vel_dir = max(0, min(100, vel_dir))

            pwm_esq.ChangeDutyCycle(vel_esq)
            pwm_dir.ChangeDutyCycle(vel_dir)
        else:
            pwm_esq.ChangeDutyCycle(max(0, vel_base - 10))
            pwm_dir.ChangeDutyCycle(max(0, vel_base - 10))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            raise KeyboardInterrupt  # --- NOVO: Joga para o bloco principal fazer o desligamento correto ---


# ==========================================
# INICIALIZAÇÃO E TRATAMENTO DE ERROS
# ==========================================
try:
    t = threading.Thread(target=thread_ultrassonico, daemon=True)
    t.start()
    seguir_linha()
except KeyboardInterrupt:
    print("\nPrograma interrompido pelo usuário ou parada de emergência acionada.")
finally:
    motores_parar()
    vs.stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()
