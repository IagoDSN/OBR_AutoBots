import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera import PiCamera
from picamera.array import PiRGBArray

# Configuração do GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Ultrassônico
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

# Configura todos os pinos como saída
GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Inicializa o PWM nos pinos de Enable (Frequência de 100Hz)
pwm_esq = GPIO.PWM(ENA, 100)
pwm_dir = GPIO.PWM(ENB, 100)

# Inicia o PWM com velocidade 0 (parado)
pwm_esq.start(0)
pwm_dir.start(0)

DISTANCIA_OBSTACULO = 6.0 
GPIO.output(TRIG, GPIO.LOW)

# Inicialização da Câmera
camera = PiCamera()
camera.resolution = (320, 200)
camera.rotation = 180 
rawCapture = PiRGBArray(camera, size=(320, 200))
time.sleep(0.1)

# Variáveis de controle e ganho
x_last = 160
y_last = 100
kp = 0.75
ap = 1  
vel_base = 60
obs_perto = 20
obs_longe = 60
lado = 1 # 1 = esquerda, 0 = direita

# Kernel global para morfologia matemática
kernel = np.ones((3, 3), np.uint8)


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

def esq_verde():
    motores_frente()
    time.sleep(0.4)
    motores_virar_esq()
    time.sleep(0.5) 
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img_atual = frame.array
        Blackline = cv2.inRange(img_atual, (0, 0, 0), (75, 75, 75))
        Blackline = cv2.erode(Blackline, kernel, iterations=2)
        contours_blk, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours_blk) > 0:
            blackbox = cv2.minAreaRect(contours_blk[0])
            (x_min, _), _, _ = blackbox
            erro = int(x_min - 160)
            if abs(erro) < 20:
                rawCapture.truncate(0)
                motores_frente() 
                return True 
        rawCapture.truncate(0)
    return False
                             
def dir_verde():
    motores_frente()
    time.sleep(0.4)
    motores_virar_dir()
    time.sleep(0.5)
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img_atual = frame.array
        Blackline = cv2.inRange(img_atual, (0, 0, 0), (75, 75, 75))
        Blackline = cv2.erode(Blackline, kernel, iterations=2)
        contours_blk, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours_blk) > 0:
            blackbox = cv2.minAreaRect(contours_blk[0])
            (x_min, _), _, _ = blackbox
            erro = int(x_min - 160)
            if abs(erro) < 20:
                rawCapture.truncate(0)
                motores_frente()
                return True 
        rawCapture.truncate(0)
    return False

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
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img_atual = frame.array
        Blackline = cv2.inRange(img_atual, (0, 0, 0), (75, 75, 75))
        contours_blk, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours_blk) > 0:
            blackbox = cv2.minAreaRect(contours_blk[0])
            (x_min, _), _, _ = blackbox
            erro = int(x_min - 160)
            if abs(erro) < 20:
                rawCapture.truncate(0)
                motores_frente()
                return True
        rawCapture.truncate(0)
    return False

def ler_distancia():
    GPIO.output(TRIG, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG, GPIO.LOW)
    
    pulso_inicio = time.time()
    timeout = pulso_inicio
    while GPIO.input(ECHO) == 0:
        pulso_inicio = time.time()
        if pulso_inicio - timeout > 0.02: # Segurança para não travar o loop
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

def desvio_obs(lado=1): # Lado padrão definido como 1 caso não seja enviado
    motores_tras()
    time.sleep(0.4)
    
    if lado == 1:
        motores_virar_esq()
        time.sleep(1.6)
        obs_esq()
        time.sleep(1.0)
        
        # Alinhamento pós desvio buscando a linha
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            img_atual = frame.array
            Blackline = cv2.inRange(img_atual, (0, 0, 0), (75, 75, 75))
            contours_blk, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_blk) > 0:
                break
            rawCapture.truncate(0)
            
        motores_frente()
        time.sleep(0.5)
        motores_virar_esq()
        
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            img_atual = frame.array
            Blackline = cv2.inRange(img_atual, (0, 0, 0), (75, 75, 75))
            contours_blk, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_blk) > 0:
                blackbox = cv2.minAreaRect(contours_blk[0])
                (x_min, _), _, _ = blackbox
                erro = int(x_min - 160)
                if abs(erro) < 20:
                    rawCapture.truncate(0)
                    motores_frente()
                    return True
            rawCapture.truncate(0)
            
    else:
        motores_virar_dir()
        time.sleep(1.6)
        obs_dir()
        time.sleep(1.0)
        
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            img_atual = frame.array
            Blackline = cv2.inRange(img_atual, (0, 0, 0), (75, 75, 75))
            contours_blk, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_blk) > 0:
                break
            rawCapture.truncate(0)
            
        motores_frente()
        time.sleep(0.5)
        motores_virar_dir()
        
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            img_atual = frame.array
            Blackline = cv2.inRange(img_atual, (0, 0, 0), (75, 75, 75))
            contours_blk, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours_blk) > 0:
                blackbox = cv2.minAreaRect(contours_blk[0])
                (x_min, _), _, _ = blackbox
                erro = int(x_min - 160)
                if abs(erro) < 20:
                    rawCapture.truncate(0)
                    motores_frente()
                    return True
            rawCapture.truncate(0)

def seguir_linha():
    global x_last, y_last
    motores_frente()
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        
        # Verificação do sensor ultrassônico primeiro
        distancia_atual = ler_distancia()
        if distancia_atual <= DISTANCIA_OBSTACULO:
            desvio_obs(lado=1) # Altere a lógica do lado se necessário
            rawCapture.truncate(0)
            continue
            
        # Verificação da área de parada vermelha
        roi_vermelho = image[0:200, 100:220]
        vermelho = cv2.inRange(roi_vermelho, (0, 0, 120), (100, 100, 255))
        vermelho_sign, _ = cv2.findContours(vermelho.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(vermelho_sign) > 0:
            motores_parar()
            rawCapture.truncate(0)
            break
        
        # 1. Monitores de Verde focados BEM EMBAIXO (Y de 150 a 195)
        parte_inferior_esq = image[150:195, 0:110]
        parte_inferior_dir = image[150:195, 210:320]
        
        green_esq = cv2.inRange(parte_inferior_esq, (0, 65, 0), (100, 200, 100))
        green_dir = cv2.inRange(parte_inferior_dir, (0, 65, 0), (100, 200, 100))
        
        green_esq = cv2.erode(green_esq, kernel, iterations=2)
        green_dir = cv2.erode(green_dir, kernel, iterations=2)
        
        contornos_esq, _ = cv2.findContours(green_esq.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contornos_dir, _ = cv2.findContours(green_dir.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # 2. Processamento do Verde (Se estiver embaixo)
        if len(contornos_esq) > 0 and len(contornos_dir) > 0:
            parte_inferior_centro = image[150:195, 115:205]
            green_centro = cv2.inRange(parte_inferior_centro, (0, 65, 0), (100, 200, 100))
            green_centro = cv2.erode(green_centro, kernel, iterations=2)
            contornos_centro, _ = cv2.findContours(green_centro.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contornos_centro) == 0:
                duplo_verde()
                rawCapture.truncate(0)
                continue
            else:
                motores_frente()
                time.sleep(0.25)
                rawCapture.truncate(0)
                continue
            
        elif len(contornos_esq) > 0:
            esq_verde()
            rawCapture.truncate(0)
            continue
            
        elif len(contornos_dir) > 0:
            dir_verde()
            rawCapture.truncate(0)
            continue

        # 3. Linha preta horizontal pura OU com verde acima dela -> Segue Reto
        regiao_gatilho = image[120:160, 30:290]
        Preto_gatilho = cv2.inRange(regiao_gatilho, (0, 0, 0), (75, 75, 75))
        pixels_pretos = np.sum(Preto_gatilho == 255)
        
        if pixels_pretos > 4000: 
            motores_frente()
            time.sleep(0.25) 
            rawCapture.truncate(0)
            continue

        # 4. PID Linha Preta Comum
        Blackline = cv2.inRange(image, (0, 0, 0), (75, 75, 75))
        Blackline = cv2.erode(Blackline, kernel, iterations=2)

        contours_blk, _ = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
            x_last, y_last = x_min, y_min
            
            if ang < -45:
                ang = 90 + ang
            if w_min < h_min and ang > 0:
                ang = (90 - ang) * -1
            if w_min > h_min and ang < 0:
                ang = 90 + ang
                
            setpoint = 160                
            error = int(x_min - setpoint) 
            ang = int(ang)                
            
            calculo_steering = int((error * kp) + (ang * ap))
            
            vel_esq = vel_base + calculo_steering
            vel_dir = vel_base - calculo_steering
            
            vel_esq = max(0, min(100, vel_esq))
            vel_dir = max(0, min(100, vel_dir))
            
            pwm_esq.ChangeDutyCycle(vel_esq)
            pwm_dir.ChangeDutyCycle(vel_dir)
        else:
            pwm_esq.ChangeDutyCycle(vel_base - 10)
            pwm_dir.ChangeDutyCycle(vel_base - 10)

        rawCapture.truncate(0)
        
        if cv2.waitKey(1) & 0xFF == ord("q"):
            motores_parar()
            break

# Executa o loop principal de forma protegida
try:
    seguir_linha()
except KeyboardInterrupt:
    motores_parar()
    GPIO.cleanup()
