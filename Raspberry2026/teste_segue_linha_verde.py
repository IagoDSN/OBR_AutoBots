from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

# --- CONFIGURAÇÃO DOS MOTORES ---
GPIO.setmode(GPIO.BCM)
IN1, IN2, ENA = 17, 27, 18
IN3, IN4, ENB = 22, 23, 19

GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)

pwm_esquerda = GPIO.PWM(ENA, 50)
pwm_direita = GPIO.PWM(ENB, 50)
pwm_esquerda.start(0)
pwm_direita.start(0)

def configurar_motores(dir_esq, dir_dir):
    if dir_esq == "FRENTE":
        GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
    elif dir_esq == "TRAS":
        GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.HIGH)
    else:
        GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.LOW)

    if dir_dir == "FRENTE":
        GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)
    elif dir_dir == "TRAS":
        GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.HIGH)
    else:
        GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.LOW)

# --- CONFIGURAÇÃO DA CÂMERA ---
camera = PiCamera()
camera.resolution = (320, 240) 
camera.framerate = 40 
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(320, 240))
time.sleep(0.1)

# --- CONSTANTES DO PID ---
Kp, Ki, Kd = 0.15, 0.001, 0.08          
velocidade_base = 40 
last_error = 0
integral = 0
x_last = 160  

# --- MAQUINA DE ESTADOS COMPLETA ---
estado_atual = "NORMAL" 
LINHA_DIVISORIA_Y = 180 

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):    
    image = frame.array
    hsv_total = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # --- 1. ROIs DO VERDE E DO PRETO ---
    roi_preto_normal = hsv_total[120:240, 0:320]
    roi_verde_esq = hsv_total[90:140, 0:80]    
    roi_verde_dir = hsv_total[90:140, 240:320] 

    roi_preto_curva_esq = hsv_total[120:240, 0:120]   
    roi_preto_curva_dir = hsv_total[120:240, 200:320]  

    # --- 2. MÁSCARAS DE COR ---
    lower_black, upper_black = np.array([0, 0, 0]), np.array([180, 255, 65])
    lower_green, upper_green = np.array([35, 50, 50]), np.array([85, 255, 255])

    mask_verde_esq = cv2.inRange(roi_verde_esq, lower_green, upper_green)
    mask_verde_dir = cv2.inRange(roi_verde_dir, lower_green, upper_green)

    # --- 3. PROCESSAMENTO DA LINHA PRETA BASEADO NO ESTADO ---
    if estado_atual == "GIRAR_ESQUERDA":
        mask_preto = cv2.inRange(roi_preto_curva_esq, lower_black, upper_black)
        offset_x = 0
    elif estado_atual == "GIRAR_DIREITA" or estado_atual == "GIRAR_180":
        mask_preto = cv2.inRange(roi_preto_curva_dir, lower_black, upper_black)
        offset_x = 200
    else: 
        mask_preto = cv2.inRange(roi_preto_normal, lower_black, upper_black)
        offset_x = 0

    kernel = np.ones((5,5), np.uint8)
    mask_preto = cv2.erode(mask_preto, kernel, iterations=2)
    mask_preto = cv2.dilate(mask_preto, kernel, iterations=4)
    
    _, contours_blk, _ = cv2.findContours(mask_preto.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # --- 4. TOMADA DE DECISÃO ---
    if estado_atual == "NORMAL" and len(contours_blk) > 0:
        canditates = []
        for con_num in range(len(contours_blk)):
            blackbox = cv2.minAreaRect(contours_blk[con_num])
            (x_min, y_min), _, _ = blackbox
            canditates.append((abs(x_min - 160), y_min, con_num))
        canditates = sorted(canditates)
        
        y_atual_preto = canditates[0][1] + 120 

        pixels_verde_esq = cv2.countNonZero(mask_verde_esq)
        pixels_verde_dir = cv2.countNonZero(mask_verde_dir)

        # Condição A: Verde dos DOIS lados
        if pixels_verde_esq > 200 and pixels_verde_dir > 200:
            if y_atual_preto < LINHA_DIVISORIA_Y:
                estado_atual = "NORMAL" 
            elif y_atual_preto >= LINHA_DIVISORIA_Y:
                estado_atual = "GIRAR_180"

        # Condição B: Verde apenas na Esquerda
        elif pixels_verde_esq > 200:
            if y_atual_preto >= LINHA_DIVISORIA_Y:
                estado_atual = "GIRAR_ESQUERDA"
            else:
                estado_atual = "NORMAL"

        # Condição C: Verde apenas na Direita
        elif pixels_verde_dir > 200:
            if y_atual_preto >= LINHA_DIVISORIA_Y:
                estado_atual = "GIRAR_DIREITA"
            else:
                estado_atual = "NORMAL"

    # --- 5. EXECUÇÃO DAS AÇÕES DOS MOTORES ---
    if len(contours_blk) > 0:
        canditates = []
        setpoint_centro = 60 if estado_atual != "NORMAL" else 160
        for con_num in range(len(contours_blk)):
            blackbox = cv2.minAreaRect(contours_blk[con_num])
            (x_min, y_min), _, _ = blackbox
            canditates.append((abs(x_min - setpoint_centro), x_min))
        canditates = sorted(canditates)
        x_real = canditates[0][1] + offset_x

        if estado_atual == "NORMAL":
            configurar_motores("FRENTE", "FRENTE")
            error = int(x_real - 160)
            integral = max(-1000, min(1000, integral + error))
            correcao = (Kp * error) + (Ki * integral) + (Kd * (error - last_error))
            last_error = error
            
            pwm_esquerda.ChangeDutyCycle(max(0, min(100, velocidade_base + correcao)))
            pwm_direita.ChangeDutyCycle(max(0, min(100, velocidade_base - correcao)))

        elif estado_atual == "GIRAR_ESQUERDA":
            configurar_motores("TRAS", "FRENTE")
            pwm_esquerda.ChangeDutyCycle(35)
            pwm_direita.ChangeDutyCycle(50)

        elif estado_atual == "GIRAR_DIREITA":
            configurar_motores("FRENTE", "TRAS")
            pwm_esquerda.ChangeDutyCycle(50)
            pwm_direita.ChangeDutyCycle(35)

        elif estado_atual == "GIRAR_180":
            configurar_motores("FRENTE", "TRAS")
            pwm_esquerda.ChangeDutyCycle(60)
            pwm_direita.ChangeDutyCycle(60)

    else:
        # --- 6. CONDIÇÃO DE RETORNO AO MODO NORMAL ---
        if estado_atual in ["GIRAR_ESQUERDA", "GIRAR_DIREITA", "GIRAR_180"]:
            estado_atual = "NORMAL"
            integral = 0
            time.sleep(0.1)
        else:
            pwm_esquerda.ChangeDutyCycle(40)
            pwm_direita.ChangeDutyCycle(40)

    # --- TELA LIMPA ---
    cv2.imshow("Robo Visao", image)
    rawCapture.truncate(0)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

pwm_esquerda.stop()
pwm_direita.stop()
GPIO.cleanup()
cv2.destroyAllWindows()
