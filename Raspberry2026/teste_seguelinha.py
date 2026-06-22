from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

# --- CONFIGURAÇÃO DOS MOTORES (GPIO) ---
GPIO.setmode(GPIO.BOARD)
GPIO.setmode(GPIO.BCM)

# --- CONFIGURAÇÃO DOS PINOS ---

# Motor A (Lado Esquerdo)
IN1 = 17
IN2 = 27
ENA = 18

# Motor B (Lado Direito)
IN3 = 22
IN4 = 23
ENB = 19

# Configurando todos os pinos como SAÍDA (OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# --- INICIALIZANDO O PWM ---
# O PWM agora é aplicado nos pinos de Enable (ENA e ENB) para controlar a velocidade
pwm_esquerda = GPIO.PWM(ENA, 50)  # Frequência de 50Hz
pwm_direita = GPIO.PWM(ENB, 50)   # Frequência de 50Hz

# Inicia o PWM com 0% de velocidade (parado)
pwm_esquerda.start(0)
pwm_direita.start(0)


# --- CONFIGURAÇÃO DA CÂMERA ---
camera = PiCamera()
camera.resolution = (320, 240) 
camera.framerate = 40 
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(320, 240))
time.sleep(0.1)

# --- VARIÁVEIS DE HISTÓRICO ---
x_last = 160  
y_last = 120  

# --- CONSTANTES DO PID ---
# DICA: Ajuste o Kp primeiro. Quando ficar razoável, ajuste o Kd para parar de tremer.
# O Ki deve ser sempre MUITO pequeno.
Kp = 0.15          # Proporcional (Força da curva)
Ki = 0.001         # Integral (Correção de pequenos desvios acumulados)
Kd = 0.08          # Derivativo (Suavidade e amortecimento)

velocidade_base = 40 # Velocidade do robô em linha reta (0 a 100)

# Variáveis para armazenar o passado do PID
last_error = 0
integral = 0

# --- LOOP PRINCIPAL ---
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):    
    image = frame.array
    
    roi = image[120:240, 0:320]
    
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 65]) 
    Blackline = cv2.inRange(hsv, lower_black, upper_black)
    
    kernel = np.ones((5,5), np.uint8)
    Blackline = cv2.erode(Blackline, kernel, iterations=2)# provavelmente vai abaixar ou aumentar
    Blackline = cv2.dilate(Blackline, kernel, iterations=4)# provavelmente vai abaixar ou aumentar
    
    img_blk, contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_blk_len = len(contours_blk)
    
    if contours_blk_len > 0:
        if contours_blk_len == 1:
            blackbox = cv2.minAreaRect(contours_blk[0])
        else:
            canditates = []
            off_bottom = 0
            for con_num in range(contours_blk_len):
                blackbox = cv2.minAreaRect(contours_blk[con_num])
                (x_min, y_min), (w_min, h_min), ang = blackbox
                box = cv2.boxPoints(blackbox)
                (x_box, y_box) = box[0]
                
                if y_box > 118:
                    off_bottom += 1
                canditates.append((y_box, con_num, x_min, y_min))
            canditates = sorted(canditates)
            
            if off_bottom > 1:
                canditates_off_bottom = []
                for con_num in range((contours_blk_len - off_bottom), contours_blk_len):
                    (y_highest, con_highest, x_min, y_min) = canditates[con_num]
                    total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
                    canditates_off_bottom.append((total_distance, con_highest))
                canditates_off_bottom = sorted(canditates_off_bottom)         
                (total_distance, con_highest) = canditates_off_bottom[0]         
                blackbox = cv2.minAreaRect(contours_blk[con_highest])   
            else:
                (y_highest, con_highest, x_min, y_min) = canditates[contours_blk_len - 1]        
                blackbox = cv2.minAreaRect(contours_blk[con_highest])
                
        (x_min, y_min), (w_min, h_min), ang = blackbox
        x_last = x_min
        y_last = y_min
        
        # --- CÁLCULO DO ERRO ---
        setpoint = 160 
        error = int(x_min - setpoint) 
        
        # --- ALGORITMO PID COMPLETO ---
        integral = integral + error # Acumula o erro ao longo do tempo
        
        # Proteção contra "Windup" (evita que o Integral cresça infinitamente e enlouqueça o robô)
        if integral > 1000: integral = 1000
        if integral < -1000: integral = -1000
            
        derivative = error - last_error # Calcula a taxa de variação
        
        # A Mágica do PID: junta as 3 partes para calcular a correção ideal
        correcao = (Kp * error) + (Ki * integral) + (Kd * derivative)
        
        last_error = error # Atualiza a memória para o próximo frame
        
        # --- APLICA O PID NOS MOTORES ---
        vel_esquerda = velocidade_base + correcao
        vel_direita = velocidade_base - correcao
        
        # Limita a velocidade entre 0 e 100% (Duty Cycle do PWM não pode passar de 100)
        vel_esquerda = max(0, min(100, vel_esquerda))
        vel_direita = max(0, min(100, vel_direita))
        
        # ENVIA O SINAL FÍSICO PARA A PONTE H (Motores giram aqui!)
        pwm_esquerda.ChangeDutyCycle(vel_esquerda)
        pwm_direita.ChangeDutyCycle(vel_direita)
        
        # --- DESENHOS EM TELA ---
        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        box[:, 1] += 120 
        
        cv2.drawContours(image, [box], 0, (0, 0, 255), 2) 
        cv2.putText(image, f"Erro: {error}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv2.putText(image, f"PID Correcao: {int(correcao)}", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(image, f"M_Esq: {int(vel_esquerda)}% M_Dir: {int(vel_direita)}%", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.line(image, (int(x_min), 130), (int(x_min), 170), (255, 0, 0), 2)
    else:
        # Se não achar a linha preta, desliga os motores para não bater
        pwm_esquerda.ChangeDutyCycle(0)
        pwm_direita.ChangeDutyCycle(0)
        integral = 0 # Reseta o integral se perder a linha

    cv2.imshow("orginal with line", image)
    rawCapture.truncate(0)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

# --- FINALIZAÇÃO LIMPA ---
pwm_esquerda.stop()
pwm_direita.stop()
GPIO.cleanup()
cv2.destroyAllWindows()
