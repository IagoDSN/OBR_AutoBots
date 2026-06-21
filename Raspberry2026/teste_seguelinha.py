from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

# --- CONFIGURAÇÃO DOS MOTORES (GPIO) ---
GPIO.setmode(GPIO.BOARD)

GPIO.setup(38, GPIO.OUT)
GPIO.setup(40, GPIO.OUT)

pwm_esquerda = GPIO.PWM(38, 50)
pwm_direita = GPIO.PWM(40, 50)
pwm_esquerda.start(0)
pwm_direita.start(0)

# --- CONFIGURAÇÃO DA CÂMERA (OTIMIZADA PARA FPS) ---
camera = PiCamera()
# Resolução reduzida pela metade para processamento ultra-rápido
camera.resolution = (320, 240) 
# Força a câmera a enviar até 40 quadros por segundo (o padrão é ~30)
camera.framerate = 40 
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(320, 240))
time.sleep(0.1)

# --- VARIÁVEIS DE CONTROLE E HISTÓRICO ---
x_last = 160  # Metade de 320
y_last = 120  # Metade de 240

# Constantes do Controle PD
Kp = 0.15          
Kd = 0.08           
velocidade_base = 40 
last_error = 0

# --- LOOP PRINCIPAL ---
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):    
    image = frame.array
    
    # 1. OTIMIZAÇÃO ROI: Agora cortamos da linha 120 até 240 (metade inferior da nova resolução)
    roi = image[120:240, 0:320]
    
    # 2. IMUNIDADE À LUZ (HSV)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 65]) 
    Blackline = cv2.inRange(hsv, lower_black, upper_black)
    
    # 3. FILTRAGEM
    kernel = np.ones((5,5), np.uint8)
    Blackline = cv2.erode(Blackline, kernel, iterations=2)
    Blackline = cv2.dilate(Blackline, kernel, iterations=4)
    
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
                
                # A altura da ROI agora é 120, então testamos se passou de 118
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
        
        if ang < -45:
            ang = 90 + ang
        if w_min < h_min and ang > 0:  
            ang = (90 - ang) * -1
        if w_min > h_min and ang < 0:
            ang = 90 + ang  
            
        # --- CÁLCULO DO ERRO (AJUSTADO PARA A NOVA RESOLUÇÃO) ---
        setpoint = 160 # O centro exato da tela de 320px
        error = int(x_min - setpoint) 
        ang = int(ang)     
        
        # Algoritmo PD
        derivative = error - last_error
        correcao = (Kp * error) + (Kd * derivative)
        last_error = error
        
        vel_esquerda = velocidade_base + correcao
        vel_direita = velocidade_base - correcao
        
        vel_esquerda = max(0, min(100, vel_esquerda))
        vel_direita = max(0, min(100, vel_direita))
        
        pwm_esquerda.ChangeDutyCycle(vel_esquerda)
        pwm_direita.ChangeDutyCycle(vel_direita)
        
        # --- DESENHOS EM TELA ---
        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        # Compensa o corte (ROI) de 120 pixels para desenhar no lugar certo
        box[:, 1] += 120 
        
        cv2.drawContours(image, [box], 0, (0, 0, 255), 2) 
        
        # Textos e linhas escalados para caberem na tela menor
        cv2.putText(image, f"Angulo: {ang}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(image, f"Erro: {error}", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv2.putText(image, f"M_Esq: {int(vel_esquerda)}% M_Dir: {int(vel_direita)}%", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.line(image, (int(x_min), 130), (int(x_min), 170), (255, 0, 0), 2)
    else:
        pwm_esquerda.ChangeDutyCycle(0)
        pwm_direita.ChangeDutyCycle(0)

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
