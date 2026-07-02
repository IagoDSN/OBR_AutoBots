import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera import PiCamera
from picamera.array import PiRGBArray  # Corrigido: Importação necessária

# Configuração do GPIO
GPIO.setmode(GPIO.BOARD)

# Inicialização da Câmera
camera = PiCamera()
camera.resolution = (320, 200)
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(320, 200))
time.sleep(0.1)

# Variáveis de controle e ganho (KP e AP movidos para cá)
x_last = 160
y_last = 100
kp = 0.75
ap = 1  
speed = 50  # Defina a velocidade base dos motores aqui

start_time = time.time()  # Corrigido: nome da variável
counter = 0

def Motor_steer(port1, port2, speed, steering):  # Corrigido: adicionado 'speed'
    if steering == 0:  # Corrigido: 'steerinf' -> 'steering'
        BP.set_motor_speed(port1, speed)
        BP.set_motor_speed(port2, speed)
        return
    elif steering > 0:
        steering = 100 - steering  # Corrigido: 'sttering' -> 'steering'
        BP.set_motor_speed(port2, speed)
        BP.set_motor_speed(port1, int(speed * steering / 100))
    elif steering < 0:
        steering = steering * -1
        BP.set_motor_speed(port1, speed)
        BP.set_motor_speed(port2, int(speed * steering / 100))

# Loop de captura contínua
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):	
    image = frame.array	
    counter += 1  # Corrigido: Incrementando o contador para o cálculo de FPS
    
    Blackline = cv2.inRange(image, (0, 0, 0), (75, 75, 75))	
    kernel = np.ones((3, 3), np.uint8)
    Blackline = cv2.erode(Blackline, kernel, iterations=2)
    
    # Corrigido para compatibilidade com OpenCV 4+ (retorna 2 valores)
    contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_blk_len = len(contours_blk)
    if contours_blk_len > 0:
        if contours_blk_len == 1:
            blackbox = cv2.minAreaRect(contours_blk[0])
        else:
            candidates = []  # Corrigido: 'canditates' -> 'candidates'
            off_bottom = 0	   
            for con_num in range(contours_blk_len):		
                blackbox = cv2.minAreaRect(contours_blk[con_num])
                (x_min, y_min), (w_min, h_min), ang = blackbox		
                box = cv2.boxPoints(blackbox)
                (x_box, y_box) = box[0]
                if y_box > 198:		 
                    off_bottom += 1
                candidates.append((y_box, con_num, x_min, y_min))		
            
            candidates = sorted(candidates)
            if off_bottom > 1:	   	
                candidates_off_bottom = []
                for con_num in range((contours_blk_len - off_bottom), contours_blk_len):
                    (y_highest, con_highest, x_min, y_min) = candidates[con_num]		
                    total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
                    candidates_off_bottom.append((total_distance, con_highest))
                candidates_off_bottom = sorted(candidates_off_bottom)         
                (total_distance, con_highest) = candidates_off_bottom[0]         
                blackbox = cv2.minAreaRect(contours_blk[con_highest])	    
            else:		
                (y_highest, con_highest, x_min, y_min) = candidates[contours_blk_len - 1]		
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
            
        setpoint = 100
        error = int(x_min - setpoint) 
        ang = int(ang)	 
        box = cv2.boxPoints(blackbox)
        box = np.intp(box)  # Corrigido: np.int0 está obsoleto, alterado para np.intp
        
        # Cálculo do steering
        calculo_steering = int((error * kp) + (ang * ap))
        
        # Corrigido: Passando os 4 parâmetros corretos para a função modificada
        # Nota: Garanta que 'BP' e 'BP.port_D' estejam inicializados corretamente no seu código completo.
        Motor_steer(BP.port_A, BP.port_D, speed, calculo_steering)
        
        # Desenhos na tela
        cv2.drawContours(image, [box], 0, (0, 0, 255), 3)	 
        cv2.putText(image, str(ang), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(image, str(error), (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2) # Ajustado Y para caber na tela de 200px
        cv2.line(image, (int(x_min), 150), (int(x_min), 200), (255, 0, 0), 3)
         
    cv2.imshow("original with line", image)	
    rawCapture.truncate(0)	
    key = cv2.waitKey(1) & 0xFF	
    if key == ord("q"):
        break

finish_time = time.time()
fps = counter / (finish_time - start_time)
print("fps por segundo = " + str(fps))  # Corrigido: Convertido para String
