from picamera2 import Picamera2
from gpiozero import Motor
import time
import cv2
import numpy as np

# =========================================================================
# MODO DEBUG
# =========================================================================
DEBUG = False

# --- CONFIGURAÇÃO DOS MOTORES (gpiozero) ---
# A classe Motor já lida com IN1, IN2 e ENA internamente.
# Ela aceita velocidades de 0.0 a 1.0 (onde 1.0 = 100%)
motor_esquerda = Motor(forward=17, backward=27, enable=18)
motor_direita = Motor(forward=22, backward=23, enable=19)

# --- CÂMERA (PICAMERA2) ---
picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (320, 240), "format": "BGR888"})
picam2.configure(config)
picam2.start()
time.sleep(0.1) # Tempo para o sensor aquecer

# --- HISTÓRICO e PID ---
x_last = 160
y_last = 120

Kp = 0.15
Ki = 0.001
Kd = 0.08

# A base do PID ainda usa a escala 0 a 100 para manter os seus multiplicadores (Kp, Kd, etc)
velocidade_base = 40 

last_error = 0
integral = 0

kernel = np.ones((5, 5), np.uint8)
setpoint = 160  # centro horizontal da imagem

try:
    while True:
        # Captura e rotação
        image = picam2.capture_array()
        image = cv2.rotate(image, cv2.ROTATE_180)
        roi = image[120:240, 0:320]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 65])
        Blackline = cv2.inRange(hsv, lower_black, upper_black)

        Blackline = cv2.erode(Blackline, kernel, iterations=2)
        Blackline = cv2.dilate(Blackline, kernel, iterations=4)

        resultado_contornos = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(resultado_contornos) == 3:
            _, contours_blk, hierarchy_blk = resultado_contornos
        else:
            contours_blk, hierarchy_blk = resultado_contornos

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
                        total_distance = (abs(x_min - x_last) ** 2 + abs(y_min - y_last) ** 2) ** 0.5
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

            # --- ERRO E PID ---
            error = int(x_min - setpoint)

            integral = integral + error
            integral = max(-1000, min(1000, integral))  # anti-windup

            derivative = error - last_error
            correcao = (Kp * error) + (Ki * integral) + (Kd * derivative)
            last_error = error

            # Mantém a matemática na escala de 0 a 100
            vel_esquerda = velocidade_base + correcao
            vel_direita = velocidade_base - correcao
            vel_esquerda = max(0, min(100, vel_esquerda))
            vel_direita = max(0, min(100, vel_direita))

            # Converte para a escala 0.0 a 1.0 do gpiozero e move o robô
            motor_esquerda.forward(vel_esquerda / 100.0)
            motor_direita.forward(vel_direita / 100.0)

            if DEBUG:
                box = cv2.boxPoints(blackbox)
                box = box.astype(int) 
                box[:, 1] += 120
                cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
                cv2.putText(image, f"Erro: {error}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                cv2.putText(image, f"PID: {int(correcao)}", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(image, f"Esq: {int(vel_esquerda)}% Dir: {int(vel_direita)}%", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                cv2.line(image, (int(x_min), 130), (int(x_min), 170), (255, 0, 0), 2)

        else:
            # Linha perdida: recuperação
            integral = 0
            if last_error > 0:
                motor_esquerda.forward(0.45) # 45%
                motor_direita.forward(0.15)  # 15%
            else:
                motor_esquerda.forward(0.15)
                motor_direita.forward(0.45)

        if DEBUG:
            cv2.imshow("Original with line", image)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

except KeyboardInterrupt:
    print("\nPrograma interrompido pelo usuário.")

finally:
    # --- FINALIZAÇÃO LIMPA ---
    print("Desligando motores e câmera...")
    motor_esquerda.stop()
    motor_direita.stop()
    # Libera os pinos GPIO corretamente
    motor_esquerda.close()
    motor_direita.close()
    picam2.stop()
    cv2.destroyAllWindows()
