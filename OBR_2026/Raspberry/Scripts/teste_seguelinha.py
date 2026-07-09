from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

# =========================================================================
# MODO DEBUG
# Deixe True enquanto estiver ajustando o PID/câmera na bancada.
# Deixe False na hora da competição -> ganha FPS real (sem imshow/desenhos)
# =========================================================================
DEBUG = False

# --- CONFIGURAÇÃO DOS MOTORES (GPIO) ---
GPIO.setwarnings(False)
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

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)

# Sentido de giro fixo (para frente) nas pontes H
GPIO.output(IN1, GPIO.HIGH)
GPIO.output(IN2, GPIO.LOW)
GPIO.output(IN3, GPIO.HIGH)
GPIO.output(IN4, GPIO.LOW)

# --- PWM ---
# OBS: RPi.GPIO faz PWM por software e pode "tremer" em algumas frequências.
# Se notar os motores instáveis mesmo com Kd ajustado, considere migrar
# para a biblioteca "pigpio" (PWM por hardware/DMA, muito mais estável):
#   import pigpio
#   pi = pigpio.pi()
#   pi.set_PWM_dutycycle(ENA, valor_0_a_255)
# Requer rodar antes: sudo pigpiod
pwm_esquerda = GPIO.PWM(ENA, 50)
pwm_direita = GPIO.PWM(ENB, 50)
pwm_esquerda.start(0)
pwm_direita.start(0)

# --- CÂMERA ---
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 40
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(320, 240))
time.sleep(0.1)

# --- HISTÓRICO ---
x_last = 160
y_last = 120

# --- PID ---
Kp = 0.15
Ki = 0.001
Kd = 0.08
velocidade_base = 40

last_error = 0
integral = 0

# Kernel de morfologia (menor = mais rápido, mas pode deixar a máscara mais "furada")
kernel = np.ones((5, 5), np.uint8)

setpoint = 160  # centro horizontal da imagem (320 / 2)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    roi = image[120:240, 0:320]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 65])
    Blackline = cv2.inRange(hsv, lower_black, upper_black)

    Blackline = cv2.erode(Blackline, kernel, iterations=2)
    Blackline = cv2.dilate(Blackline, kernel, iterations=4)

    # Compatível com OpenCV 4.x (2 valores) e 3.x (3 valores)
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

        vel_esquerda = velocidade_base + correcao
        vel_direita = velocidade_base - correcao
        vel_esquerda = max(0, min(100, vel_esquerda))
        vel_direita = max(0, min(100, vel_direita))

        pwm_esquerda.ChangeDutyCycle(vel_esquerda)
        pwm_direita.ChangeDutyCycle(vel_direita)

        if DEBUG:
            box = cv2.boxPoints(blackbox)
            box = box.astype(int)  # substitui np.int0 (removido em NumPy novo)
            box[:, 1] += 120
            cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
            cv2.putText(image, f"Erro: {error}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            cv2.putText(image, f"PID Correcao: {int(correcao)}", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(image, f"M_Esq: {int(vel_esquerda)}% M_Dir: {int(vel_direita)}%", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            cv2.line(image, (int(x_min), 130), (int(x_min), 170), (255, 0, 0), 2)

    else:
        # Linha perdida: em vez de andar reto "no escuro", gira em direção
        # ao último erro conhecido para tentar reencontrar a linha.
        integral = 0
        if last_error > 0:
            # linha saiu para a direita -> gira para a direita
            pwm_esquerda.ChangeDutyCycle(45)
            pwm_direita.ChangeDutyCycle(15)
        else:
            # linha saiu para a esquerda -> gira para a esquerda
            pwm_esquerda.ChangeDutyCycle(15)
            pwm_direita.ChangeDutyCycle(45)

    if DEBUG:
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
