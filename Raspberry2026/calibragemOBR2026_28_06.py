import time
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

# Inicialização da Câmera
camera = PiCamera()
camera.resolution = (320, 200)
camera.rotation = 180 
rawCapture = PiRGBArray(camera, size=(320, 200))
time.sleep(0.1)

print("--- MODO DE CALIBRAÇÃO DE CORES ---")
print("Acerte a câmera do robô para o alvo.")
print("O terminal mostrará os valores BGR do pixel central.")
print("Pressione 'q' na janela de vídeo para sair.\n")

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    
    # 1. Pega o pixel central da imagem para análise manual
    altura, largura, _ = image.shape
    centro_x, centro_y = int(largura / 2), int(altura / 2)
    pixel_central = image[centro_y, centro_x]
    b, g, r = pixel_central[0], pixel_central[1], pixel_central[2]
    
    # Exibe os valores no terminal em tempo real
    print(f"Pixel Central -> Azul(B): {b} | Verde(G): {g} | Vermelho(R): {r} | Brilho/Cinza: {int((int(b)+int(g)+int(r))/3)}")

    # 2. Desenha uma mira no centro da imagem para você saber onde apontar
    cv2.circle(image, (centro_x, centro_y), 5, (255, 255, 255), -1)
    cv2.circle(image, (centro_x, centro_y), 6, (0, 0, 0), 1)

    # 3. Aplica os filtros atuais do seu código para ver o que está sendo detectado
    mascara_preto = cv2.inRange(image, (0, 0, 0), (75, 75, 75))
    mascara_verde = cv2.inRange(image, (0, 65, 0), (100, 200, 100))
    mascara_vermelho = cv2.inRange(image, (0, 0, 120), (100, 100, 255))
    
    # Filtro para o Branco (geralmente valores muito altos em todos os canais)
    mascara_branco = cv2.inRange(image, (200, 200, 200), (255, 255, 255))

    # 4. Mostra as janelas de vídeo
    cv2.imshow("1. Imagem Original (Aponte a Mira)", image)
    cv2.imshow("2. O que o robo ve como PRETO", mascara_preto)
    cv2.imshow("3. O que o robo ve como VERDE", mascara_verde)
    cv2.imshow("4. O que o robo ve como VERMELHO", mascara_vermelho)
    cv2.imshow("5. O que o robo ve como BRANCO", mascara_branco)

    rawCapture.truncate(0)
    
    # Se pressionar 'q', fecha o programa
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()
