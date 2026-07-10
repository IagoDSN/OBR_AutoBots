from picamera2 import Picamera2
from gpiozero import Motor
import time
import cv2
import numpy as np

# --- CONFIGURAÇÕES GORAIS DO ROBÔ ---
CONFIG = {
    "DEBUG": False,
    "CAM_SIZE": (320, 240),
    "SETPOINT": 160,
    # Motores (Pinos GPIO)
    "PIN_PIN_ESQ_FWD": 17, "PIN_PIN_ESQ_BWD": 27, "PIN_PIN_ESQ_EN": 18,
    "PIN_PIN_DIR_FWD": 22, "PIN_PIN_DIR_BWD": 23, "PIN_PIN_DIR_EN": 19,
    # PID e Velocidade
    "KP": 0.15, "KI": 0.001, "KD": 0.08,
    "VELOCIDADE_BASE": 40,
    # Filtro de Cor (Preto)
    "HSV_LOWER": np.array([0, 0, 0]),
    "HSV_UPPER": np.array([180, 255, 65])
}

class RoboSeguidor:
    def __init__(self, config):
        self.cfg = config
        
        # Inicializa Motores
        self.motor_esq = Motor(forward=self.cfg["PIN_PIN_ESQ_FWD"], backward=self.cfg["PIN_PIN_ESQ_BWD"], enable=self.cfg["PIN_PIN_ESQ_EN"])
        self.motor_dir = Motor(forward=self.cfg["PIN_PIN_DIR_FWD"], backward=self.cfg["PIN_PIN_DIR_BWD"], enable=self.cfg["PIN_PIN_DIR_EN"])
        
        # Inicializa Câmera
        self.picam2 = Picamera2()
        cam_config = self.picam2.create_video_configuration(main={"size": self.cfg["CAM_SIZE"], "format": "BGR888"})
        self.picam2.configure(cam_config)
        
        # Variáveis de Controle
        self.x_last, self.y_last = 160, 120
        self.last_error = 0
        self.integral = 0
        self.kernel = np.ones((5, 5), np.uint8)

    def iniciar(self):
        print("Aquecendo sensor da câmera...")
        self.picam2.start()
        time.sleep(0.1)
        print("Robô pronto e rodando!")

    def processar_imagem(self):
        """Captura o frame, filtra a linha preta e retorna a posição X ou None se perder a linha."""
        image = self.picam2.capture_array()
        image = cv2.rotate(image, cv2.ROTATE_180)
        roi = image[120:240, 0:320]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.cfg["HSV_LOWER"], self.cfg["HSV_UPPER"])
        mask = cv2.erode(mask, self.kernel, iterations=2)
        mask = cv2.dilate(mask, self.kernel, iterations=4)

        contornos = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blk = contornos[0] if len(contornos) == 2 else contornos[1]

        if len(contours_blk) == 0:
            return None, image

        # Lógica de seleção do contorno correto
        if len(contours_blk) == 1:
            blackbox = cv2.minAreaRect(contours_blk[0])
        else:
            candidates = []
            off_bottom = 0
            for i, con in enumerate(contours_blk):
                bbox = cv2.minAreaRect(con)
                (x_min, y_min), _, _ = bbox
                box = cv2.boxPoints(bbox)
                if box[0][1] > 118:
                    off_bottom += 1
                candidates.append((box[0][1], i, x_min, y_min))
            candidates.sort()

            if off_bottom > 1:
                candidates_off = []
                for idx in range((len(contours_blk) - off_bottom), len(contours_blk)):
                    _, con_idx, x_m, y_m = candidates[idx]
                    dist = (abs(x_m - self.x_last)**2 + abs(y_m - self.y_last)**2)**0.5
                    candidates_off.append((dist, con_idx))
                candidates_off.sort()
                blackbox = cv2.minAreaRect(contours_blk[candidates_off[0][1]])
            else:
                blackbox = cv2.minAreaRect(contours_blk[candidates[-1][1]])

        (x_min, y_min), _, _ = blackbox
        self.x_last, self.y_last = x_min, y_min
        return x_min, image

    def calcular_pid(self, x_linha):
        """Calcula a correção necessária com base no erro atual."""
        error = int(x_linha - self.cfg["SETPOINT"])
        self.integral = max(-1000, min(1000, self.integral + error)) # Anti-windup
        derivative = error - self.last_error
        
        correcao = (self.cfg["KP"] * error) + (self.cfg["KI"] * self.integral) + (self.cfg["KD"] * derivative)
        self.last_error = error
        return correcao, error

    def mover(self, vel_esq, vel_dir):
        """Aplica as velocidades limitando-as entre 0.0 e 1.0 para o gpiozero."""
        vel_esq_norm = max(0.0, min(1.0, vel_esq / 100.0))
        vel_dir_norm = max(0.0, min(1.0, vel_dir / 100.0))
        self.motor_esq.forward(vel_esq_norm)
        self.motor_dir.forward(vel_dir_norm)

    def recuperar_linha(self):
        """Ação tomada quando a linha some da visão."""
        self.integral = 0
        if self.last_error > 0:
            self.mover(45, 15)  # Gira para a direita
        else:
            self.mover(15, 45)  # Gira para a esquerda

    def renderizar_debug(self, image, erro, correcao, v_esq, v_dir):
        """Exibe informações visuais na tela durante os testes."""
        cv2.putText(image, f"Erro: {erro}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv2.putText(image, f"PID: {int(correcao)}", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(image, f"Esq: {int(v_esq)}% Dir: {int(v_dir)}%", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.imshow("Original com Linha", image)

    def desligar(self):
        """Finalização limpa dos recursos."""
        print("\nDesligando motores e sensores de forma segura...")
        self.motor_esq.stop()
        self.motor_dir.stop()
        self.motor_esq.close()
        self.motor_dir.close()
        self.picam2.stop()
        cv2.destroyAllWindows()


# --- LOOP PRINCIPAL DE EXECUÇÃO ---
if __name__ == "__main__":
    robo = RoboSeguidor(CONFIG)
    robo.iniciar()

    try:
        while True:
            x_linha, frame = robo.processar_imagem()

            if x_linha is not None:
                # Se achou a linha, calcula PID e se move
                correcao, erro = robo.calcular_pid(x_linha)
                v_esq = robo.cfg["VELOCIDADE_BASE"] + correcao
                v_dir = robo.cfg["VELOCIDADE_BASE"] - correcao
                robo.mover(v_esq, v_dir)
            else:
                # Se perdeu a linha, executa manobra de recuperação
                robo.recuperar_linha()
                erro, correcao, v_esq, v_dir = 0, 0, 0, 0

            if robo.cfg["DEBUG"]:
                robo.renderizar_debug(frame, erro, correcao, v_esq, v_dir)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        pass
    finally:
        robo.desligar()
