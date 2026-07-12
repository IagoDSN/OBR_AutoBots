"""
Robô Seguidor de Linha (Raspberry Pi + PiCamera2 + Ponte H)
=============================================================

Estrutura do arquivo:
    1. Config          -> todas as constantes/parâmetros em um só lugar
    2. MotorController -> controla a ponte H (liga/desliga, PWM, curva)
    3. DetectionResult  -> pacote de retorno de um ciclo de detecção
    4. LineDetector     -> processa o frame, calcula erro e o controle PD
    5. Camera           -> thread de captura contínua da PiCamera2
    6. main()           -> loop principal, junta tudo

Correções feitas na v1 (reorganização):
    - Import correto: `Picamera2` (era `PiCamera2`, causava ImportError)
    - Rotação da câmera feita via `Transform`, não `set_property` (que não existe)
    - Chamada de `Motor_Steer` corrigida (usava `speed=`, parâmetro inexistente)
    - `np.int0` (removido no NumPy atual) trocado por `.astype(int)`
    - Tratamento de erro na inicialização de câmera/motores
    - Logging no lugar de prints soltos

Evoluções da v2 (controle PD + otimização de memória):
    - Controle Proporcional-Derivativo (KP + KD), não só P
    - Guard contra "derivative kick": ao perder a linha, o histórico de erro
      é zerado, evitando um pico artificial no termo D quando ela reaparece
    - Todos os buffers de imagem (HSV, máscara, erosão, dilatação, preview)
      são alocados UMA VEZ e reaproveitados a cada frame via `dst=`, em vez
      de criar uma matriz nova na RAM ~30-60x por segundo
    - `DetectionResult` (dataclass) no lugar de tupla solta — mais legível
      para quem for mexer no código depois
    - Overlay de depuração agora também mostra o valor de steering, útil
      para calibrar KP/KD observando o robô em tempo real
    - Removido `import time`, que não era usado em nenhum lugar
"""

from __future__ import annotations

import logging
import queue
import threading
from dataclasses import dataclass

import cv2
import numpy as np
from gpiozero import OutputDevice, PWMOutputDevice
from libcamera import Transform
from picamera2 import Picamera2

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("line_follower")


# ----------------------------------------------------------------------------
# 1. CONFIGURAÇÃO — mude parâmetros aqui, sem precisar mexer no resto do código
# ----------------------------------------------------------------------------
@dataclass(frozen=True)
class Config:
    # Pinos - Motor A (Esquerdo)
    IN1_PIN: int = 17
    IN2_PIN: int = 27
    ENA_PIN: int = 18  # PWM

    # Pinos - Motor B (Direito)
    IN3_PIN: int = 22
    IN4_PIN: int = 23
    ENB_PIN: int = 19  # PWM

    # Câmera
    FRAME_WIDTH: int = 320
    FRAME_HEIGHT: int = 180
    ROTATE_180: bool = True

    # Visão computacional (faixa HSV considerada "linha preta")
    HSV_LOWER: tuple = (0, 0, 0)
    HSV_UPPER: tuple = (180, 255, 50)
    ERODE_ITER: int = 5
    DILATE_ITER: int = 9
    OFF_BOTTOM_Y_THRESHOLD: int = 358  # ponto considerado "saindo pela base da imagem"

    # Controle PD
    KP: float = 0.75  # Ganho Proporcional
    KD: float = 0.0   # Ganho Derivativo. Comece baixo (0.05-0.3) e suba aos
                       # poucos: deve reduzir oscilação sem criar tremores.
    SETPOINT_X: int = 160  # centro da imagem (FRAME_WIDTH / 2)
    BASE_SPEED: int = 60   # 0 a 100

    SHOW_PREVIEW: bool = True  # desligue se rodar sem monitor (headless) —
                                # também evita alocar o buffer de preview


# ----------------------------------------------------------------------------
# 2. CONTROLE DOS MOTORES (Ponte H)
# ----------------------------------------------------------------------------
class MotorController:
    """Controla os dois motores via ponte H usando PWM (gpiozero)."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.in1 = OutputDevice(cfg.IN1_PIN)
        self.in2 = OutputDevice(cfg.IN2_PIN)
        self.ena = PWMOutputDevice(cfg.ENA_PIN)

        self.in3 = OutputDevice(cfg.IN3_PIN)
        self.in4 = OutputDevice(cfg.IN4_PIN)
        self.enb = PWMOutputDevice(cfg.ENB_PIN)

    def steer(self, base_speed: float, steering: float) -> None:
        """
        Ajusta a velocidade de cada motor para seguir a curva.

        base_speed: velocidade base, de 0 a 100
        steering:   >0 curva para um lado, <0 curva para o outro, 0 = reto
                    (aceita float — o termo D pode gerar valores não inteiros)
        """
        # Sempre para frente (o robô não faz ré nesta versão)
        self.in1.on()
        self.in2.off()
        self.in3.on()
        self.in4.off()

        speed_factor = max(0.0, min(base_speed, 100.0)) / 100.0

        if steering == 0:
            self.ena.value = speed_factor
            self.enb.value = speed_factor
            return

        steering_mod = (100 - min(abs(steering), 100)) / 100.0

        if steering > 0:
            self.enb.value = speed_factor
            self.ena.value = speed_factor * steering_mod
        else:
            self.ena.value = speed_factor
            self.enb.value = speed_factor * steering_mod

    def stop(self) -> None:
        self.ena.value = 0
        self.enb.value = 0

    def shutdown(self) -> None:
        """Desliga tudo com segurança."""
        self.stop()
        self.in1.off()
        self.in3.off()


# ----------------------------------------------------------------------------
# 3. RESULTADO DE UM CICLO DE DETECÇÃO
# ----------------------------------------------------------------------------
@dataclass(frozen=True)
class DetectionResult:
    """Tudo que um frame processado produz, já pronto para uso."""

    blackbox: tuple  # saída de cv2.minAreaRect: ((x,y), (w,h), ângulo)
    error: int        # distância (px) do centro da linha até o setpoint
    angle: int         # ângulo estimado da linha (graus)
    steering: float    # saída do controlador PD, pronta para o MotorController


# ----------------------------------------------------------------------------
# 4. DETECÇÃO DA LINHA + CONTROLE PD
# ----------------------------------------------------------------------------
class LineDetector:
    """
    Processa cada frame e devolve a posição da linha + o comando de direção.

    Controle implementado — PD (Proporcional-Derivativo):
        derivativo = erro_atual - erro_anterior
        steering   = (erro_atual * KP) + (derivativo * KD)

    Se no futuro o robô apresentar erro residual constante em curvas longas
    (algo que P e D sozinhos não corrigem), considere adicionar um termo
    integral (KI) — com cuidado para limitar o acúmulo ("integral windup").

    Otimização de memória: os buffers intermediários (HSV, máscara, erosão,
    dilatação) são alocados UMA VEZ em __init__ e reaproveitados a cada frame
    via o parâmetro `dst=` do OpenCV, em vez de criar uma matriz nova na RAM
    a cada iteração do loop principal. Em um Raspberry Pi isso reduz o
    trabalho do alocador de memória e a variação de latência (jitter) do
    loop de controle — importante porque o termo D é sensível ao intervalo
    de tempo entre frames.
    """

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.kernel = np.ones((3, 3), np.uint8)
        self.x_last = float(cfg.SETPOINT_X)
        self.y_last = cfg.FRAME_HEIGHT / 2

        # --- Estado do controle PD ---
        # None = ainda não há erro anterior válido (primeiro frame, ou a
        # linha acabou de ser reencontrada depois de sumir). Isso evita o
        # "derivative kick": um pico artificial no termo D por falta de
        # histórico, ou por um salto brusco de erro ao reencontrar a linha.
        self.erro_anterior: float | None = None

        # --- Limites HSV pré-convertidos (evita criar 2 arrays por frame) ---
        self._hsv_lower = np.array(cfg.HSV_LOWER, dtype=np.uint8)
        self._hsv_upper = np.array(cfg.HSV_UPPER, dtype=np.uint8)

        # --- Buffers pré-alocados (evita realocação de memória por frame) ---
        shape_3ch = (cfg.FRAME_HEIGHT, cfg.FRAME_WIDTH, 3)
        shape_1ch = (cfg.FRAME_HEIGHT, cfg.FRAME_WIDTH)
        self._hsv_buffer = np.empty(shape_3ch, dtype=np.uint8)
        self._mask_buffer = np.empty(shape_1ch, dtype=np.uint8)
        self._eroded_buffer = np.empty(shape_1ch, dtype=np.uint8)
        self._dilated_buffer = np.empty(shape_1ch, dtype=np.uint8)

    def _select_best_contour(self, contours) -> np.ndarray:
        """Escolhe o contorno mais relevante quando há mais de um candidato."""
        candidates = []
        off_bottom = 0

        for idx, cnt in enumerate(contours):
            box = cv2.minAreaRect(cnt)
            (x_min, y_min), _, _ = box
            box_points = cv2.boxPoints(box)
            _, y_box = box_points[0]
            if y_box > self.cfg.OFF_BOTTOM_Y_THRESHOLD:
                off_bottom += 1
            candidates.append((y_box, idx, x_min, y_min))

        candidates.sort()

        if off_bottom > 1:
            bottom_candidates = []
            for i in range(len(candidates) - off_bottom, len(candidates)):
                _, idx, x_min, y_min = candidates[i]
                dist = ((x_min - self.x_last) ** 2 + (y_min - self.y_last) ** 2) ** 0.5
                bottom_candidates.append((dist, idx))
            bottom_candidates.sort()
            _, best_idx = bottom_candidates[0]
        else:
            _, best_idx, _, _ = candidates[-1]

        return contours[best_idx]

    def detect(self, frame_rgb: np.ndarray) -> DetectionResult | None:
        """Processa um frame; retorna None se nenhuma linha foi encontrada."""
        assert frame_rgb.shape == self._hsv_buffer.shape, (
            "Frame recebido não bate com FRAME_WIDTH/FRAME_HEIGHT do Config "
            "— isso forçaria o OpenCV a realocar os buffers a cada frame."
        )

        # Conversões reaproveitando os buffers pré-alocados (dst=) em vez de
        # criar uma matriz nova a cada iteração do loop principal.
        cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV, dst=self._hsv_buffer)
        cv2.inRange(self._hsv_buffer, self._hsv_lower, self._hsv_upper, dst=self._mask_buffer)
        cv2.erode(self._mask_buffer, self.kernel, dst=self._eroded_buffer, iterations=self.cfg.ERODE_ITER)
        cv2.dilate(self._eroded_buffer, self.kernel, dst=self._dilated_buffer, iterations=self.cfg.DILATE_ITER)

        # Desde o OpenCV 3.2, findContours() não modifica mais a imagem de
        # entrada; e mesmo que modificasse, esse buffer é sobrescrito no
        # próximo frame de qualquer forma — por isso não precisamos de
        # `.copy()` aqui (mais uma alocação evitada por frame).
        contours, _ = cv2.findContours(self._dilated_buffer, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.erro_anterior = None  # linha perdida: zera o histórico do D
            return None

        best_contour = contours[0] if len(contours) == 1 else self._select_best_contour(contours)
        blackbox = cv2.minAreaRect(best_contour)
        (x_min, y_min), (w_min, h_min), ang = blackbox

        self.x_last, self.y_last = x_min, y_min

        if ang < -45:
            ang = 90 + ang
        if w_min < h_min and ang > 0:
            ang = (90 - ang) * -1
        if w_min > h_min and ang < 0:
            ang = 90 + ang

        error = int(x_min - self.cfg.SETPOINT_X)

        # --- Controle PD ---
        derivativo = 0.0 if self.erro_anterior is None else float(error - self.erro_anterior)
        steering = (error * self.cfg.KP) + (derivativo * self.cfg.KD)
        self.erro_anterior = error  # atualiza o histórico para o próximo ciclo

        return DetectionResult(blackbox=blackbox, error=error, angle=int(ang), steering=steering)


# ----------------------------------------------------------------------------
# 5. CÂMERA (thread de captura contínua)
# ----------------------------------------------------------------------------
class Camera:
    """Captura frames em uma thread separada, mantendo sempre o mais recente."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._picam = Picamera2()
        video_config = self._picam.create_video_configuration(
            main={"size": (cfg.FRAME_WIDTH, cfg.FRAME_HEIGHT), "format": "RGB888"},
            transform=Transform(hflip=True, vflip=True) if cfg.ROTATE_180 else Transform(),
        )
        self._picam.configure(video_config)

        self._frame_queue: queue.Queue = queue.Queue(maxsize=1)
        self._running = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._picam.start()
        self._running.set()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        log.info("Câmera iniciada.")

    def _capture_loop(self) -> None:
        while self._running.is_set():
            frame = self._picam.capture_array()
            if self._frame_queue.full():
                try:
                    self._frame_queue.get_nowait()
                except queue.Empty:
                    pass
            self._frame_queue.put(frame)

    def get_frame(self, timeout: float | None = None):
        return self._frame_queue.get(timeout=timeout)

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=1)
        self._picam.stop()
        log.info("Câmera parada.")


# ----------------------------------------------------------------------------
# 6. LOOP PRINCIPAL
# ----------------------------------------------------------------------------
def draw_debug_overlay(image: np.ndarray, result: DetectionResult) -> None:
    (x_min, _), _, _ = result.blackbox
    box = cv2.boxPoints(result.blackbox).astype(int)  # substitui np.int0 (removido no NumPy novo)
    cv2.drawContours(image, [box], 0, (0, 0, 255), 3)
    cv2.putText(image, f"Ang:{result.angle}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.putText(image, f"Err:{result.error}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
    cv2.putText(image, f"Str:{result.steering:.1f}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 0), 2)
    cv2.line(image, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)


def main() -> None:
    cfg = Config()

    camera = Camera(cfg)
    motors = MotorController(cfg)
    detector = LineDetector(cfg)

    # Buffer de exibição pré-alocado. Só existe se o preview estiver ligado —
    # em uso 100% autônomo/headless (SHOW_PREVIEW=False) nem essa alocação
    # nem a conversão RGB->BGR chegam a acontecer.
    display_buffer = (
        np.empty((cfg.FRAME_HEIGHT, cfg.FRAME_WIDTH, 3), dtype=np.uint8)
        if cfg.SHOW_PREVIEW
        else None
    )

    try:
        camera.start()
        log.info("Sistema e motores prontos. Rastreamento (KP=%.2f, KD=%.2f)...", cfg.KP, cfg.KD)

        while True:
            frame_rgb = camera.get_frame()
            result = detector.detect(frame_rgb)

            if result is None:
                # Perdeu a linha: para os motores por segurança
                motors.stop()
                if cfg.SHOW_PREVIEW:
                    cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR, dst=display_buffer)
            else:
                motors.steer(cfg.BASE_SPEED, result.steering)
                if cfg.SHOW_PREVIEW:
                    cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR, dst=display_buffer)
                    draw_debug_overlay(display_buffer, result)

            if cfg.SHOW_PREVIEW:
                cv2.imshow("original with line", display_buffer)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        log.info("Interrompido pelo usuário.")

    finally:
        motors.shutdown()
        camera.stop()
        cv2.destroyAllWindows()
        log.info("Encerrado.")


if __name__ == "__main__":
    main()
