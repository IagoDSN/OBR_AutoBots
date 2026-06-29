"""
main_picamera_gpiozero.py
=========================
Interface principal do robô adaptada para:
  - PiCamera2  →  captura de imagem direta sem shared_memory
  - gpiozero.Motor (PWM via ponte H L298N/L293D)  →  controle de motores DC

Pinagem padrão da ponte H (ajuste conforme sua fiação):
  Motor Esquerdo : IN1=GPIO17, IN2=GPIO27, ENA=GPIO18 (PWM)
  Motor Direito  : IN1=GPIO22, IN2=GPIO23, ENB=GPIO24 (PWM)

Como o gpiozero.Motor funciona:
  motor.forward(speed)   – avança  | speed: 0.0 (parado) a 1.0 (máxima velocidade)
  motor.backward(speed)  – recua   | speed: 0.0 (parado) a 1.0 (máxima velocidade)
  motor.stop()           – para o motor imediatamente (corta o sinal PWM)
"""

# ─────────────────────────────────────────────────────────────────
#  IMPORTAÇÕES PADRÃO DO PYTHON
# ─────────────────────────────────────────────────────────────────

import tkinter                    # Biblioteca nativa de interface gráfica do Python
from datetime import timedelta    # Para formatar tempo decorrido em hh:mm:ss
from multiprocessing import Process  # Permite rodar tarefas em paralelo (processos separados)
from tkinter import *             # Importa todos os widgets do tkinter (Canvas, StringVar etc.)

# ─────────────────────────────────────────────────────────────────
#  BIBLIOTECAS DE TERCEIROS
# ─────────────────────────────────────────────────────────────────

import customtkinter as ctk       # Versão moderna e estilizada do tkinter (tema escuro etc.)
import cv2                        # OpenCV – processamento de imagem (usado para captura/resize)
import numpy as np                # Numpy – matrizes e operações numéricas eficientes
import psutil                     # Leitura de métricas do sistema (uso de CPU, memória etc.)
from PIL import Image             # Pillow – carrega e converte imagens para exibição na UI
from numba import njit            # Compilador JIT – acelera funções Python com código nativo

# ─────────────────────────────────────────────────────────────────
#  CÂMERA – PiCamera2
# ─────────────────────────────────────────────────────────────────

from picamera2 import Picamera2
# Picamera2 é a biblioteca oficial da Raspberry Pi para câmeras CSI (câmera flat).
# Substitui a antiga "picamera" e o uso de shared_memory para trafegar frames.

# ─────────────────────────────────────────────────────────────────
#  CONTROLE DE MOTORES – gpiozero + pigpio
# ─────────────────────────────────────────────────────────────────

from gpiozero import Motor        # Classe que controla motor DC via ponte H usando GPIO
from gpiozero import Device       # Classe base para trocar o "pin factory" do gpiozero
from gpiozero.pins.pigpio import PiGPIOFactory
# PiGPIOFactory usa o daemon pigpio em vez do RPi.GPIO padrão.
# Isso dá um sinal PWM muito mais preciso e estável nos pinos de enable da ponte H.
# ATENÇÃO: o daemon pigpio precisa estar rodando → execute "sudo pigpiod" no terminal antes.
# Se não tiver pigpio instalado, comente as duas linhas abaixo e o Device.pin_factory.

Device.pin_factory = PiGPIOFactory()
# Diz ao gpiozero para usar pigpio em TODOS os pinos (substitui o padrão RPi.GPIO).

# ─────────────────────────────────────────────────────────────────
#  MÓDULOS INTERNOS DO PROJETO
# ─────────────────────────────────────────────────────────────────

from control import control_loop        # Loop de controle autônomo do robô (lógica de navegação)
from mp_manager import *                # Variáveis compartilhadas entre processos (multiprocessing)
from sensor_serial import serial_loop   # Loop que lê sensores via porta serial (UART/USB)

# ─────────────────────────────────────────────────────────────────
#  TEMA DA INTERFACE GRÁFICA
# ─────────────────────────────────────────────────────────────────

ctk.set_appearance_mode("Dark")          # Interface escura (dark mode)
ctk.set_default_color_theme("dark-blue") # Esquema de cores azul-escuro para botões e frames

# ─────────────────────────────────────────────────────────────────
#  CONFIGURAÇÕES DE RESOLUÇÃO DAS CÂMERAS
# ─────────────────────────────────────────────────────────────────
# Resolução de EXIBIÇÃO na UI (tamanho do widget CTkImage na tela):
CAM1_WIDTH  = 320   # largura do frame da câmera 1 exibido na tela (pixels)
CAM1_HEIGHT = 240   # altura  do frame da câmera 1 exibido na tela (pixels)

CAM2_WIDTH  = 320   # largura do frame da câmera 2 exibido na tela (pixels)
CAM2_HEIGHT = 200   # altura  do frame da câmera 2 exibido na tela (pixels)

# Resolução de CAPTURA da PiCamera2 (o que o sensor realmente grava):
# Pode ser diferente da resolução de exibição — o CTkImage faz o redimensionamento.
CAM1_CAPTURE = (320, 240)   # câmera 1: resolução nativa de captura (largura × altura)
CAM2_CAPTURE = (320, 200)   # câmera 2: resolução nativa de captura (largura × altura)

# ─────────────────────────────────────────────────────────────────
#  INSTÂNCIAS DOS MOTORES (criadas uma vez ao iniciar o programa)
# ─────────────────────────────────────────────────────────────────
# Motor(forward=pino_sentido_A, backward=pino_sentido_B, enable=pino_PWM, pwm=True)
# Os pinos usam numeração BCM (Broadcom) — os números estampados no SoC, não na placa.

MOTOR_LEFT  = Motor(forward=17, backward=27, enable=18, pwm=True)
# Motor esquerdo: frente→GPIO17, ré→GPIO27, velocidade PWM→GPIO18

MOTOR_RIGHT = Motor(forward=22, backward=23, enable=24, pwm=True)
# Motor direito: frente→GPIO22, ré→GPIO23, velocidade PWM→GPIO24

# ─────────────────────────────────────────────────────────────────
#  VARIÁVEIS GLOBAIS DE INTERFACE
# ─────────────────────────────────────────────────────────────────

display_status = "none"   # Guarda o último status exibido para evitar atualizar desnecessariamente
data_font_size = 15       # Tamanho padrão da fonte nos rótulos de dados dos sensores
label_color    = "#141414"  # Cor de fundo dos campos de dados (quase preto)
button_color   = "#141414"  # Cor de fundo dos botões (igual ao label, integra ao tema escuro)
testing_mode   = False    # Se True, desativa o modelo 3D (útil para testar sem os arquivos .npz)

# ─────────────────────────────────────────────────────────────────
#  MODELO 3D DO ROBÔ (mapa de imagens pré-renderizadas)
# ─────────────────────────────────────────────────────────────────
# O arquivo .npz contém um dicionário: chave = (yaw, pitch) → valor = imagem CTkImage
# Isso evita renderizar o 3D em tempo real — basta indexar a imagem correta.

if not testing_mode:
    model_map = np.load(
        "../../Python/main/resources/robot_model.npz", allow_pickle=True
    )["image_hashmap"].item()
    # allow_pickle=True é necessário porque o dicionário Python foi serializado com pickle.
    # .item() converte o array numpy de 0-dimensão de volta para o dict Python original.


# ─────────────────────────────────────────────────────────────────
#  FUNÇÃO AUXILIAR – CÁLCULO DE YAW E PITCH PARA O MODELO 3D
# ─────────────────────────────────────────────────────────────────

@njit(cache=True)
# @njit compila esta função para código nativo (C-like) na primeira chamada.
# cache=True salva o binário compilado em disco para não recompilar a cada execução.
def get_yaw_pitch(yaw, pitch):
    """
    Recebe yaw (rotação horizontal) e pitch (inclinação vertical) do IMU.
    Retorna (yaw_ajustado, pitch_limitado) compatíveis com as chaves do model_map.

    - O yaw é arredondado ao múltiplo de 2 mais próximo e mapeado para 0–358°
      considerando que o modelo está orientado 270° em relação ao norte do sensor.
    - O pitch é arredondado ao múltiplo de 2 e limitado entre -30° e +30°
      (além disso o modelo visual não muda).
    """
    rounded_yaw   = round(yaw   / 2) * 2   # arredonda para granularidade de 2 graus
    rounded_pitch = round(pitch / 2) * 2   # arredonda para granularidade de 2 graus
    wrapped_yaw   = (270 - rounded_yaw) % 360    # converte para o sistema do modelo
    clamped_pitch = max(-30, min(rounded_pitch, 30))  # limita pitch ao intervalo [-30, 30]
    return wrapped_yaw, clamped_pitch


# ─────────────────────────────────────────────────────────────────
#  FUNÇÃO AUXILIAR – DESENHA CÍRCULOS NO CANVAS DE VÍTIMAS
# ─────────────────────────────────────────────────────────────────

def create_circle(x, y, r, canvas, style):
    """
    Desenha um círculo no Canvas tkinter.
    x, y = coordenadas do centro | r = raio | canvas = widget Canvas
    style define a cor de preenchimento:
      1 → cinza escuro (#292929) = posição vazia (nenhuma vítima coletada ainda)
      2 → cinza claro (#BBBBBB)  = vítima VIVA coletada
      3 → preto (Black)          = vítima MORTA coletada
    """
    x0, y0, x1, y1 = x - r, y - r, x + r, y + r  # bounding box do círculo
    if style == 1:
        return canvas.create_oval(x0, y0, x1, y1, outline=label_color, width=3, fill="#292929")
    elif style == 2:
        return canvas.create_oval(x0, y0, x1, y1, outline=label_color, width=3, fill="#BBBBBB")
    elif style == 3:
        return canvas.create_oval(x0, y0, x1, y1, outline=label_color, width=3, fill="Black")


# =================================================================
#  CONTROLE DE MOTORES
# =================================================================

def set_motors(left_speed: float, right_speed: float):
    """
    Controla os dois motores DC através da ponte H usando gpiozero.
    Parâmetros:
      left_speed  : velocidade do motor esquerdo — negativo = ré, positivo = frente
      right_speed : velocidade do motor direito  — negativo = ré, positivo = frente
    Faixa válida: -1.0 (ré máxima) a +1.0 (frente máxima). 0.0 = parado.
    """

    def _drive(motor: Motor, speed: float):
        """Função interna que traduz velocidade (+/-) nos métodos do gpiozero."""
        if speed > 0:
            motor.forward(min(speed, 1.0))   # min() garante que não ultrapasse 1.0 (100%)
        elif speed < 0:
            motor.backward(min(-speed, 1.0)) # inverte o sinal para obter valor positivo
        else:
            motor.stop()                     # velocidade zero = para o motor

    _drive(MOTOR_LEFT,  left_speed)   # aplica ao motor esquerdo
    _drive(MOTOR_RIGHT, right_speed)  # aplica ao motor direito


def stop_motors():
    """Para ambos os motores imediatamente. Chamado ao fechar o programa."""
    MOTOR_LEFT.stop()
    MOTOR_RIGHT.stop()


# =================================================================
#  INICIALIZAÇÃO DAS CÂMERAS (PiCamera2)
# =================================================================

def init_cameras():
    """
    Cria e configura as instâncias da PiCamera2.
    Retorna (cam1, cam2) onde cam2 pode ser None se não houver segunda câmera.

    Fluxo de configuração de cada câmera:
      1. Picamera2(índice) — abre a câmera pelo índice do dispositivo (0 = primeira)
      2. create_preview_configuration() — define resolução e formato de pixel
      3. cam.configure(cfg) — aplica a configuração
      4. cam.start() — inicia o sensor e começa a capturar frames internamente
    """

    # ── Câmera 1 (câmera Pi principal, índice 0) ──────────────────
    cam1 = Picamera2(0)
    cfg1 = cam1.create_preview_configuration(
        main={"size": CAM1_CAPTURE, "format": "RGB888"}
        # RGB888 = 3 bytes por pixel (R, G, B) — compatível com PIL.Image.fromarray()
    )
    cam1.configure(cfg1)   # aplica resolução e formato
    cam1.start()           # começa a capturar (frames ficam em buffer interno)

    # ── Câmera 2 (câmera secundária, índice 1) ────────────────────
    # Pode ser uma segunda câmera Pi num adaptador dual, ou uma câmera USB.
    # Se não existir, capturamos a exceção e usamos cam2 = None como fallback.
    try:
        cam2 = Picamera2(1)
        cfg2 = cam2.create_preview_configuration(
            main={"size": CAM2_CAPTURE, "format": "RGB888"}
        )
        cam2.configure(cfg2)
        cam2.start()
    except Exception:
        # Câmera 2 não encontrada ou falhou ao inicializar
        cam2 = None
        print("[AVISO] Câmera 2 não encontrada. Rodando com apenas uma câmera.")

    return cam1, cam2


# =================================================================
#  CLASSE PRINCIPAL DA APLICAÇÃO (janela gráfica)
# =================================================================

class App(ctk.CTk):
    """
    Janela principal do robô.
    Herda de ctk.CTk (CustomTkinter) que herda de tkinter.Tk.
    Responsável por:
      - Montar todos os widgets da interface
      - Atualizar sensores, câmeras e indicadores a cada 100 ms (método main())
      - Receber comandos do operador (botões de calibração, fechar etc.)
    """

    def __init__(self, cam1: Picamera2, cam2):
        """
        Construtor: recebe as câmeras já iniciadas e constrói toda a UI.
        cam1 = câmera principal (Picamera2)
        cam2 = câmera secundária (Picamera2 ou None)
        """
        super().__init__()  # inicializa o CTk (janela raiz do tkinter)

        # Guarda as câmeras como atributos para acessar no loop main()
        self.cam1 = cam1
        self.cam2 = cam2

        # ── Configuração da janela ────────────────────────────────
        self.title("Overengineering²")          # título da janela (barra de título)
        self.geometry("1024x600")               # tamanho padrão (largura x altura em pixels)
        self.resizable(False, False)            # impede redimensionamento manual
        self.attributes("-topmost", True, "-fullscreen", True)
        # -topmost = janela sempre por cima das outras
        # -fullscreen = ocupa toda a tela (bom para display de robô sem teclado)

        # ──────────────────────────────────────────────────────────
        #  MAINFRAME – container principal que ocupa toda a janela
        # ──────────────────────────────────────────────────────────
        self.mainFrame = ctk.CTkFrame(master=self)
        self.mainFrame.pack(pady=8, padx=8, fill="both", expand=True)
        # pack() com fill="both" + expand=True faz o frame ocupar toda a janela

        # Grid de 5 colunas com pesos (colunas 0 e 4 são maiores)
        for col, w in enumerate([2, 1, 1, 1, 2]):
            self.mainFrame.grid_columnconfigure(col, weight=w)
        for row in range(3):
            self.mainFrame.grid_rowconfigure(row)

        # ──────────────────────────────────────────────────────────
        #  CÂMERA 1 — exibida no canto superior esquerdo
        # ──────────────────────────────────────────────────────────
        self.top_cam = ctk.CTkFrame(master=self.mainFrame)
        self.top_cam.grid(column=0, row=0, columnspan=2, sticky="w", padx=8, pady=8)
        # columnspan=2: ocupa 2 colunas do grid | sticky="w": alinha à esquerda

        self.top_camera = ctk.CTkLabel(self.top_cam, text="")
        # O Label começa vazio — a imagem da câmera é atribuída no loop main()
        self.top_camera.grid(padx=6, pady=6)

        # ──────────────────────────────────────────────────────────
        #  CÂMERA 2 — exibida no canto superior direito
        # ──────────────────────────────────────────────────────────
        self.bottom_cam = ctk.CTkFrame(master=self.mainFrame)
        self.bottom_cam.grid(column=3, row=0, columnspan=2, sticky="e", padx=8, pady=8)
        # sticky="e": alinha à direita

        self.bottom_camera = ctk.CTkLabel(self.bottom_cam, text="")
        self.bottom_camera.grid(padx=6, pady=6)

        # ──────────────────────────────────────────────────────────
        #  DATAFRAME — painel de dados dos sensores (esquerda, meio)
        # ──────────────────────────────────────────────────────────
        self.dataFrame = ctk.CTkFrame(master=self.mainFrame)
        self.dataFrame.configure(width=self.dataFrame["width"])
        self.dataFrame.grid_propagate(0)
        # grid_propagate(0) impede que o frame encolha para caber nos filhos
        self.dataFrame.grid(column=0, row=1, sticky="nswe", padx=8, pady=4)

        # Colunas alternadas: rótulo (peso 0 = fixo) e valor (peso 1 = flexível)
        for col, w in enumerate([0, 1, 0, 1]):
            self.dataFrame.grid_columnconfigure(col, weight=w)
        for row in range(5):
            self.dataFrame.grid_rowconfigure(row, weight=1)

        # Lista de (texto do rótulo, nome do atributo StringVar)
        sensors = [
            ("Front L:", "label_sensor_1_var"),   # sensor ultrassônico frontal esquerdo
            ("Front R:", "label_sensor_2_var"),   # sensor ultrassônico frontal direito
            ("Left:",    "label_sensor_3_var"),   # sensor lateral esquerdo
            ("Right:",   "label_sensor_4_var"),   # sensor lateral direito
            ("Front C:", "label_sensor_5_var"),   # sensor frontal central
            ("Back:",    "label_sensor_6_var"),   # sensor traseiro
            ("Gripper:", "label_sensor_7_var"),   # sensor da garra (distância de pega)
            ("Yaw:",     "label_sensor_x_var"),   # ângulo horizontal (IMU)
            ("Pitch:",   "label_sensor_y_var"),   # inclinação frontal/traseira (IMU)
            ("Roll:",    "label_sensor_z_var"),   # inclinação lateral (IMU)
        ]
        units = ["mm"] * 7 + ["°", "°", "°"]  # unidades correspondentes a cada sensor

        # Cria dinamicamente os rótulos e campos de valor para cada sensor
        for i, (txt, attr) in enumerate(sensors):
            row, col_pair = divmod(i, 2)   # distribui em 5 linhas × 2 colunas
            col_label = col_pair * 2       # coluna do rótulo (0 ou 2)
            col_data  = col_label + 1      # coluna do valor  (1 ou 3)
            sticky_label = "es" if row == 4 else "e"  # última linha alinha sul+leste

            # Rótulo fixo (texto estático como "Front L:")
            ctk.CTkLabel(
                self.dataFrame, text=txt, font=("Arial", data_font_size)
            ).grid(column=col_label, row=row, sticky=sticky_label, padx=10, pady=5)

            # Variável dinâmica que será atualizada pelo loop main()
            var = tkinter.StringVar(value=f"0 {units[i]}")
            setattr(self, attr, var)   # self.label_sensor_1_var = var, etc.

            # Campo de valor com fundo escuro (parece um "display")
            ctk.CTkLabel(
                self.dataFrame, textvariable=var,
                fg_color=label_color, corner_radius=4,
                font=("Arial", data_font_size)
            ).grid(column=col_data, row=row, sticky="w", padx=8, pady=5)

        # ──────────────────────────────────────────────────────────
        #  MODELFRAME — visualização 3D do robô (centro)
        # ──────────────────────────────────────────────────────────
        if not testing_mode:
            self.modelFrame = ctk.CTkFrame(
                master=self.mainFrame, width=266, fg_color="#292929"
            )
            self.modelFrame.configure(width=self.modelFrame["width"])
            self.modelFrame.grid_propagate(0)
            self.modelFrame.grid(column=1, row=1, columnspan=3, sticky="nswe", padx=6, pady=4)

            self.modelImage = ctk.CTkLabel(self.modelFrame, text="")
            # A imagem do modelo 3D é trocada no loop main() conforme o yaw/pitch do IMU
            self.modelImage.grid(sticky="nswe", padx=8, pady=8)

        # ──────────────────────────────────────────────────────────
        #  LABEL DE STATUS — mensagem textual de estado do robô
        # ──────────────────────────────────────────────────────────
        self.label_status_var  = tkinter.StringVar(value=" ")
        self.label_status_font = ctk.CTkFont(family="arial", size=15)
        ctk.CTkLabel(
            master=self.mainFrame, corner_radius=4,
            textvariable=self.label_status_var,
            text_color="white", font=self.label_status_font, fg_color="#141414"
        ).grid(column=3, columnspan=2, sticky="s", row=0, padx=8, pady=17)
        # Posicionado abaixo da câmera 2, no lado direito

        # ──────────────────────────────────────────────────────────
        #  ZONEFRAME — painel de indicadores de zona (direita, meio)
        # ──────────────────────────────────────────────────────────
        self.zoneFrame = ctk.CTkFrame(master=self.mainFrame)
        self.zoneFrame.configure(width=self.zoneFrame["width"])
        self.zoneFrame.grid_propagate(0)
        self.zoneFrame.grid(column=4, row=1, sticky="nswe", padx=8, pady=4)
        self.zoneFrame.grid_columnconfigure(0, weight=2)  # coluna de canvas (maior)
        self.zoneFrame.grid_columnconfigure(1, weight=1)
        self.zoneFrame.grid_columnconfigure(2, weight=1)
        self.zoneFrame.grid_rowconfigure(0, weight=1)
        self.zoneFrame.grid_rowconfigure(1, weight=1)

        # Canvas para exibir os círculos de vítimas coletadas
        self.canvas = Canvas(
            self.zoneFrame, width=80, height=30,
            bg="#292929", highlightthickness=0
        )
        # highlightthickness=0 remove a borda padrão do Canvas
        self.canvas.grid(column=0, row=1, sticky="sw", padx=4, pady=4)

        # ── Botão de SAIR da calibração (ícone ✕) ─────────────────
        self.exit_color_calibration_button = ctk.CTkButton(
            master=self.zoneFrame, corner_radius=5,
            command=self.exit_calibrate_color,   # chama exit_calibrate_color() ao clicar
            text="✕",
            text_color="white", font=("Arial", 20), width=40, height=30,
            fg_color=button_color, hover_color="black"
        )
        self.exit_color_calibration_button.grid(column=2, row=0, sticky="ne", padx=53, pady=8)

        # ── Botão de INICIAR calibração (ícone ✎ / ⎚ / ✓) ────────
        # O ícone muda conforme o estado da calibração (em andamento, conferindo etc.)
        self.color_calibration_button_var = tkinter.StringVar(value="✎")
        self.color_calibration_button = ctk.CTkButton(
            master=self.zoneFrame, corner_radius=5,
            command=self.set_calibrate_color_status,
            textvariable=self.color_calibration_button_var,  # ícone dinâmico
            text_color="white", font=("Arial", 20), width=40, height=30,
            fg_color="black", hover_color=button_color
        )
        self.color_calibration_button.grid(column=2, row=0, sticky="ne", padx=8, pady=8)

        # ── Botão de ESCOLHER cor a calibrar (cicla entre "z-g", "l-gz" etc.) ──
        self.color_choose_button_var = tkinter.StringVar(value="z-g")
        self.color_choose_button = ctk.CTkButton(
            master=self.zoneFrame, corner_radius=5,
            command=self.choose_color,
            textvariable=self.color_choose_button_var,
            text_color="white", font=("Arial", data_font_size), width=40, height=30,
            fg_color=button_color, hover_color="black"
        )
        self.color_choose_button.grid(column=2, row=0, sticky="ne", padx=8, pady=44)

        # ──────────────────────────────────────────────────────────
        #  BARFRAME — barra inferior com timers e logo
        # ──────────────────────────────────────────────────────────
        self.barFrame = ctk.CTkFrame(master=self.mainFrame)
        self.barFrame.grid_propagate(0)
        self.barFrame.grid(column=0, row=2, columnspan=5, sticky="nwe", padx=8, pady=8)
        # columnspan=5: ocupa todas as colunas do grid (barra de rodapé)

        for col, w in enumerate([2, 1, 1, 1, 2]):
            self.barFrame.grid_columnconfigure(col, weight=w)
        for row in range(3):
            self.barFrame.grid_rowconfigure(row)

        # Logo da equipe (canto superior direito da barra)
        logo = Image.open("../../Python/main/resources/logo/logo_white_transparent.png")
        self.logo_img = ctk.CTkImage(logo, size=(50, 50))
        ctk.CTkLabel(self.barFrame, text="", image=self.logo_img).grid(
            column=2, row=0, columnspan=3, rowspan=3, sticky="ne", padx=8, pady=18
        )

        # Timer principal da execução (mm:ss:cs)
        self.label_timer_var = tkinter.StringVar(value="--:--:--")
        ctk.CTkLabel(
            master=self.barFrame, textvariable=self.label_timer_var,
            text_color="white", font=("Arial", 30), width=80, height=30
        ).grid(column=2, row=0, sticky="n", padx=0, pady=10)

        # Timer da zona de evacuação (aparece quando o robô entra na zona)
        self.label_timer_zone_var = tkinter.StringVar(value="--:--:--")
        ctk.CTkLabel(
            master=self.barFrame, textvariable=self.label_timer_zone_var,
            text_color="white", font=("Arial", 20), width=50, height=30
        ).grid(column=2, row=1, sticky="s", padx=0, pady=0)

        # ──────────────────────────────────────────────────────────
        #  BOTÕES DO CANTO SUPERIOR DIREITO
        # ──────────────────────────────────────────────────────────

        # Botão ✕ — fecha o programa completamente
        self.exit_button = ctk.CTkButton(
            master=self.mainFrame, corner_radius=10, command=self.exit,
            text="✕", text_color="white", font=("Arial", 20), width=30, height=30,
            fg_color="black", bg_color="black", hover_color=button_color
        )
        self.exit_button.grid(column=4, row=0, sticky="ne", padx=8, pady=4)

        # Botão – / + — alterna entre fullscreen e janela normal
        self.expand_button_var = tkinter.StringVar(value="-")
        self.expand_button = ctk.CTkButton(
            master=self.mainFrame, corner_radius=10, command=self.expand,
            textvariable=self.expand_button_var, text_color="white",
            font=("Arial", 20), width=40, height=30,
            fg_color="black", bg_color="black", hover_color=button_color
        )
        self.expand_button.grid(column=4, row=0, sticky="ne", padx=50, pady=4)

        # Botão * — captura e salva um frame da câmera em disco
        self.capture_button = ctk.CTkButton(
            master=self.mainFrame, corner_radius=10, command=self.capture_image,
            text="*", text_color="white", font=("Arial", 20), width=40, height=30,
            fg_color="black", bg_color="black", hover_color=button_color
        )
        self.capture_button.grid(column=4, row=0, sticky="ne", padx=92, pady=4)

        # ──────────────────────────────────────────────────────────
        #  INDICADORES DE PERFORMANCE E ESTADO (topo)
        # ──────────────────────────────────────────────────────────

        # Uso de CPU em tempo real (psutil.cpu_percent() no loop main)
        self.label_cpu_var = tkinter.StringVar(value="0 %")
        ctk.CTkLabel(
            master=self.mainFrame, textvariable=self.label_cpu_var,
            text_color="white", font=("Arial", 15), width=100, height=30, fg_color=button_color
        ).grid(column=3, columnspan=2, row=0, sticky="nw", padx=12, pady=4)

        # IPS_C = Iterações Por Segundo do loop de controle (process control_loop)
        self.label_ips_c_var = tkinter.StringVar(value="0")
        ctk.CTkLabel(
            master=self.mainFrame, textvariable=self.label_ips_c_var,
            text_color="white", font=("Arial", 15), width=85, height=30, fg_color=button_color
        ).grid(column=3, columnspan=2, row=0, sticky="nw", padx=118, pady=4)

        # IPS_S = Iterações Por Segundo do loop serial (process serial_loop)
        self.label_ips_s_var = tkinter.StringVar(value="0")
        ctk.CTkLabel(
            master=self.mainFrame, textvariable=self.label_ips_s_var,
            text_color="white", font=("Arial", 15), width=85, height=30, fg_color=button_color
        ).grid(column=3, columnspan=2, row=0, sticky="ne", padx=179, pady=4)

        # Indicador de HOLD — mostra se o robô está pausado (‖) ou em execução (▶)
        self.label_hold_var = tkinter.StringVar(value="‖")
        ctk.CTkLabel(
            master=self.mainFrame, corner_radius=5,
            textvariable=self.label_hold_var, text_color="white",
            font=("Arial", 15), width=40, height=30,
            fg_color=button_color, bg_color=button_color
        ).grid(column=2, row=0, sticky="n", padx=8, pady=4)

        # Indicador de INCLINAÇÃO — "n" (nenhuma), "u" (subindo rampa), "d" (descendo)
        self.label_rotation_var = tkinter.StringVar(value="-")
        ctk.CTkLabel(
            master=self.mainFrame, corner_radius=5,
            textvariable=self.label_rotation_var, text_color="white",
            font=("Arial", data_font_size), width=40, height=30, fg_color=button_color
        ).grid(column=2, row=0, sticky="s", padx=8, pady=123)

        # Ângulo do GAP (lacuna detectada na linha guia)
        self.label_gap_angle_var = tkinter.StringVar(value="0")
        ctk.CTkLabel(
            master=self.mainFrame, corner_radius=5,
            textvariable=self.label_gap_angle_var, text_color="white",
            font=("Arial", 12), width=40, height=30, fg_color=button_color
        ).grid(column=2, row=0, sticky="s", padx=8, pady=83)

        # Seta de DIREÇÃO atual do robô (⇧ frente, ⇦ esquerda, ⇨ direita, ⟲ virar)
        self.label_turn_dir_var = tkinter.StringVar(value="⇧")
        ctk.CTkLabel(
            master=self.mainFrame, corner_radius=10,
            textvariable=self.label_turn_dir_var, text_color="white",
            font=("Arial", 25), width=40, height=30,
            fg_color=button_color, bg_color=button_color
        ).grid(column=2, row=0, sticky="s", padx=8, pady=43)

        # Ângulo da LINHA guia detectado pela câmera inferior
        self.label_angle_var = tkinter.StringVar(value="0")
        ctk.CTkLabel(
            master=self.mainFrame, corner_radius=5,
            textvariable=self.label_angle_var, text_color="white",
            font=("Arial", data_font_size), width=40, height=30, fg_color=button_color
        ).grid(column=2, row=0, sticky="s", padx=8, pady=8)

    # ================================================================
    #  MÉTODOS DE CONTROLE DA JANELA
    # ================================================================

    def expand(self):
        """
        Alterna entre modo fullscreen (sem bordas) e janela normal.
        Útil para depuração no desktop com mouse e teclado.
        """
        if self.expand_button_var.get() == "-":
            # Estava em fullscreen → volta para janela normal
            self.attributes("-topmost", False, "-fullscreen", False)
            self.expand_button_var.set("+")
        else:
            # Estava em janela → vai para fullscreen
            self.attributes("-topmost", True, "-fullscreen", True)
            self.expand_button_var.set("-")

    def exit(self):
        """
        Encerra o programa com segurança:
          1. Para os motores (evita que fiquem rodando sem controle)
          2. Para e fecha as câmeras (libera recursos do hardware)
          3. Sinaliza o encerramento para os processos filhos (terminate.value = True)
          4. Aguarda e termina cada processo filho
          5. Destrói a janela tkinter
        """
        stop_motors()   # para os motores antes de qualquer outra coisa

        # Para e fecha câmera 1
        self.cam1.stop()
        self.cam1.close()

        # Para e fecha câmera 2 (se existir)
        if self.cam2:
            self.cam2.stop()
            self.cam2.close()

        # Sinaliza para os processos filhos que devem encerrar
        terminate.value = True
        time.sleep(0.5)  # dá tempo para os loops lerem o sinal

        # Termina cada processo filho (serial_loop, control_loop etc.)
        for process in processes:
            process.terminate()
            time.sleep(0.5)  # aguarda o encerramento de cada um

        self.destroy()  # fecha a janela tkinter (encerra o mainloop)

    @staticmethod
    def capture_image():
        """
        Sinaliza para o processo de controle que deve salvar um frame em disco.
        capture_image.value é uma variável compartilhada (mp_manager).
        O processo que monitora este valor é responsável pela escrita do arquivo.
        """
        capture_image.value = True

    def exit_calibrate_color(self):
        """
        Cancela a calibração em andamento:
          - Reseta o estado da calibração para "none"
          - Volta o ícone do botão para o lápis (✎)
          - Atualiza a mensagem de status
        """
        calibrate_color_status.value = "none"
        self.color_calibration_button_var.set("✎")
        self.set_calibration_status()

    @staticmethod
    def set_calibration_status():
        """
        Atualiza a variável compartilhada 'status' com uma mensagem
        que orienta o operador durante a calibração de cores.

        Cada código de cor (ex: "z-g" = zona verde, "l-bn" = linha preta normal)
        tem uma instrução específica para posicionamento do objeto a calibrar.

        Estados possíveis de calibrate_color_status:
          "calibrate" → pede para posicionar o objeto na câmera correta
          "check"     → mostra a imagem binária resultante para confirmar
          "none"      → calibração encerrada
        """
        color = calibrate_color_status.value

        # Dicionário: código → ([instrução "calibrate"], [instrução "check"])
        calib_map = {
            "z-g":   (["green evac-point", "center", "zone"],       ["green evac-point", "zone cam"]),
            "l-gz":  (["green evac-point", "top",    "line"],       ["green evac-point", "line cam"]),
            "z-r":   (["red evac-point",   "center", "zone"],       ["red evac-point",   "zone cam"]),
            "l-rz":  (["red evac-point",   "top",    "line"],       ["red evac-point",   "line cam"]),
            "l-bz":  (["black line",       "center", "line"],       ["black zone",       "line cam"]),
            "l-bn":  (["black line",       "top and bottom", "line"], ["black normal",   "line cam"]),
            "l-bv":  (["black line",       "top and bottom", "line"], ["black silver validation (LED off)", "line cam"]),
            "l-bvl": (["black line",       "top",    "line"],       ["black silver validation (LED on)",  "line cam"]),
            "l-bd":  (["black line at a ramp down", "top", "line"], ["black ramp down",  "line cam"]),
            "l-gl":  (["green marker",     "center", "line"],       ["green",            "line cam"]),
            "l-rl":  (["red goal tile",    "center", "line"],       ["red",              "line cam"]),
        }

        c = calibration_color.value   # cor atualmente selecionada para calibrar

        if color == "calibrate" and c in calib_map:
            ct = calib_map[c][0]
            status.value = (
                f'Please move the {ct[0]} to the {ct[1]} of the {ct[2]} camera and press ⎚'
            )
        elif color == "check" and c in calib_map:
            it = calib_map[c][1]
            status.value = f'This is the {it[0]} binary image ({it[1]}). Press ✓ to confirm'
        elif color == "none":
            status.value = "Stopped"

    def set_calibrate_color_status(self):
        """
        Cicla pelo fluxo de calibração a cada clique:
          none → calibrate → check → none
        Cada estado muda o ícone do botão e a mensagem de status.
        Ao iniciar a calibração, também inicia o timer se ainda não estiver rodando.
        """
        if calibrate_color_status.value == "none":
            calibrate_color_status.value = "calibrate"
            self.color_calibration_button_var.set("⎚")   # ícone "capturar"
            if run_start_time.value == -1:
                run_start_time.value = time.perf_counter()  # inicia o cronômetro
        elif calibrate_color_status.value == "calibrate":
            calibrate_color_status.value = "check"
            self.color_calibration_button_var.set("✓")   # ícone "confirmar"
        elif calibrate_color_status.value == "check":
            calibrate_color_status.value = "none"
            self.color_calibration_button_var.set("✎")   # ícone "editar" (inicial)
        self.set_calibration_status()   # atualiza mensagem no label de status

    def choose_color(self):
        """
        Cicla pela lista de cores/modos de calibração disponíveis.
        Só funciona quando não há calibração em andamento (estado "none").
        A cor atual é armazenada em calibration_color (variável compartilhada).
        """
        if calibrate_color_status.value != "none":
            return  # bloqueia troca de cor durante calibração ativa

        cycle = ["z-g", "l-gz", "z-r", "l-rz", "l-bz", "l-bn", "l-bv", "l-bvl", "l-bd", "l-gl", "l-rl"]
        cur = calibration_color.value
        # Próximo na lista (circular: após o último volta ao primeiro)
        nxt = cycle[(cycle.index(cur) + 1) % len(cycle)] if cur in cycle else cycle[0]
        calibration_color.value = nxt
        self.color_choose_button_var.set(nxt)  # atualiza o texto do botão
        self.set_calibration_status()           # atualiza instrução de status

    # ================================================================
    #  LOOP PRINCIPAL DE ATUALIZAÇÃO DA UI (chamado a cada 100 ms)
    # ================================================================

    def main(self, *args):
        """
        Coração da interface gráfica. Chamado inicialmente após app.main()
        e reagendado a si mesmo com self.after(100, self.main) ao final.
        Atualiza TODOS os elementos visuais com dados frescos dos processos.

        Fluxo a cada 100 ms:
          1. Lê variáveis compartilhadas (sensores, estado, timers)
          2. Atualiza os StringVar → os Labels se atualizam automaticamente
          3. Captura frames das câmeras e atualiza os CTkLabel de câmera
          4. Seleciona imagem do modelo 3D pelo yaw/pitch atual
          5. Desenha círculos de vítimas no Canvas
          6. Agenda a próxima chamada em 100 ms
        """
        global display_status

        # ── 1. Sensores de distância (ultrassônico / ToF) ─────────
        # sensor_one … sensor_seven são Value() de multiprocessing, atualizados
        # pelo processo serial_loop a partir dos dados recebidos via serial.
        self.label_sensor_1_var.set(f"{sensor_one.value:.0f} mm")    # frontal esquerdo
        self.label_sensor_2_var.set(f"{sensor_two.value:.0f} mm")    # frontal direito
        self.label_sensor_3_var.set(f"{sensor_three.value:.0f} mm")  # lateral esquerdo
        self.label_sensor_4_var.set(f"{sensor_four.value:.0f} mm")   # lateral direito
        self.label_sensor_5_var.set(f"{sensor_five.value:.0f} mm")   # frontal central
        self.label_sensor_6_var.set(f"{sensor_six.value:.0f} mm")    # traseiro
        self.label_sensor_7_var.set(f"{sensor_seven.value:.0f} mm")  # garra

        # ── 2. IMU (Unidade de Medição Inercial) ──────────────────
        # sensor_x = yaw (Norte/Sul) | sensor_y = pitch | sensor_z = roll
        self.label_sensor_x_var.set(f"{sensor_x.value} °")
        self.label_sensor_y_var.set(f"{sensor_y.value} °")
        self.label_sensor_z_var.set(f"{sensor_z.value} °")

        # ── 3. Indicadores de navegação ───────────────────────────
        self.label_angle_var.set(f"{line_angle.value} °")       # ângulo da linha guia
        self.label_gap_angle_var.set(f"{gap_angle.value:.2f} °")  # ângulo do gap

        # ── 4. Performance do sistema ─────────────────────────────
        self.label_cpu_var.set(f"CPU: {psutil.cpu_percent()} %")
        # cpu_percent() retorna uso médio de CPU de todos os núcleos (%)

        self.label_ips_c_var.set(f"IPS_C: {iterations_control.value}")
        # iterations_control = contador de iterações/segundo do process control_loop

        self.label_ips_s_var.set(f"IPS_S: {iterations_serial.value}")
        # iterations_serial  = contador de iterações/segundo do process serial_loop

        # ── 5. Mensagem de status ─────────────────────────────────
        # Só atualiza o texto (e o tamanho da fonte) se o status mudou,
        # para evitar reprocessamento gráfico desnecessário a cada frame.
        if display_status != status.value:
            # Mensagens longas (>60 chars) usam fonte menor para caber no espaço
            self.label_status_font.configure(size=12 if len(status.value) > 60 else 15)
            self.label_status_var.set(f"... {status.value} ...")
            display_status = status.value   # guarda para comparar no próximo ciclo

        # ── 6. Indicador de inclinação (rampa) ────────────────────
        ry = rotation_y.value
        # "none" = plano | "ramp_up" = subindo rampa | qualquer outro = descendo
        self.label_rotation_var.set("n" if ry == "none" else ("u" if ry == "ramp_up" else "d"))

        # ── 7. Seta de direção atual ──────────────────────────────
        td, obj = turn_dir.value, objective.value
        # Mapeia (direção, objetivo) → símbolo unicode de seta
        dir_map = {
            ("straight",    "follow_line"): "⇧",   # seguindo em frente
            ("left",        "follow_line"): "⇦",   # virando à esquerda
            ("right",       "follow_line"): "⇨",   # virando à direita
            ("turn_around", "follow_line"): "⟲",   # fazendo meia-volta
        }
        self.label_turn_dir_var.set(
            "II" if obj == "stop" else dir_map.get((td, obj), "⇧")
            # "II" = parado | get com default "⇧" = fallback para frente
        )

        # ── 8. Indicador de Hold (pausa manual) ───────────────────
        # switch.value == 0 → robô em hold (pausa) | != 0 → em execução
        self.label_hold_var.set("II" if switch.value == 0 else "▶")

        # ── 9. Timers (tempo decorrido) ───────────────────────────
        def fmt_timer(start_val, label_var):
            """
            Formata o tempo decorrido desde start_val (perf_counter) como mm:ss:cs.
            Se start_val == -1, o timer ainda não foi iniciado (não atualiza).
            """
            if start_val != -1:
                # timedelta formata como "H:MM:SS.ffffff"
                parts = (
                    str(timedelta(seconds=time.perf_counter() - start_val))
                    .replace(".", ":").split(":")  # substitui ponto decimal por ":"
                )
                try:
                    # parts[1]=minutos, parts[2]=segundos, parts[3][:2]=centésimos
                    label_var.set(f"{parts[1]}:{parts[2]}:{parts[3][:2]}")
                except IndexError:
                    pass  # ignora se o formato for inesperado

        fmt_timer(run_start_time.value,  self.label_timer_var)       # timer da rodada
        fmt_timer(zone_start_time.value, self.label_timer_zone_var)  # timer da zona

        # ── 10. Círculos de vítimas coletadas ─────────────────────
        alive = picked_up_alive_count.value   # número de vítimas vivas coletadas
        dead  = picked_up_dead_count.value    # número de vítimas mortas coletadas
        # 3 círculos: 2 posições para vivas (cinza claro) + 1 para morta (preto)
        create_circle(15, 15, 10, self.canvas, 2 if alive >= 1 else 1)
        create_circle(40, 15, 10, self.canvas, 2 if alive >= 2 else 1)
        create_circle(65, 15, 10, self.canvas, 3 if dead  >= 1 else 1)

        # ── 11. Câmera 1 — captura e exibe frame ──────────────────
        # cam1.capture_array() retorna numpy array (H, W, 3) em formato RGB888.
        # Não usa shared_memory; a captura acontece diretamente aqui na thread da UI.
        frame1 = self.cam1.capture_array()
        img_cam_1     = Image.fromarray(frame1)    # converte para PIL Image
        img_tks_cam_1 = ctk.CTkImage(img_cam_1, size=(CAM1_WIDTH, CAM1_HEIGHT))
        # CTkImage redimensiona automaticamente para (CAM1_WIDTH, CAM1_HEIGHT) = (320, 240)
        self.top_camera.configure(image=img_tks_cam_1)
        # configure(image=...) atualiza o Label sem recriar o widget

        # ── 12. Câmera 2 — captura e exibe frame (se disponível) ──
        if self.cam2:
            frame2 = self.cam2.capture_array()
            img_cam_2     = Image.fromarray(frame2)
            img_tks_cam_2 = ctk.CTkImage(img_cam_2, size=(CAM2_WIDTH, CAM2_HEIGHT))
            # Redimensiona para (CAM2_WIDTH, CAM2_HEIGHT) = (320, 200)
            self.bottom_camera.configure(image=img_tks_cam_2)

        # ── 13. Modelo 3D do robô ─────────────────────────────────
        if not testing_mode:
            # Calcula (yaw_ajustado, pitch_limitado) a partir dos dados do IMU
            rotation = get_yaw_pitch(sensor_x.value, sensor_y.value)
            # Busca a imagem pré-renderizada correspondente no dicionário
            self.modelImage.configure(image=model_map[rotation])

        # ── 14. Agenda o próximo ciclo de atualização ─────────────
        self.after(100, self.main)
        # self.after(ms, callback) é o equivalente tkinter de setTimeout(callback, ms)
        # Garante que a UI não bloqueie — a janela continua responsiva entre frames.


# =================================================================
#  PONTO DE ENTRADA DO PROGRAMA
# =================================================================

if __name__ == "__main__":
    """
    Bloco executado somente quando o script é chamado diretamente
    (não quando importado como módulo).
    
    Sequência de inicialização:
      1. Registra o tempo de início do programa
      2. Inicializa as câmeras
      3. Cria e inicia os processos paralelos (serial, controle)
      4. Cria a janela da UI e inicia o loop principal
    """

    # ── 1. Tempo de início ────────────────────────────────────────
    program_start_time.value = time.perf_counter()
    # perf_counter() é o relógio de alta resolução do Python (melhor que time.time())
    # Este valor é compartilhado com os outros processos via mp_manager

    # ── 2. Câmeras ────────────────────────────────────────────────
    cam1, cam2 = init_cameras()
    # cam2 pode ser None se não houver segunda câmera conectada

    # ── 3. Processos paralelos ────────────────────────────────────
    processes = [
        Process(target=serial_loop,  args=()),
        # serial_loop: lê os sensores de distância e IMU via serial (Arduino/ESP)
        # e atualiza sensor_one…sensor_z nas variáveis compartilhadas

        Process(target=control_loop, args=()),
        # control_loop: executa a lógica de navegação autônoma do robô
        # lê os sensores, decide direção, chama set_motors() indiretamente
        # via variáveis compartilhadas ou diretamente pelo gpiozero

        # NOTA: line_cam_loop e zone_cam_loop foram removidos.
        # Antes, esses processos capturavam frames e os passavam via shared_memory.
        # Agora, a captura ocorre diretamente no loop da UI (método main() acima).
        # Se você precisar de visão computacional em processo separado, use
        # multiprocessing.Queue ou shared_memory para trafegar os frames.
    ]

    # Inicia cada processo com uma pausa entre eles (evita conflito de recursos)
    for process in processes:
        process.start()
        print(process)   # exibe PID e nome do processo no terminal
        time.sleep(0.5)  # 500 ms de espera antes de iniciar o próximo

    # ── 4. Interface gráfica ──────────────────────────────────────
    app = App(cam1, cam2)   # instancia a janela principal passando as câmeras
    app.main()              # primeira chamada ao loop de atualização da UI
    app.mainloop()          # inicia o event loop do tkinter (bloqueia aqui até fechar)
    # mainloop() processa eventos de teclado, mouse e os callbacks de after()
    # até que destroy() seja chamado (botão ✕ → método exit())
