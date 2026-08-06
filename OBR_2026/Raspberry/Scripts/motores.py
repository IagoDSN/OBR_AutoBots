import time
from gpiozero import Motor, DigitalOutputDevice
from line_cam import calcular_comando_motor, FRAME_WIDTH, DEBUG

CONTROL_HZ = 20  # mesmo ritmo do line_cam (sleep de 0.05s)

# --- Pinos dos motores (BCM) — confira contra sua fiacao real ---
PIN_DIR_FORWARD, PIN_DIR_BACKWARD, PIN_DIR_EN = 18, 19, 23
PIN_ESQ_FORWARD, PIN_ESQ_BACKWARD, PIN_ESQ_EN = 20, 21, 24


def controlar_motores(camera_ok, line_status, line_angle, stop_flag):
    """Processo de controle: le status/erro compartilhados pelo line_cam
    e aciona os motores. Motor/DigitalOutputDevice sao criados aqui dentro
    (nao no topo do modulo) pra nao serem reservados pelo processo pai
    antes do fork."""
    motor_dir = Motor(forward=PIN_DIR_FORWARD, backward=PIN_DIR_BACKWARD)
    en_dir = DigitalOutputDevice(PIN_DIR_EN, initial_value=True)

    motor_esq = Motor(forward=PIN_ESQ_FORWARD, backward=PIN_ESQ_BACKWARD)
    en_esq = DigitalOutputDevice(PIN_ESQ_EN, initial_value=True)

    center_x = FRAME_WIDTH // 2

    def aplicar_comando(vel_esq_pct, vel_dir_pct):
        motor_esq.value = max(-1.0, min(1.0, vel_esq_pct / 100.0))
        motor_dir.value = max(-1.0, min(1.0, vel_dir_pct / 100.0))

    def parar_motores():
        motor_esq.stop()
        motor_dir.stop()

    try:
        while not stop_flag.value:
            if camera_ok.value == 0:
                break

            status = line_status.value
            erro = line_angle.value
            vel_esq, vel_dir = calcular_comando_motor(status, erro, center_x)
            aplicar_comando(vel_esq, vel_dir)

            if DEBUG:
                print(f"[control] status={status} erro={erro:.1f} "
                      f"vel_esq={vel_esq:.1f} vel_dir={vel_dir:.1f}")

            time.sleep(1.0 / CONTROL_HZ)
    finally:
        parar_motores()