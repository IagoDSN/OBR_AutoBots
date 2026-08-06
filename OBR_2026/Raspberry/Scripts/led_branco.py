import time
import serial

import mp_manager as mgr

PORTA_SERIAL = "/dev/ttyACM0"   # confira com `ls /dev/tty*` antes/depois de plugar o Arduino
BAUD_RATE = 9600

CMD_LIGAR = b"1"
CMD_DESLIGAR = b"0"


def led_branco_loop():
    print("[LED] Processo de LED iniciado.")
    try:
        arduino = serial.Serial(PORTA_SERIAL, BAUD_RATE, timeout=1)
        time.sleep(2)  # Arduino reseta ao abrir a serial; espera estabilizar
    except serial.SerialException as e:
        print(f"[LED] Não consegui abrir a serial com o Arduino: {e}")
        return

    try:
        arduino.write(CMD_LIGAR)
        print("[LED] Comando LIGAR enviado ao Arduino.")

        while not mgr.terminate.is_set():
            # reenvia como "heartbeat": se o Arduino resetar no meio da prova,
            # ele volta pro estado ligado no próximo ciclo
            arduino.write(CMD_LIGAR)
            time.sleep(0.5)

    except Exception as e:
        print(f"[LED] Ocorreu um erro no loop do LED: {e}")

    finally:
        print("[LED] Desligando os LEDs com segurança...")
        try:
            arduino.write(CMD_DESLIGAR)
            time.sleep(0.05)
            arduino.write(CMD_DESLIGAR)  # manda 2x — serial pode perder byte
        finally:
            arduino.close()
        print("[LED] LEDs desligados.")


if __name__ == "__main__":
    led_branco_loop()