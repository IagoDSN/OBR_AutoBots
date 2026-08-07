"""
Teste isolado dos dois motores (esquerdo e direito).
Roda sem multiprocessing, sem camera, sem serial — só GPIO e motores.

Uso:
    python3 teste_motores.py
"""
import time

# Pi 5 precisa do backend lgpio; forçar ANTES de importar Motor/DigitalOutputDevice
from gpiozero import Device
from gpiozero.pins.lgpio import LGPIOFactory
Device.pin_factory = LGPIOFactory()

from motores import PonteHBTS7960

# Mesmos pinos usados no control.py
RPWM_ESQ, LPWM_ESQ, REN_ESQ, LEN_ESQ = 12, 13, 5, 6
RPWM_DIR, LPWM_DIR, REN_DIR, LEN_DIR = 18, 19, 20, 21

VELOCIDADE_TESTE = 50   # % — comece baixo, é só teste de bancada
DURACAO = 2             # segundos por movimento


def testar_motor(nome, motor, velocidade):
    print(f"[{nome}] frente {velocidade}%...")
    motor.set_velocidade(velocidade)
    time.sleep(DURACAO)

    print(f"[{nome}] parado...")
    motor.set_velocidade(0)
    time.sleep(1)

    print(f"[{nome}] ré {velocidade}%...")
    motor.set_velocidade(-velocidade)
    time.sleep(DURACAO)

    print(f"[{nome}] parado.")
    motor.set_velocidade(0)
    time.sleep(1)


def main():
    print("Inicializando motores...")
    motor_esq = PonteHBTS7960(RPWM_ESQ, LPWM_ESQ, REN_ESQ, LEN_ESQ)
    motor_dir = PonteHBTS7960(RPWM_DIR, LPWM_DIR, REN_DIR, LEN_DIR)
    print("Motores inicializados.\n")

    try:
        testar_motor("ESQUERDO", motor_esq, VELOCIDADE_TESTE)
        testar_motor("DIREITO", motor_dir, VELOCIDADE_TESTE)

        print("\nTestando os dois motores juntos (frente)...")
        motor_esq.set_velocidade(VELOCIDADE_TESTE)
        motor_dir.set_velocidade(VELOCIDADE_TESTE)
        time.sleep(DURACAO)
        motor_esq.set_velocidade(0)
        motor_dir.set_velocidade(0)

        print("Teste concluído.")
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário.")
    finally:
        motor_esq.parar()
        motor_dir.parar()
        motor_esq.fechar()
        motor_dir.fechar()
        print("Motores parados e GPIO liberado.")


if __name__ == "__main__":
    main()