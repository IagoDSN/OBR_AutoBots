

import time
import control


TIMEOUT_SEGUNDOS = 0.5

FREQUENCIA_HZ = 50


def rodar(estado, evento_parar):
    control.iniciar_motores()
    print("[motor] motores iniciados")

    intervalo = 1.0 / FREQUENCIA_HZ

    try:
        while not evento_parar.is_set():
            direcao, erro, potencia, timestamp = estado.ler()
            idade_do_dado = time.time() - timestamp

            if direcao is None or idade_do_dado > TIMEOUT_SEGUNDOS:
                # Ou a câmera nunca achou a linha, ou o dado tá velho demais
                # (câmera travou) -> por segurança, para o robô.
                control.parar()
            else:
                control.aplicar_comando(direcao, potencia)

            time.sleep(intervalo)
    finally:
        control.parar()
        print("[motor] processo encerrado")
